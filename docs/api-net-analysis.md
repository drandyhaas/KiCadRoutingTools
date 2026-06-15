# Net Analysis API (`net_queries`, `connectivity`)

- **`net_queries.py`** — questions about *nets*: which match a pattern,
  which are differential pairs, which are power nets, what order to route
  them in (MPS), how long a route is.
- **`connectivity.py`** — questions about *copper*: which segments are
  connected, where stubs end, which pads still need connecting.

## `net_queries.py`

### Pattern matching

#### `matches_net_filter`

```python
matches_net_filter(net_name: str, patterns: List[str]) -> bool
```

fnmatch-style wildcards (`*`, `?`), with `!` prefix for exclusion. A name
matches if it matches at least one inclusion pattern (when any exist) and no
exclusion pattern. Exclusion-only lists match everything not excluded.

#### `expand_net_patterns`

```python
expand_net_patterns(pcb_data, patterns: List[str],
                    exclude_unconnected: bool = True) -> List[str]
```

Expands patterns to the sorted list of actual net names on the board,
dropping `unconnected-*` nets by default.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import expand_net_patterns

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
print(expand_net_patterns(pcb, ['*lvds_rx1*']))
print(expand_net_patterns(pcb, ['*lvds*', '!*_N']))   # only the P sides
```

### `identify_power_nets`

```python
identify_power_nets(pcb_data, patterns: List[str],
                    widths: List[float]) -> Dict[int, float]
```

Pattern-based power net detection (the engine behind `--power-nets`).
`patterns[i]` gets `widths[i]`; the first matching pattern wins, so order
patterns most-specific first. Returns net_id → track width, ready to assign
to `GridRouteConfig.power_net_widths`.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import identify_power_nets

pcb = parse_kicad_pcb('kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb')
widths = identify_power_nets(pcb, ['*GND*', '*VCC*', '+*V*'], [0.5, 0.4, 0.4])
for net_id, w in sorted(widths.items()):
    print(f"net {net_id:3d} {pcb.nets[net_id].name:12s} -> {w}mm")
```

For datasheet-driven analysis (when net names aren't trustworthy), use the
`/analyze-power-nets` skill instead — see [Power Nets](power-nets.md).

### `find_differential_pairs`

```python
find_differential_pairs(pcb_data, patterns: List[str])
    -> Dict[str, DiffPairNet]
```

Finds complete P/N pairs among nets matching the patterns, keyed by base
name. Recognized suffix conventions:

| Style | Example pair |
|-------|--------------|
| `_P` / `_N` | `LVDS0_P`, `LVDS0_N` |
| `P` / `N` (no underscore) | `CLKP`, `CLKN` |
| `+` / `-` | `USB+`, `USB-` |
| `_t` / `_c` (DDR true/complement, case-insensitive) | `DQS0_t`, `DQS0_c`, `CK_T`, `CK_C` |
| `_t_X` / `_c_X` (with channel suffix) | `DQS0_t_A`, `DQS0_c_A` |
| `_tX` / `_cX` (no-separator channel) | `DQS0_TA`, `DQS0_CA` |
| `DP` / `DM`, `DPLUS` / `DMINUS` (USB) | `USB_DP`, `USB_DM` |

KiCad's `Net-(<ref>-<pin>)` auto-names are unwrapped first, so a buried suffix
like `Net-(U12-USB_D+)` / `Net-(U12-USB_D-)` still pairs.

Nets only pair **within the same suffix style**: `CLK+` will never pair with
an unrelated `CLK_N`. Only complete pairs (both sides found) are returned;
each value is a [`DiffPairNet`](api-routing-config.md#diffpairnet).

```python
from kicad_parser import parse_kicad_pcb
from net_queries import find_differential_pairs

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
pairs = find_differential_pairs(pcb, ['*lvds*'])
for base, pair in sorted(pairs.items())[:5]:
    print(f"{base}: P={pair.p_net_name} (id {pair.p_net_id}), "
          f"N={pair.n_net_name} (id {pair.n_net_id})")
```

The single-net helper is `extract_diff_pair_base(net_name)`, returning
`(base_name, is_positive, style)` or `None`.

#### `find_single_ended_nets`

```python
find_single_ended_nets(pcb_data, patterns, exclude_net_ids=None)
    -> List[Tuple[str, int]]   # (net_name, net_id)
```

Pattern-matched nets minus an exclusion set — typically the diff-pair net
IDs you just found.

### Routing-status queries

```python
get_all_unrouted_net_ids(pcb_data) -> List[int]
```

Net IDs that still need work: ≥ 2 pads and either no segments, or multiple
disconnected segment groups, or a single group that doesn't reach all pads.

```python
calculate_route_length(segments, vias=None, pcb_data=None) -> float
calculate_via_barrel_length(vias, pcb_data) -> float
```

Total routed length in mm. With `pcb_data` (and a stackup), via barrel
lengths are included — this matches KiCad's length measurements.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import get_all_unrouted_net_ids, calculate_route_length

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
print(f"{len(get_all_unrouted_net_ids(pcb))} nets still unrouted")

net = next(n for n in pcb.nets.values() if n.name.endswith('lvds_rx1_8_P'))
segs = [s for s in pcb.segments if s.net_id == net.net_id]
vias = [v for v in pcb.vias if v.net_id == net.net_id]
print(f"{net.name}: {calculate_route_length(segs, vias, pcb):.2f} mm")
```

### `expand_pad_layers`

```python
expand_pad_layers(pad_layers: List[str], routing_layers: List[str])
    -> List[str]
```

Expands KiCad wildcard layer specs (through-hole pads carry `'*.Cu'`) to the
actual routing layer names.

### Position/geometry queries

```python
find_pad_nearest_to_position(pcb_data, net_id, x, y) -> Optional[Pad]
find_pad_at_position(pcb_data, x, y, tolerance=0.01) -> Optional[Pad]
get_chip_pad_positions(pcb_data, net_ids, min_pads=4)
    -> List[Tuple[float, float, str]]     # pseudo-stub positions on chips
get_source_chip_center(pcb_data, source_pads) -> Optional[Tuple[float, float]]
find_containing_or_nearest_bga_zone(point, bga_zones) -> Optional[Tuple]
compute_routing_aware_distance(target_free_end, source_chip_center,
                               bga_zone) -> float   # path length around the BGA
```

### MPS net ordering

#### `compute_mps_net_ordering`

```python
compute_mps_net_ordering(pcb_data, net_ids: List[int],
                         center=None, diff_pairs=None,
                         use_boundary_ordering=True,
                         bga_exclusion_zones=None,
                         reverse_rounds=False,
                         crossing_layer_check=True,
                         return_extended_info=False,
                         use_segment_intersection=None)
    -> List[int] | MPSResult
```

Orders nets to minimize crossing conflicts (Maximum Planar Subset): project
each net's endpoints onto the chip boundary, detect pairs whose endpoints
interleave (they must cross), then greedily order least-conflicted first.
P/N nets of a `diff_pairs` dict are treated as single units.

With `return_extended_info=True` you get an `MPSResult` with the full
picture: `ordered_ids`, `conflicts` (layer-filtered), `geometric_conflicts`
(all crossings), `unit_layers`, `unit_to_nets`, `unit_names`,
`round_assignments`, and `num_rounds`.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import find_differential_pairs, compute_mps_net_ordering

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
pairs = find_differential_pairs(pcb, ['*lvds_rx1*'])
net_ids = [i for p in pairs.values() for i in (p.p_net_id, p.n_net_id)]
order = compute_mps_net_ordering(pcb, net_ids, diff_pairs=pairs)
print("route in this order:", [pcb.nets[i].name for i in order[:6]], "...")
```

Algorithm details and strategy comparison: [Net Ordering](net-ordering.md).

## `connectivity.py`

### `find_connected_groups`

```python
find_connected_groups(segments, tolerance=0.01,
                      layer_aware=True, vias=None)
    -> List[List[Segment]]
```

Groups segments into connected components (union-find with spatial hashing,
O(n)). With `layer_aware=True` (default), segments on different layers only
connect where a via from `vias` sits at the shared position — pass the
net's vias or stubs that merely overlap in XY will be split correctly.

```python
from kicad_parser import parse_kicad_pcb
from connectivity import find_connected_groups

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
net = next(n for n in pcb.nets.values() if n.name.endswith('lvds_rx1_8_P'))
segs = [s for s in pcb.segments if s.net_id == net.net_id]
vias = [v for v in pcb.vias if v.net_id == net.net_id]
groups = find_connected_groups(segs, vias=vias)
print(f"{net.name}: {len(segs)} segments in {len(groups)} connected group(s)")
```

One group = fully connected copper; more = stubs/partial routing.

### Stub queries

```python
find_stub_free_ends(segments, pads, tolerance=0.05)
    -> List[Tuple[float, float, str]]   # (x, y, layer)
```

Endpoints of a connected segment group that touch neither another segment
nor a pad — i.e. where the router should continue from.

```python
get_stub_direction(segments, stub_x, stub_y, stub_layer, tolerance=0.05)
    -> Tuple[float, float]            # unit vector pad -> free end
get_stub_segments(pcb_data, net_id, stub_x, stub_y, stub_layer,
                  tolerance=0.05) -> List[Segment]
get_stub_vias(pcb_data, net_id, stub_segments, tolerance=0.05) -> List[Via]
calculate_stub_length(pcb_data, net_id) -> float
calculate_stub_via_barrel_length(stub_vias, stub_layer, pcb_data) -> float
get_stub_endpoints(pcb_data, net_ids) -> List[Tuple[float, float, str]]
get_net_stub_centroids(pcb_data, net_id) -> List[Tuple[float, float]]
```

### Endpoint selection for routing

```python
get_net_endpoints(pcb_data, net_id, config, use_stub_free_ends=False)
    -> (sources, targets, error_message)
```

The router's source/target picker. Handles all the cases: two stub groups,
one stub group plus bare pads, pad-to-pad, already connected (returns an
error string). `sources`/`targets` are `(gx, gy, layer_idx, orig_x, orig_y)`
tuples — **grid** coordinates plus the original mm position.
`use_stub_free_ends=True` restricts to stub tips (diff-pair mode).

```python
get_net_routing_endpoints(pcb_data, net_id) -> List[Tuple[float, float]]
```

Simplified two-point (source/target centroid) version used for MPS crossing
detection.

```python
get_multipoint_net_pads(pcb_data, net_id, config) -> Optional[List[Tuple]]
```

Returns 3+ endpoint tuples if the net needs multi-point (MST) routing, else
`None`.

```python
normalize_endpoints_by_component(pcb_data, sources, targets, net_id)
    -> (sources, targets)
```

Swaps source/target so the source is always on the alphabetically-first
component — keeps crossing detection consistent across nets.

### Connectivity through zones

```python
get_zone_connected_pad_groups(segments, vias, pads, zones,
                              routing_layers=None) -> Dict[int, int]
```

Pad index → component ID, where pads sharing an ID are already connected
through tracks **or copper zones/planes**. Used to skip MST edges between
pads a plane already joins. Pads with no usable copper layer get unique
negative IDs.

### Graph/geometry helpers

```python
compute_mst_edges(points, use_manhattan=False)
    -> List[Tuple[int, int, float]]     # (index_a, index_b, length)
compute_mst_segments(points)
    -> List[Tuple[Tuple, Tuple]]        # ((x1, y1), (x2, y2))
find_farthest_pad_pair(pads) -> Tuple[int, int]   # Manhattan distance
find_closest_point_on_segments(segments, target_x, target_y, target_layers)
    -> (x, y, layer, distance)          # tap point for multi-point routing
find_connected_segment_positions(pcb_data, start_x, start_y, net_id,
                                 tolerance=0.1, layer=None) -> set
find_connected_segments(pcb_data, start_x, start_y, net_id) -> List[Segment]
segments_intersect(a1, a2, b1, b2) -> bool   # shared endpoints don't count
is_edge_stub(pad_x, pad_y, bga_zones) -> bool
```

### Example: stub anatomy of a fanned-out net

```python
from kicad_parser import parse_kicad_pcb
from connectivity import find_connected_groups, find_stub_free_ends

pcb = parse_kicad_pcb('kicad_files/fanout_output.kicad_pcb')
net = next(n for n in pcb.nets.values()
           if len(n.pads) >= 2 and any(s.net_id == n.net_id for s in pcb.segments))

segs = [s for s in pcb.segments if s.net_id == net.net_id]
vias = [v for v in pcb.vias if v.net_id == net.net_id]
for i, group in enumerate(find_connected_groups(segs, vias=vias)):
    ends = find_stub_free_ends(group, net.pads)
    print(f"{net.name} group {i}: {len(group)} segments, free ends: {ends}")
```

## Gotchas

- **Layer-aware grouping needs the vias.** `find_connected_groups(segs)`
  without `vias` treats a layer change as a break even if a via is there.
- **First-match-wins in `identify_power_nets`** — order patterns from
  specific to general.
- **Suffix styles never mix** in diff-pair detection; if your P/N naming is
  unconventional, use the `/identify-diff-pairs` skill (pin-function based).
- **Grid vs mm**: `get_net_endpoints` / `get_multipoint_net_pads` return
  grid coordinates (plus originals); most other functions here are mm.

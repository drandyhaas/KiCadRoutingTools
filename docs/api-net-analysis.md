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

#### `nets_for_components`

```python
nets_for_components(pcb_data, patterns, *, mode='any', match='glob',
                    exclude_patterns=None) -> ComponentNetSelection
```

The nets touching a set of footprints — the engine behind `route.py --component`
and the GUI's Comp Filter, shared so a reference cannot select different nets on
the two fronts.

`match` decides what a **bare** token means; a token carrying `*`, `?` or `[` is
an fnmatch glob under both. Matching is case-insensitive.

| `match` | bare token | `'U1'` matches `U10`? | used by |
|---|---|---|---|
| `'glob'` | exact reference | no | CLI, plan replay |
| `'substring'` | substring | yes | GUI Comp Filter (narrow-as-you-type) |

`mode` decides which nets of the matched footprints are selected:

| `mode` | selects |
|---|---|
| `'any'` | net has ≥1 pad on a matched footprint (default) |
| `'between'` | net reaches ≥2 **distinct** matched footprints — the wires between the selected parts |
| `'internal'` | **every** pad of the net is on a matched footprint — nets that never leave the block |

`'internal'` is near-empty for a single selected footprint; that is the correct
answer, not a bug. `exclude_patterns` (e.g. `POWER_NET_EXCLUSION_PATTERNS`) drops
matching nets and reports them in `excluded_names`, so a caller can say what it
dropped instead of dropping it silently.

Returns a `ComponentNetSelection` with `net_names`, `net_ids`, `matched_refs`,
`unmatched_patterns` and `excluded_names`. `unmatched_patterns` exists so a
typo'd reference is reportable: selecting by component and getting nothing back
is otherwise indistinguishable from "this part has no routable nets".

```python
from kicad_parser import parse_kicad_pcb
from net_queries import nets_for_components

pcb = parse_kicad_pcb('kicad_files/splitflap_driver.kicad_pcb')

sel = nets_for_components(pcb, ['U1'])
print(len(sel.net_names), sel.matched_refs)          # U1 only -- not U10

sel = nets_for_components(pcb, ['U*'])               # glob: the whole U series
print(len(sel.matched_refs), 'footprints')

sel = nets_for_components(pcb, ['U1'], match='substring')
print(sel.matched_refs)                              # ['U1', 'U10'] -- GUI behaviour

sel = nets_for_components(pcb, ['U1', 'ZZ99'])
print(sel.unmatched_patterns)                        # ['ZZ99'] -- a typo, reportable
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
| `_PX` / `_NX` (indexed, index kept in base) | `FE_CLK_P0`, `FE_CLK_N0` |
| `P` / `N` (no underscore) | `CLKP`, `CLKN` |
| `+` / `-` (not a `Net-(<ref>-±)` passive terminal) | `USB+`, `USB-` |
| `_t` / `_c` (DDR true/complement, case-insensitive) | `DQS0_t`, `DQS0_c`, `CK_T`, `CK_C` |
| `_t_X` / `_c_X` (with channel suffix) | `DQS0_t_A`, `DQS0_c_A` |
| `_tX` / `_cX` (no-separator channel) | `DQS0_TA`, `DQS0_CA` |
| `DP` / `DM` / `DN`, `DPLUS` / `DMINUS` (USB) | `USB_DP`, `USB_DM`, `USB_DN` |

For the indexed `_PX` / `_NX` style the trailing index stays in the base name,
so `FE_CLK_P0` pairs only with `FE_CLK_N0` (never `FE_CLK_N1`). For the USB
`DP`/`DM`/`DN` style, `P` is positive and both `M` and `N` are the negative
half, so `/USB_DP` pairs with either `/USB_DM` or `/USB_DN`.

KiCad's `Net-(<ref>-<pin>)` auto-names are unwrapped first, so a buried suffix
like `Net-(U12-USB_D+)` / `Net-(U12-USB_D-)` still pairs. When the unwrapped
pin path is hierarchical (`Net-(U12-GPIO19/.../USB_D-)`, KiCad escapes `/` as
`{slash}`), only the **leaf** segment is used, so the two halves pair even when
their chip-internal prefixes differ (`GPIO19` vs `GPIO20`). User-named
hierarchical nets (not `Net-(...)` auto-names) keep their full path, so
`/bank1/CLK_N` never pairs with `/bank2/CLK_P`.

A bare `+`/`-` suffix on a `Net-(<ref>-+)` / `Net-(<ref>--)` auto-name is **not**
treated as a differential pair: that is a 2-terminal passive's polarity pad
(buzzer, LED, etc.), not a coupled signal. A genuine pair is `FOO+`/`FOO-`,
never `FOO-+`/`FOO--`.

Nets only pair **within the same suffix style**: `CLK+` will never pair with
an unrelated `CLK_N`. Only complete pairs (both sides found) are returned;
each value is a [`DiffPairNet`](api-routing-config.md#diffpairnet).

A pair is selected when **either** half matches the patterns, and the pattern
is matched against the full net name, its leaf (last `/`-separated segment),
the pair base name, and the base name's leaf. So a one-sided glob like `*_P`,
or an explicit base name like `/DVI_CK`, selects the **whole** pair — even for
hierarchical (slash-separated) net names such as `/FPGA-DDR3L/CK0_P`.

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

### Per-net and pin-pair length

```python
net_copper_length(pcb_data, net_id, include_vias=True) -> float
net_copper_lengths(pcb_data, net_ids=None, include_vias=True) -> Dict[int, float]
```

Ask "how long is net N?" directly from a parsed board, without hand-slicing
the segment lists. `calculate_route_length` above takes a **segment list**, not
a `(pcb_data, net_id)` pair — passing it a `PCBData` raises (#489 §7).
`net_copper_lengths` measures many nets in one pass over the copper.

Both report **total net copper**. That equals the signal path only on a clean
point-to-point net; on a multipoint/fly-by net, or one carrying a stub, it is
the sum of every branch and matches no path at all.

```python
pin_pair_path_length(pcb_data, net_id, pad_a, pad_b, tolerance=0.02)
    -> Optional[float]
```

Shortest **copper path between two pads** — the from-to measurement length
matching needs. Walks a weighted graph over the net's own copper: segments
carry their length, coincident endpoints and T-junctions join at zero cost,
and a via crosses its span at the barrel length from the stackup. Pads and vias
attach by copper **overlap** (a track ends anywhere inside them, not on their
centre), and their own pad/barrel copper is not charged — matching how KiCad
measures track length.

Returns `None` when no track path joins the two pads: an unrouted or broken
net, or one whose pads meet only through a **zone** (pours are not traversed).
`0.0` means the pads' copper touches directly.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import net_copper_length, pin_pair_path_length

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
net = next(n for n in pcb.nets.values() if n.name == 'Net-(U2A-DATA_4)')
pads = pcb.pads_by_net[net.net_id]
total = net_copper_length(pcb, net.net_id)
path = pin_pair_path_length(pcb, net.net_id, pads[0], pads[1])
print(f"{net.name}: {total:.2f} mm total copper, "
      f"{'no track path' if path is None else f'{path:.2f} mm pin-to-pin'}")
# Net-(U2A-DATA_4): 25.99 mm total copper, 23.52 mm pin-to-pin
# -> 2.5 mm of that net's copper is branch/stub, not on the signal path.
```

### Ground domains

```python
resolve_gnd_net_id(pcb_data, preferred_name=None) -> Tuple[Optional[int], Optional[str]]
resolve_ground_domains(pcb_data) -> Dict[int, Set[str]]
ground_domain_bridges(pcb_data) -> List[Dict]
resolve_return_net_id(pcb_data, net_id=None, preferred_name=None)
    -> Tuple[Optional[int], Optional[str]]
describe_ground_domains(pcb_data, preferred_name=None) -> Optional[str]
```

`resolve_gnd_net_id` gives the board-global ground (explicit name > exact `GND` >
the GND family). Use `resolve_return_net_id` for **return-via placement**: it
returns the ground net a given signal should return to, which on a split-ground
board is its own domain rather than one arbitrary net (#489 §5).

`resolve_ground_domains` maps each ground-family net to the components returning
to it. `ground_domain_bridges` finds the single-point ties between domains
(populated 2-pad parts with one pad on each ground — ferrites, 0 Ω links, net
ties), which are excluded from domain membership so it cannot leak across the
split. `describe_ground_domains` returns a one-shot warning string when a board
has several domains and the caller has not disambiguated, else `None`.

`resolve_return_net_id` is identical to `resolve_gnd_net_id` whenever the board
has fewer than two ground domains or a `preferred_name` is given, and falls back
to it for any signal whose domain is ambiguous.

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
get_chip_pad_positions(pcb_data, net_ids, min_pads=4)
    -> List[Tuple[float, float, str]]     # pseudo-stub positions on chips
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

### Connectivity through zones

```python
get_copper_connected_terminal_groups(pcb_data, net_id, pad_info)
    -> Dict[int, int]
```

`pad_info` index → component ID for multipoint terminals (rows as returned
by `get_multipoint_net_pads`; real pads or stub free-ends), grouped by the
net's **existing copper** using the authoritative overlap-aware definition
(`check_net_connectivity`: cap overlap, T-junctions, zones, pad outlines).
Drives the multipoint component MST — copper the checker already grades
connected is never re-tapped (issue #317). Terminals not tied to any copper
get unique negative IDs.

```python
compute_component_mst_edges(positions, components)
    -> List[Tuple[int, int, float]]     # (index_a, index_b, length)
```

Minimum spanning tree over connected **components** of terminals: joining N
components takes exactly N−1 edges, each realized by the closest terminal
pair (Manhattan) between the two components it joins.

### Graph/geometry helpers

```python
compute_mst_edges(points, use_manhattan=False)
    -> List[Tuple[int, int, float]]     # (index_a, index_b, length)
compute_mst_segments(points)
    -> List[Tuple[Tuple, Tuple]]        # ((x1, y1), (x2, y2))
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

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
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

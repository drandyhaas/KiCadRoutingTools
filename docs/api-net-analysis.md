# Net Analysis API (`net_queries`, `connectivity`)

- **`net_queries.py`** — questions about *nets*: which match a pattern,
  which are differential pairs, which are power nets, what order to route
  them in (MPS), how long a route is.
- **`connectivity.py`** — questions about *copper*: which segments are
  connected, where stubs end, which pads still need connecting.

Examples run from the repository root with the engine directory on the
import path: `PYTHONPATH=py_router python3 <your script>` (see the
[overview](python-api.md)).

## `net_queries.py`

### Pattern matching

#### `matches_net_filter`

```python
matches_net_filter(net_name: str, patterns: List[str]) -> bool
```

fnmatch-style wildcards (`*`, `?`), with `!` prefix for exclusion. A name
matches if it matches at least one inclusion pattern (when any exist) and no
exclusion pattern. Exclusion-only lists match everything not excluded.

Matching is case-sensitive and sheet-path aware (`net_pattern_matches`,
#493): a pattern with no `/` (`GND`, `VCC*`) also matches the last path
component of a hierarchical net, so `GND` matches `/GND` and `/Analog/GND`;
a pattern with a `/` is matched against the full name only. A `!`-pattern
that names a net verbatim is that active-low net, not an exclusion (`!RESET`),
and `\!FOO` is always the literal name `!FOO` (#177).

#### `expand_net_patterns`

```python
expand_net_patterns(pcb_data, patterns: List[str],
                    exclude_unconnected: bool = True,
                    quiet: bool = False) -> List[str]
```

Expands patterns to the actual net names on the board, with the same
matching rules as `matches_net_filter`, dropping `unconnected-*` nets from
wildcards by default (naming one literally still selects it). Patterns apply
in order: each wildcard adds its matches sorted, an exclusion removes what
it matches (and keeps it out of later patterns), and an exclusion-only list
implies a leading `*`. A literal naming no net is kept as given, with a
warning. It prints how many nets each wildcard matched; `quiet=True` prints
only what is wrong with a pattern (it matched nothing, or an unqualified
exclusion also took a namesake), for a caller that resolves many patterns and
reports the totals itself.

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
`patterns[i]` gets `widths[i]` (unequal lengths raise `ValueError`); each
pattern is a case-sensitive fnmatch against the full net name, and the first
matching pattern wins, so order patterns most-specific first. Returns
net_id → track width, ready to assign to `GridRouteConfig.power_net_widths`.

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
name (when two conventions share a base, the second pair is keyed
`<base>(<style>)`). Recognized suffix conventions:

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

Pattern-matched nets (case-sensitive fnmatch on the full name) minus an
exclusion set — typically the diff-pair net IDs you just found.

### Routing-status queries

```python
get_all_unrouted_net_ids(pcb_data) -> List[int]
```

The router's candidate list for stub-proximity costs: net IDs with ≥ 2 pads
and no segments, several disconnected segment groups (via-aware), or a single
group — which it counts too, without testing whether that group reaches
every pad. So on a routed board it returns every multi-pad net; it is not a
connectivity grade. To ask whether a net is connected, use
`check_connected.check_net_connectivity`.

```python
calculate_route_length(segments, vias=None, pcb_data=None) -> float
calculate_via_barrel_length(vias, pcb_data) -> float
```

Total routed length in mm. With `pcb_data` (and a stackup), via barrel
lengths are included — this matches KiCad's track + via length. It is copper
only: KiCad also adds each pad's **pad-to-die** length (`Pad.die_length`,
#1248), which `net_match_length` below includes and this does not.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import get_all_unrouted_net_ids, calculate_route_length

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
print(f"{len(get_all_unrouted_net_ids(pcb))} candidate nets")

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

Each via is charged as **KiCad's length DRC** charges it (#1313,
`kicad_via_length`): the barrel between the outermost and innermost of its
layers where a same-net track has an endpoint exactly at the via, or a
same-net pad is centred on it (a via-in-pad joins the pad's layer). One joined
layer charges nothing. A via-in-pad on F.Cu whose track leaves on In1.Cu is
charged F.Cu→In1.Cu, not the whole stack. On routed_output (10 layers,
1.59 mm) this matches kicad-cli 10.0.0's `length_out_of_range` actuals to
0.1 µm on every routed net it reported. `calculate_route_length` — the
router's own measure, used for meander sizing — still charges the full span.

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
measures track length. A **via-in-pad** — a via whose centre lies in the pad —
joins that pad on every layer both carry, so a via centred in an SMD pad whose
tracks leave on an inner layer reaches its pad through the barrel (#1314;
before, the pad met only track ends and such a net read as not joined).

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

```python
net_pads_joined(pcb_data, net_id, pads=None, tolerance=0.02) -> bool
net_match_length(pcb_data, net_id, mode='total', tolerance=0.02,
                 include_pad_to_die=True) -> Optional[float]
```

`net_copper_length` reports whatever copper a net has, routed or not: a net
with no copper reads `0.0` and one carrying only fanout stubs reads the stub
length. Neither is the length of a signal, so a **length-match grade must not
use it raw** (#1246). `net_pads_joined` asks the question first — are all of
the net's copper pads (NPTH and paste-only pads excluded) joined to each other
by its track copper, on the same graph `pin_pair_path_length` walks (zones are
not traversed)?

`net_match_length` is the one measurement every length-match grader uses
(`board_score.py --length-groups`, the review-routed-board recipe): total net
copper (`mode='total'`) only once `net_pads_joined` holds, or the path between
the net's first two pads (`mode='pin_pair'`). `None` in either mode means "not
routed yet" (unrouted, stub-only, broken or plane-only), never `0.0`.

Both modes add the pads' **pad-to-die** length, as KiCad does (#1248): every
pad of the net in `'total'` mode (KiCad's length/skew DRC), the measured pad
pair in `'pin_pair'` mode (KiCad's tuner adds its start and end pads'). Pass
`include_pad_to_die=False` for copper only.

```python
pad_to_die_length(pads) -> float            # mm, sum of Pad.die_length
pad_to_die_delay(pads) -> float             # ps, sum of Pad.die_delay
net_pad_to_die_length(pcb_data, net_id) -> float   # every pad of the net
net_pad_to_die_delay(pcb_data, net_id) -> float
net_match_pad_to_die(pcb_data, net_id, mode='total', attr='die_length') -> float
```

`net_match_pad_to_die` is the term `net_match_length` adds in `mode`; with
`attr='die_delay'` it sums the same pads' delays (ps) instead.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import net_copper_length, net_match_length

pcb = parse_kicad_pcb('kicad_files/orangecrab_ext_pll.kicad_pcb')
nid = next(i for i, n in pcb.nets.items() if n.name == 'RAM_D8')
print(f"{net_copper_length(pcb, nid):.2f} mm of copper, "
      f"match length {net_match_length(pcb, nid)}")
# 3.64 mm of copper, match length None
# -> fanout stubs only: the net is not routed, so it has no length to match.
```

### Differential-pair copper: coupling and skew (#1303)

```python
diff_pair_coupling(pcb_data, p_net_id, n_net_id, gap_min=None, gap_max=None,
                   epsilon=KICAD_DRC_EPSILON_MM, index=None) -> dict
diff_pair_skew(pcb_data, p_net_id, n_net_id, tolerance=0.02) -> dict
diff_pair_copper(pcb_data, p_net_id, n_net_id, gap_min=None, gap_max=None,
                 index=None) -> dict        # both, rounded to 0.1 um
CopperIndex(pcb_data)                       # build once per board state
```

What a pair's copper delivers, measured as KiCad's DRC measures it. Coupling
is KiCad 10.0.0's diff-pair coupling test (`drc_test_provider_diff_pair_
coupling.cpp`) in KiCad's own integer geometry (`kicad_seg.py`): for every P
track and every N track on one layer whose lines are parallel (they do not
meet inside the int32 coordinate range -- a few nm off parallel still counts),
the parts facing each other are a candidate; its gap is their distance minus
the two half widths; it counts when nothing else lies on the one line KiCad
checks between P and N (a track or via touching the span's ends, or a pad a
parent track leaves, does not block) and, given a window, the gap is in
`[gap_min - 0.5 um, gap_max + 0.5 um]`. Returns `coupled_mm`, `length_p_mm` /
`length_n_mm` (tracks only, as KiCad's test sums them), `uncoupled_mm` (=
`max(P, N) - coupled`, KiCad's `diff_pair_uncoupled` actual), `coupled_by_layer`,
`layers_p` / `layers_n`, and `arc_chords` (tracks parsed from `(arc ...)`, which
the parser linearises -- nonzero means the figures are approximate).
`diff_pair_skew` is each member's `net_match_length(mode='total')` and their
difference, `None` while a member's pads are not joined. On a routed imx6ulz
board and a ten-case fixture both matched kicad-cli 10.0.0's actuals to 0.1 um.

```python
net_signal_length(pcb_data, net_id, tolerance=0.02) -> Optional[float]
```

What a net's copper will measure once the post-route cleanup has trimmed it,
read off a board that still carries the trimmings (#1303): the pad-to-pad path
of a 2-pad net (`pin_pair_path_length`; a loop collapses to it), the copper of
a larger net with every dead-end branch trimmed, plus every pad's pad-to-die;
`None` while the pads are not joined. Intra-pair matching measures its two legs
with it.

The gap window is the caller's: `check_drc` and `board_score` pass the pair's
binding `diff_pair_gap` (`design_rules.DesignRules`), as KiCad does;
route_diff passes the gap it routed the pair at, up to 1.5x
(`diff_pair_custody.measure_pair_copper`), because with no maximum KiCad also
counts single-ended legs that happen to run parallel at 3-6x the gap.

```python
from kicad_parser import parse_kicad_pcb
from net_queries import diff_pair_copper

pcb = parse_kicad_pcb('kicad_files/rp2350_fpga_eensy_prePlane.kicad_pcb')
ids = {n.name: i for i, n in pcb.nets.items()}
r = diff_pair_copper(pcb, ids['/RP2354A/USB_D+'], ids['/RP2354A/USB_D-'])
print(f"coupled {r['coupled_mm']} mm, uncoupled {r['uncoupled_mm']} mm, "
      f"skew {r['skew_mm']} mm")
# coupled 1.8 mm, uncoupled 0.7294 mm, skew 0.0366 mm
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
find_connected_groups(segments, tolerance=COINCIDENCE_TOL,
                      layer_aware=True, vias=None)
    -> List[List[Segment]]
```

`COINCIDENCE_TOL` (0.02 mm) is the engine's one endpoint-coincidence
radius (#320); `find_stub_free_ends` and `find_connected_segment_positions`
below default to it too.

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
find_stub_free_ends(segments, pads, tolerance=0.05,
                    coincidence_tol=COINCIDENCE_TOL)
    -> List[Tuple[float, float, str]]   # (x, y, layer)
```

Endpoints of a connected segment group that touch neither another segment
nor a pad — i.e. where the router should continue from. `tolerance` is the
pad-attach radius (an end this close to a pad centre is on the pad);
`coincidence_tol` decides whether two segment ends meet.

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
                                 tolerance=None, layer=None) -> set
    # tolerance None = COINCIDENCE_TOL; crossing layers needs a same-net via
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

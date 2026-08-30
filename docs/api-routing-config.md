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
| `via_rung` | `0` | #568 per-search via legality rung. `0` = the configured via size (normal). `1` = judge via placement at the SMALLER fab-ladder rung, used only on a `dataclasses.replace()` clone for one escalation retry when a route is blocked purely by via legality. There is no CLI flag or GUI control: the engine sets it, and `KICAD_VIA_RUNG` (`2` = default rust per-rung map, `1` = python overlay, `0` = off) is a kill switch. |
| `grid_step` | `0.1` | Routing grid resolution |
| `layers` | `['F.Cu', 'B.Cu']` | Copper layers available to the router |

### Search algorithm

| Field | Default | Meaning |
|-------|---------|---------|
| `max_iterations` | `200000` | A* iteration cap per attempt |
| `max_probe_iterations` | `5000` | Quick probe per direction to detect blocked routes early |
| `heuristic_weight` | `2.3` | A* greediness (>1 = faster, possibly non-optimal paths); 1.9 -> 2.3 from the #586 corpus dose-response (peak at 2.3, both neighbors worse) |
| `via_cost` | `75` | Via penalty, in grid steps at the reference grid; 50 -> 75 from the #586 corpus (DRC -13, composes with heuristic 2.3) |
| `turn_cost` | `1000` | Penalty per direction change (straighter routes) |
| `direction_order` | `'forward'` | Try `'forward'` or `'backward'` direction first |

### Proximity penalties (soft costs)

Quick table; every field is specified in detail right below — **what feeds it,
its gates and exemptions, how sources compose, and who consumes it**. Costs are
"mm-equivalent detour per cell" at the reference grid: `0.2` means a cell at
the source center costs like 0.2&nbsp;mm of extra path (2&times; the plain move
cost of a 0.1&nbsp;mm cell); all falloffs are linear from full cost at the
source to zero at the radius.

| Field | Default | Meaning |
|-------|---------|---------|
| `stub_proximity_radius` | `2.0` | Radius around unrouted stubs / unescaped chip pads |
| `stub_proximity_cost` | `0.2` | Cost at the stub center (mm-equivalent detour) |
| `via_proximity_cost` | `10.0` | Multiplier on graded proximity when placing a via (0 = no extra cost) |
| `bga_proximity_radius` | `7.0` | Distance from BGA edges to penalize |
| `bga_proximity_cost` | `0.2` | Cost at the BGA edge (0 = fully off, radius disarmed too) |
| `package_proximity_zones` | `None` | Per-package proximity rects `(min_x, min_y, max_x, max_y, radius_mm)` for BGA/QFN/QFP, filled by the batch engines under the opt-in `KICAD_PACKAGE_PROXIMITY`. `None` = legacy (the hard BGA zones at the flat radius) |
| `track_proximity_distance` | `2.0` | Radius around routed tracks (same layer) |
| `track_proximity_cost` | `0.0` | Cost near routed tracks (**0 = off by default**) |
| `plan_probe` | `False` | Marks a config as the global plan's (#589) rough-route PROBE, whose relaxed legality lets probe terminals overlap future nets' copper. Set only on the plan's `replace()` clone — never on a config that emits copper |
| `ripped_route_avoidance_radius` | `1.0` | Radius around just-ripped routes |
| `ripped_route_avoidance_cost` | `0.1` | Cost near just-ripped routes (helps reroutes diverge) |

Two per-cell fields carry all of these: the **stub map** (one all-layer map)
and the **layer map** (one map per copper layer). Every A* move pays
`stub(cell) + layer(cell, current layer)` on top of the move cost; placing a
via pays `(stub + layer-at-destination) x via_proximity_cost` on top of the
via cost (both routers, single-ended and pose/diff-pair, same formula).

#### Stub map (all-layer): `stub_proximity_radius` / `stub_proximity_cost`

Sources, each with its own gate — this is the complete list:

1. **Stub endpoints** of *unrouted foreign* nets (`connectivity.get_stub_endpoints`).
   Gate: the net has >= 2 segments forming >= 2 disconnected groups — i.e.
   PARTIALLY routed copper with free ends (fanout stubs, interrupted routes).
   A bare unrouted net emits none; a fully-connected net emits none.
2. **Chip-pad pseudo-stubs** (`net_queries.get_chip_pad_positions`): pads whose
   future escape needs the surrounding space. Gate: the pad's footprint is a
   fine-pitch chip package (**BGA/QFN/QFP by `detect_package_type`, >= 4 pads**)
   AND the net has not yet escaped that pad (no same-net segment end or via
   within the pad's half-size + 0.05 mm). A pad retires as a source the moment
   fanout or routing attaches copper to it. (Historically ANY >= 4-pad
   footprint counted — 4-pad capacitors and multi-pin connectors emitted
   pseudo-stubs, and a connector's many distinct nets stacked into exactly the
   open-field noise the #584 sum experiment measured.)
3. **Ripped-route ghost vias** (see the ripped-route section below) at the
   `ripped_route_avoidance_*` falloff, per ripped net.
4. **Plane-flow via avoidance**: `route_planes` / the plane region connector
   stamp foreign vias into their own scratch maps with caller-chosen
   radius/cost (same mechanism, different knobs; not part of signal prepare).

Exclusions and exemptions:

- The **current net never repels itself** (its id is excluded from the source
  net list), and **routed nets are excluded** (their real copper is a hard
  obstacle instead).
- **Endpoint exemption** (soft-knobs C5): lookups within one track-width +
  clearance of the current net's own endpoints return 0, so the final approach
  to a pad sitting beside foreign stubs is not taxed. This exemption applies
  to the stub map ONLY — layer-map costs (BGA field, track ghosts) are not
  endpoint-exempt.
- `via_proximity_cost = 0` means vias pay **no extra** near stubs (the
  historical inverted meaning — 0 = hard via ban + ~200x CPU hazard — was
  removed in Rust 0.20.1).

Definition review flags (open questions, not bugs):

- The stamp is **all-layer**: a stub on B.Cu repels routing on F.Cu.
  `get_stub_endpoints` returns each stub's layer, but the stamp ignores it —
  layer-aware stub proximity is an untested candidate refinement.
- Stub endpoints of nets destined for **planes/pours** still emit; arguably a
  net that will be pour-connected needs no escape corridor.

#### BGA proximity field: `bga_proximity_radius` / `bga_proximity_cost`

Geometry: for every auto-detected **BGA** component (pad-bounding-box zones —
**QFN/QFP get NEITHER the hard zone NOR the proximity field**; the engine only
ever auto-detects BGA packages, so a QFN's sole protection is its pads as hard
obstacles plus chip-pad pseudo-stubs on unescaped pads), the zone interior
carries the full edge-tier cost on EVERY copper layer (so allowed-cells
windows punched through the hard block are not free-via holes), with the
linear ring falling to zero at `bga_proximity_radius` outside the zone edge.

Plumbing: the field rides the **layer map** under a reserved cache key
(soft-knobs B1), registered once per `batch_route` / `batch_route_diff_pairs`
call and re-merged on every per-net prepare — stamping it into the base map
instead would be wiped before every net (that was the pre-B1 silent no-op).

`bga_proximity_cost = 0` fully disarms the feature: the field is not emitted
AND the `is_in_bga_proximity` radius used by the proximity heuristic and the
pose router is not armed (historically the radius armed unconditionally, so
zeroing the cost silently left a 10x diff-pair via cliff active in the 7 mm
ring; both the cliff and the gap are gone as of Rust 0.20.1).

Definition review flags: the radius is a flat 7 mm regardless of package size;
QFN/QFP have no hard zone and no proximity ring at all — their escape collar
carries no graded protection (#585 item 4). Both flags still describe the
DEFAULT path; `package_proximity_zones` below is the opt-in answer to them.

#### Per-package proximity zones: `package_proximity_zones`

`(min_x, min_y, max_x, max_y, radius_mm)` rects giving **every** fine-pitch
package — BGA, QFN **and** QFP with >= 4 pads — its own proximity ring, with the
radius scaled by escape demand rather than flat:

```
r = bga_proximity_radius * sqrt(n_pads / 1000), clamped to [2.0, bga_proximity_radius]
```

so a 1000+-ball monster earns the full 7 mm, a 100-ball BGA ~2.2 mm, and any
QFN the 2 mm floor. Rects are pad-bounding-box anchored (margin 0), like the
hard zones.

This feeds the **soft field only** — the hard exclusion zones stay BGA-only, so
a QFN's interior remains routable, just priced. It also defines the
max-composition boundary in `zoned` sum mode
(`obstacle_costs.proximity_max_zone_rects`).

`None` (the default) means legacy behavior: the field is built from
`bga_exclusion_zones` at the flat `bga_proximity_radius`. `bga_proximity_cost`
and `bga_proximity_radius` still gate the whole field either way — this changes
*where* the rings go, not whether they exist.

Filled by `batch_route` / `batch_route_diff_pairs` from the board
(`routing_common.package_proximity_zones`) **only under the opt-in env knob
`KICAD_PACKAGE_PROXIMITY`**, so a default run leaves it `None`. It is opt-in
because it screened **negative** as a default (glasgow 7 -> 10 failures, `oc`
midchain 23 -> 26): the sqrt scaling *shrinks* the ring on exactly the
large BGAs that need it most, and tiny DFN/SOT packages classified as QFN add
clutter. A scale-up-only variant with a pad-count floor is the pending idea.

#### Track proximity: `track_proximity_distance` / `track_proximity_cost`

Per-layer corridors around copper **routed in THIS run**: on every routing
success the net's segments are sampled (~1 mm spacing, deduped within the net)
into a per-net array in the track-proximity cache; rip-up removes the entry.
Merged into the layer map on every prepare.

**Default OFF** (`cost 0.0`) — yet it is the strongest single knob measured
(2026-08-02 study: `--track-proximity-cost 2` took cynthion 97.9 -> 100%).
Chain-level results were mixed across board types, so it ships as retry
guidance rather than a default.

Definition review flags: **pre-existing input-board copper never emits** — 
nets registered as rippable-pre-existing skip the cache, so corridors of
already-routed boards exert no spreading pressure on new routes. No width
awareness (a 2 mm power trunk and a 0.1 mm signal emit the same field).

#### Via proximity multiplier: `via_proximity_cost`

Not a field — a multiplier consumed at via placement in BOTH routers:
`(stub + layer-at-destination) x via_proximity_cost` added to the via cost.
Because the layer map carries every contributor, this multiplies BGA fields,
track ghosts, ripped-route ghosts, congestion and plane-fragility costs alike
— a via in a fragile pour neck at default fragility pays up to 10 x 2.0
mm-equivalent. `0` = no extra cost (see stub map above); fractional values
round up to at least 1.

#### Ripped-route avoidance: `ripped_route_avoidance_radius` / `_cost`

When rip-up removes a net's copper, its former corridor keeps a per-net ghost:
segment corridors go to the **layer map** (per layer), via sites to the
**stub map** (all-layer), both at this falloff. Ghosts are dropped the moment
the ripped net has real copper again (soft-knobs C1 — a re-routed net's ghost
would otherwise repel everyone from a corridor that is already re-occupied or
legitimately free), and a net can never appear as both a live corridor and a
ghost (enforced with a warning at merge). The #540 outer-pass "casualty"
ghosts and the plane-repair corridor ghosts (soft mode) ride the same dicts
and falloff (plane-repair floors zero knobs to 0.1/1.0 rather than no-op).

#### Env-gated fields riding the same layer map

These compose with everything above and are documented here because they are
otherwise invisible in flag tables:

| Field | Env knob | Default | What it prices |
|-------|----------|---------|----------------|
| Plane fragility | `KICAD_PLANE_FRAGILITY_COST` | **2.0 = ON** | Cells near pour boundaries/necks, so signals do not bisect a plane. The LARGEST default field (10x the stub tier). `0` reverts. |
| Congestion v1 | `KICAD_CONGESTION_COST` | 0 = off | All-layer copper-density field. |
| Congestion v2 | `KICAD_CONGESTION2_COST` | 0 = off | Demand/capacity bins, owner-exempt (your own destination does not repel you). A source in the layer-map composition pass since d52f1c2 (#585 item 7): sums in sum mode, max-mode unchanged by commutativity. |
| History congestion | `KICAD_HISTORY_COST` | 0 = off | PathFinder-style negotiated congestion (#590): per-CELL, CUMULATIVE conflict history for the rest of the call. See below. |

##### History congestion (#590), the only field that measures TIME

Every other field prices the board's state *now*. This one prices how often a
cell has been **fought over**, so repeatedly-contested ground gets steadily
more expensive and the contest dissolves without anyone arbitrating whom to
rip (the 0802 rip study refuted every frame-local rip gate and concluded "soft
rra pricing beats refusal"; this is that conclusion made cumulative).

Conflict events, charged in mm-equivalent to the cells involved (v2
re-targeted the charging after the v1 one-board screen — whole-footprint rip
stamps were measured negative, raw-frontier charging mostly priced static
copper no rip can clear):

- a **contest** (primary, full increment) — the intersection of a failed
  search's blocked frontier with a routed net's keep-out: ground one net
  holds and another just stalled against, the engine's analog of
  PathFinder's *overused nodes*. Charged inside
  `analyze_frontier_blocking` (which already computes the per-blocker
  intersection for rip ranking), **before** any rip — so a ripped blocker's
  reroute already sees its contested ground priced and relocates instead of
  re-taking it. Repeat contests **escalate**: at `KICAD_HISTORY_ESCALATE`
  1.0 a re-charged cell *doubles* (0.1 → 0.2 → 0.4 …), because a routing
  call has only a handful of rip rounds and a flat increment cannot build a
  gradient in that many iterations. One failure analyzed twice in a row is
  charged once.
- a **rip's whole footprint** (v1) — `KICAD_HISTORY_RIP_WEIGHT` × the
  increment over the ripped copper (half width + `KICAD_HISTORY_RADIUS`,
  vias on every layer). Default **0**: kept only for A/B against v1.
- a **failed search's raw blocked frontier** (v1) —
  `KICAD_HISTORY_BLOCKED_WEIGHT` × the increment. Default **0**: the contest
  event charges the useful subset at full weight.

| Knob | Default | Meaning |
|------|---------|---------|
| `KICAD_HISTORY_COST` | `0` (off) | mm-equivalent per contest event |
| `KICAD_HISTORY_CAP` | `0` (uncapped) | Ceiling on a cell's accumulated history. Guards the *productive*-churn regime — a fine-pitch BGA escape field, where 15 rips converge, must not wall itself off. |
| `KICAD_HISTORY_ESCALATE` | `1.0` | Repeat-contest multiplier: re-charging a cell adds `max(inc, escalate × accumulated)` — 1.0 doubles per repeat; `0` = flat v1 accumulation |
| `KICAD_HISTORY_RIP_WEIGHT` | `0` (off) | v1 whole-footprint rip stamp weight |
| `KICAD_HISTORY_RADIUS` | `0.25` mm | Added to the copper half-width when a v1 rip stamps |
| `KICAD_HISTORY_BLOCKED_WEIGHT` | `0` (off) | v1 raw-frontier weight |
| `KICAD_HISTORY_MAX_CELLS` | `500000` | Growth guard bounding the per-prepare merge: past this, each event **evicts the lowest-weight cells** to make room (chronological refusal would unprice the endgame conflicts — the ones completion is measured on). Evictions are disclosed in the run summary along with the field's size and upkeep time. |

**Cost.** The event path is cheap (a rip stamps its corridor by vectorized
linspace sampling at half-radius spacing, and the sorted field takes new cells
by merge-insert, so an event is O(field), not O(field·log field) — ~14 ms at a
pathological 2 M-cell field, well under 1 ms at realistic sizes). What you
actually pay is the per-prepare composition, which now vstacks one more source:
measured against a 200 k-row baseline of existing sources, +2.5 ms/prepare at a
50 k-cell field and +8.7 ms at 200 k. The run summary prints the field's event
count, cell count, peak, and upkeep time so an A/B can see both sides.

Distinct from the ripped-route ghosts above in the two ways that matter: those
are per-NET and are **deleted** when the victim reroutes (C1); this is
per-CELL and survives for the whole call. Scope is one routing call (fresh at
batch start, like congestion v2's bins); cross-step persistence is a v2
question. No decay — decay is an FPGA-ism for hundred-iteration convergence.
No CLI flag or GUI control until the corpus says it earns one.

#### Composition: how multiple sources combine (`KICAD_PROXIMITY_SUM`)

Every source above is deduped WITHIN itself (per net per category: a net's six
connector stubs, or one track's overlapping sample disks, count once — cost
never depends on sampling density). Across sources, per cell:

- **unset (default): max.** Saturating — 30 overlapping stub fields cost the
  same as one. (Briefly `zoned` by default; reverted after the sets-11-15
  holdout showed it erasing gains on ordinary boards at real CPU cost —
  the composition modes stay opt-in / retry-tier.)
- **`1`/`sum`: sum.** Density gradient — corridors threading many nets' fields
  price proportionally (glasgow A/B: 11 -> 4 failures). Steeper fields around
  clustered pads (lpddr4 A/B: 6x search, one extra pad short).
- **`zoned`: sum, except max inside BGA escape fields** (zone +
  `bga_proximity_radius`), where stacked foreign fields price a net's
  MANDATORY approach rather than an avoidable crowd. On boards with no BGA
  it is exactly `sum`.
- **`softcap`: the global blend `max + α·(sum − max)` per cell, everywhere**
  (#584). `KICAD_PROXIMITY_SOFTCAP_ALPHA` sets α (default `0.3`): the
  max-mode floor plus a fraction of the crowd pressure, so density still
  repels but stacked fields on mandatory approaches can't explode. α=0
  reproduces max, α=1 reproduces sum (both exactly, up to integer-cost
  rounding). Implemented with the existing primitives — the stub map stamps
  the `(1−α)`-scaled max-insert batch plus the `α`-scaled grouped sum; the
  layer map blends both aggregates in one pass.

Double-merge safety is structural in all modes: the layer-map write primitive
is an idempotent max-insert over uniquely-composed rows (re-merging the same
composition rewrites identical values), and the stub map is stamped once per
cleared prepare cycle.

#### Search-side (not stamps)

`proximity_heuristic_factor` (default `0.0` since the heuristic-weight 2.3 flip; formerly `0.02`) adds an estimated proximity cost
per remaining step to the A* heuristic when a route's endpoints sit inside
stub/BGA zones — a deliberate slight overestimate that trades exactness for
search speed on proximity-heavy boards.

### Layer preferences and alignment

| Field | Default | Meaning |
|-------|---------|---------|
| `layer_costs` | `[]` | Per-layer cost multipliers (1.0 = neutral), parallel to `layers` |
| `direction_preference_cost` | `5` | Penalty for off-direction moves; layers alternate H/V starting horizontal on top (0 = off). A WEAK nudge is the optimum: #663's corpus screen (sets 1-5, 75 boards/arm, one commit) measured 5 against the former 250 default at -22 incomplete nets (-19.6%), W15/L6, real DRC flat. 250 priced every off-axis move above 3 vias (`VIA_COST` 75), forcing detours; 0 loses the layer organization entirely (single-board dose curve 0->19, 5->9, 250->15 issues). Higher values (e.g. 5000 = 5x a move) enforce strict human-style lanes but make dense boards' short diagonal hops unroutable (sets 6-11 A/B regression). NOTE the oracle-weld and plane sub-configs never receive this parameter and run at `GridRouteConfig`'s own 250 default |
| `vertical_attraction_radius` | `1.0` | Radius for cross-layer alignment bonus (0 = off) |
| `vertical_attraction_cost` | `0.0` | Bonus (negative cost) for vertically aligned positions (**0 = off by default**). NET-AGNOSTIC (soft-knobs C4): the per-cell layer bitmask carries no net id, so it pulls toward ANY other net's tracks on other layers; suppressed inside stub-proximity cells and BGA zones; capped so a step can never go free |

### BGA zones

| Field | Default | Meaning |
|-------|---------|---------|
| `bga_exclusion_zones` | `[]` | `(min_x, min_y, max_x, max_y[, edge_tolerance])` regions blocking vias; from `auto_detect_bga_exclusion_zones` |

### Impedance and per-net widths

| Field | Default | Meaning |
|-------|---------|---------|
| `impedance_target` | `None` | Target Z0 in Ω; when set, per-layer widths come from `layer_widths` |
| `layer_widths` | `{}` | Layer name → width (filled by `impedance.calculate_layer_widths_for_impedance`) |
| `coplanar_gap` | `0.0` | #486 **declaration**: design gap (mm) from a controlled-impedance trace's edge to the same-layer ground pour. `> 0` means outer-layer widths came from the coplanar-waveguide-over-ground model rather than microstrip. Set from `route.py --coplanar-gap` / `route_diff.py --coplanar-gap` |
| `coplanar_net_ids` | `set()` | #486: net ids the coplanar declaration applies to. **Empty with a non-zero `coplanar_gap` means the whole call is coplanar**, and `layer_widths` already holds the CPW widths. Non-empty means only these nets are, and their widths live in `coplanar_layer_widths` |
| `coplanar_layer_widths` | `{}` | #486: layer name → CPW-derived width, used only for `coplanar_net_ids`. Lets one call mix coplanar and microstrip nets |
| `reserve_layer_widths` | `False` | Obstacle-stamp reserve policy (#156): `False` = stamps reserve nominal `track_width`, wide/impedance nets ride per-net fractional `track_margin`; `True` (diff engine) = stamps bake the full per-layer width |
| `power_net_widths` | `{}` | net_id → width override for power nets (never below `track_width`) |
| `net_track_widths` | `{}` | net_id → the net's OWN netclass width (mm, #435), used EXACTLY (may be *narrower* than `track_width`); auto-read from the `.kicad_pro` only when `--track-width` is omitted AND no `--impedance` is set (impedance-solved widths would be overridden otherwise, #610), floored at the fab minimum by the caller. Lower priority than a manual `power_net_widths` override |
| `netclass_width_floors` | `{}` | net_id → netclass-declared width (mm) as an **escalation floor** only: loaded even when an explicit `--track-width` suppresses `net_track_widths`, so the rescue/terminal ladders may narrow a stuck net down to min(nominal, fab track floor, its netclass width) — designer-sanctioned geometry — without changing nominal routing. Clamped at the advanced-tier fab floor at load |
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
| `meander_spacing` | `2.0` | Centre-to-centre pitch of adjacent meander arms, in multiples of the net's routed track width (#501) |
| `net_layer_widths` | `{}` | Per-net per-layer widths reapplied from stored `.kicad_pro` impedance declarations on a redo (#521); outranks the netclass scalar |
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
| `hole_clearance` | `0.0` | Copper-to-HOLE floor (KiCad's `min_hole_clearance`) — keeps TRACKS off an NPTH wall. NOT the same rule as `hole_to_hole_clearance` (drill-to-drill). `0` = read the board's own constraint, falling back to `routing_defaults.NPTH_TO_TRACK_CLEARANCE` |
| `board_edge_clearance` | `0.0` | Clearance from board edge (0 = use `clearance`) |
| `same_net_pad_clearance` | `-1.0` | #581: edge-to-edge clearance between **every placed via** and **same-net SMD pads**. `> 0` forbids via-in-pad globally (routing/tap/rescue via placement blocks same-net SMD pads at this clearance; pad-centre swap vias are declined; the #189 in-pad rescue is disabled; the sub-grid via nudge honors it; BGA under-pad escapes run dog-bone, QFN refuses via-in-pad). `-1` **and** `0` preserve pre-#581 behavior exactly (0 keeps only its legacy meaning where `route_planes` passes it explicitly into its stitching via maps). Set from `--same-net-pad-clearance` (planes/route/route_diff/fanout/repair) or auto-read from the persisted `.kicad_pro` record (`kicad_routing_tools.same_net_pad_clearance`); an active flag value is persisted so later chain steps inherit it |
| `net_clearances` | `{}` | `{net_id: class_clearance_mm}` — per-net **net-class** clearance for KiCad's cross-class rule (see below) |
| `net_clearance_floor` | `None` | Routing-side floor (max class clearance among the nets being routed this call); set by `set_net_clearances()` |
| `layer_clearances` | `{}` | `{layer_name: mm}` — per-layer clearance from the board's `.kicad_dru` custom rules (#498). **Replacement** semantics, mirroring KiCad's precedence: on a ruled layer the value replaces the net/class-resolved pair clearance for every pair there (it may tighten *or* relax); unruled layers keep the normal resolution. Auto-read engine-side from the sibling `.kicad_dru` by **every routing step** — `batch_route`/`batch_route_diff_pairs`, plane create/repair, BGA/QFN fanout, oracle sub-routes (`kicad_dru.install_layer_clearances`, fab-floor pinned; path discovery via `PCBData.source_path` where the engine signature has no `input_file`) — there is deliberately **no CLI flag and no GUI control**; the rules file is the single source of truth and `check_drc`/staged kicad-cli grade from the same file. Resolve via `layer_clearance(layer, fallback)`; stack-spanning pairs (via barrels) use `stack_clearance(fallback)` = max over the rules and the fallback. Empty map = byte-identical to no rules |
| `track_clearances` | `{}` | `{net_id: mm}` — the track-to-track channel (#735): the effective per-obstacle map computed from the board's `.kicad_dru` TRACK-scoped rules (`A.NetClass=='X' && B.NetClass!='X' && A.Type=='track' && B.Type=='track'`) over THIS call's routed set. Applied by `track_obstacle_clearance(net_id, resolved)` — **raise-only** over the fully-resolved pair value, seg-vs-seg obstacle expansion only (pads/vias exempt), per-layer like every segment stamp. Auto-read engine-side (`kicad_dru.install_track_clearances`, same no-flag convention as `layer_clearances`); an explicit dict (tests/GUI) wins and stops the auto-read. Empty map short-circuits to byte-identical behavior. The PLACEMENT side reads the same rules pair-EXACTLY rather than through this map (`kicad_dru.track_pair_clearance`, the resolver `check_drc` and `placement/fanout_clearance`'s connector gate both call since #735); `kicad_dru.board_track_rules(pcb_data)` is the quiet reader for an engine that has no `GridRouteConfig` |

#### Cross-class clearance (KiCad `max(classA, classB)`)

KiCad's required spacing between two nets of **different** net classes is
`max(classA, classB)`. The router honors this: every foreign obstacle — pre-placed
copper **and** copper routed earlier in the same call (in-run) — is priced at
`max(routing-side floor, that obstacle net's own class clearance)`. `config.clearance`
remains the Default/routing-side clearance; `net_clearances` carries the non-Default
classes. An **empty** map reproduces plain `config.clearance` behaviour exactly
(byte-identical — the feature is inert until a map is supplied).

```python
from routing_config import GridRouteConfig
config = GridRouteConfig(track_width=0.15, clearance=0.15)
power_hi_net_id, sig_a, sig_b, some_default_net = 1, 2, 3, 4
config.set_net_clearances({power_hi_net_id: 0.25}, routed_net_ids=[sig_a, sig_b])
assert config.obstacle_clearance(power_hi_net_id) == 0.25   # max(floor 0.15, class 0.25)
assert config.obstacle_clearance(some_default_net) == 0.15  # not in the map -> config.clearance
```

- `set_net_clearances(net_clearances, routed_net_ids)` installs the map and computes
  `net_clearance_floor` (over the routed nets only, so a foreign class cannot inflate
  the floor and over-block every routed net). `batch_route` / `batch_route_diff_pairs`
  call it once per run.
- `obstacle_clearance(net_id)` is the single accessor the base-map builder **and** every
  incremental obstacle stamper read, so ADD and REMOVE derive an identical per-obstacle
  clearance (ref-count symmetry).
- The map is **auto-read** from the sibling `.kicad_pro` netclasses by `route.py` /
  `route_diff.py` / the fanout and plane CLIs (and inside `batch_route` for any other
  caller). Only non-Default classes appear, so an all-Default board yields `{}`.
  Supply `--net-clearances <json>` (net name → mm) to override. The GUI derives the
  same map from the live board.
- **`--clearance` is a ceiling on the map (#439).** When `--clearance` is GIVEN, each
  auto-read class is capped at it (`net_clearances[nid] = min(class, clearance)`) before
  it is installed — a class *tighter* than `--clearance` survives; a *looser* one is
  capped, because stock net classes are largely aspirational (real boards, and even the
  human-routed references, route below them). The output `.kicad_pro` writeback then
  clamps each non-Default class to that same routed floor, so KiCad grades exactly what
  was routed. When `--clearance` is OMITTED there is no ceiling: each net routes at its
  own class and the writeback preserves the classes (base = the board's Default class).
  An explicit `--net-clearances` map is used as given (not capped). In the GUI, checking
  the **Min Clearance** override box is the "`--clearance` given" signal.

### Strategies and recovery

| Field | Default | Meaning |
|-------|---------|---------|
| `max_rip_up_count` | `3` | Max blocking routes ripped up at once (progressive 1..N) |
| `ripup_abandon_metric` | `'stranded'` | Keep-retry vs abandon rule for multipoint tap rip-ups (see [rip-up-reroute.md](rip-up-reroute.md#abandon-metrics)) |
| `ripup_blocker_select` | `'count'` | Blocker-ordering algorithm for the rip-up ladder: `'count'`, `'near-target'`, `'bidir'`, `'mincut'` (see [rip-up-reroute.md](rip-up-reroute.md#blocker-selection-algorithms)) |
| `bus_rip_resistance` | `1.0` | >1 divides bus-group members' blocker scores so the ladder prefers ripping bystanders over a settled bus river; mincut prices member cells higher by the same factor. Env: `KICAD_BUS_RIP_RESISTANCE` |
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
| `proximity_heuristic_factor` | `0.0` | Tightens the A* heuristic for proximity costs (off: hw 2.3 already covers it; s2 rescan quality-neutral, -8% CPU) |

### Methods

```python
config.get_track_width(layer) -> float
```
Layer-aware width: `layer_widths[layer]` if impedance-controlled, else
`track_width`.

```python
config.get_net_track_width(net_id, layer) -> float
```
Net- and layer-aware width. Priority: `power_net_widths[net_id]` (floored up to
`track_width`) → `net_track_widths[net_id]` (the net's own class width, used
exactly, #435) → `coplanar_layer_widths[layer]` when `net_id` is in
`coplanar_net_ids` (#486) → `layer_widths[layer]` → `track_width`. This is what
obstacle expansion uses.

**The coplanar rung is a declaration, not a measurement.** At route time the
pour does not exist yet, so `coplanar_gap` states what the plane step is going
to do. Pour with a matching `route_planes --zone-clearance`, then verify the
geometry actually came out that way with
`check_impedance.py --coplanar-gap <same value>`. Nothing in the router enforces
the gap — a coplanar-declared net whose pour never arrives is simply routed at a
width that assumes a ground it does not have.

```python
config.get_max_track_width() -> float
```
Maximum width across layers (for worst-case via clearance).

```python
config.route_reserve_width(layer) -> float
```
Routing-side width the obstacle stamps reserve for the *future* routed track
(#156). With `reserve_layer_widths=False` (single-ended engine, the default)
this is the nominal `track_width` (floored to a narrower impedance layer
width), and any net routing wider — power override or impedance layer width —
covers its extra half-width through its own fractional `track_margin`. With
`reserve_layer_widths=True` (the diff-pair engine) it is the full
`get_track_width(layer)`, baked mm-exact into the maps. Track margins are
always computed against this value, so stamps and margins cannot drift.

```python
config.track_margins_for_net(net_id) -> List[float]   # grid cells, per layer
config.track_margins_for_width(width) -> List[float]  # for a uniform width
config.base_track_margins() -> List[float]            # each layer's own base width
```
Per-layer **fractional** A* track margins (#156): the exact extra half-width
over `route_reserve_width(layer)`, in grid cells — no ceil, no `+1` (the Rust
swept-capsule `segment_blocked` covers diagonals precisely). Passed to
`route_multi` / `route_with_frontier`, which accept a float or a per-layer
`list[float]`.

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

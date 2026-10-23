# Bus Routing

Two tools route buses:

- **`route.py --bus`** (this page's main subject): the A* router's bus mode. It detects groups of nets that travel together, routes each group first and middle-out, and attracts every member to one planned corridor (or to its already-routed neighbour), so the group comes out as parallel traces in a common river.
- **The topo bus step, `awx/route_bus.py`** (experimental): plans a whole bus between two ball arrays (a DDR, an LVDS bus) before laying any copper. See [Buses between two arrays](#buses-between-two-arrays-the-topo-bus-step-experimental) at the end.

Implementation of `--bus`: `bus_detection.py` (detection, filter, ordering, neighbour selection), `bus_corridor.py` (corridor planning), `single_ended_loop.py` (where they run), `rust_router/src/router.rs` (the attraction discount).

## Overview

Without bus routing, each net in a data bus is routed independently and the results can fan out along unrelated paths. With `--bus`, nets whose endpoints cluster together are detected as a group, a corridor with room for the whole group is planned, the group routes middle-out ahead of every other net, and each member is *attracted* toward the corridor, producing parallel traces that follow it.

```bash
python3 py_router/route.py board.kicad_pcb --nets "DATA*" --bus
```

The GUI's Advanced options tab has the same switch ("Enable bus routing", with the four parameters below) and the `bus` ordering choice. `route_diff.py` accepts the `--bus` flags, but pairs are never bus-routed: the parameters are ignored there.

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--bus` | off | Enable bus detection, corridor planning and attraction |
| `--bus-detection-radius` | 5.0 | Max endpoint distance for nets to form a bus (mm) |
| `--bus-min-nets` | 2 | Minimum number of nets for a raw clique (the strict filter below then needs 3) |
| `--bus-attraction-radius` | 5.0 | Reach of the attraction around the corridor or neighbour path (mm) |
| `--bus-attraction-bonus` | 5000 | Attraction strength: `bonus/50` percent off a step's cost, capped at 90% (the default is at the cap) |
| `--ordering bus` | - | The bus-first ordering alone, with no corridor and no attraction (see [Net Ordering](net-ordering.md)) |

## Bus Detection

`detect_bus_groups()` looks for *cliques* of nets whose endpoints cluster: a group qualifies if either all of its **source** endpoints or all of its **target** endpoints lie pairwise within `--bus-detection-radius` of each other. Endpoints are the centroids of the net's two main stub groups where stubs exist, otherwise pad positions.

The clique search (`_find_largest_clique()`) is greedy: candidate pairs are sorted closest-first, and each seed pair is grown by adding nets within radius of *every* current member. The largest clique (≥ `--bus-min-nets`) becomes a bus; its nets are removed and detection repeats until no further group can be formed.

**Both-end coherence:** a clique found at one end is then sub-clustered by where its members' *other* endpoints go (greedy centroid clustering at 2× the detection radius). A 20-net grab-bag at a BGA corner splits into the real rivers (the SD-card group, the SDIO group, the PCM group), because members of a true bus travel together at both ends. Sub-groups below `--bus-min-nets` fall back to individual routing.

**Strict geometric filter** (`filter_bus_groups_geometric()`, on by default): a group is kept as a bus only if its members genuinely travel together. Each member's run is the farthest-apart pair of its pads; the group keeps its largest subset whose runs are at least 5 mm long, parallel within a cos > 0.9 cone (either way round, a net has no direction) and within a 1.5× length ratio of each other, and drops power nets (`--power-nets`). A group left with fewer than 3 members is not a bus. This retires decoupling-cap clusters and power rails that raw cliques swept up. The run prints `Bus filter: N raw clique group(s) -> M geometric bus(es)`.

When the clique formed at the targets, members route from that clustered end, in one direction only, where the attraction can guide them.

## Routing Order: Bus Groups First, Middle-Out

Bus groups are moved to the **front** of the routing order, ahead of every non-bus net; the remaining nets follow in the order the `--ordering` strategy gave them. Within a bus, nets are ordered by physical position along the bus's dominant axis (left-to-right if the endpoints spread more in X, else top-to-bottom), and routing starts from the middle net and alternates outward (`get_bus_routing_order()`):

```
Physical order:   A  B  C  D  E
Routing order:    C, B, D, A, E
```

The middle net (the "guide") is also the group's corridor representative.

## Corridor Planning

Before any member routes, `plan_bus_corridors()` probe-routes each group's representative at a ladder of inflated widths: rung *k* routes it as a track `2k × (track + clearance)` wider, which keeps room for *k* sibling tracks on each side (the wide-power-net `track_margin` mechanism, neck-down disabled, on one probe map that leaves out every bus member's own copper, since the siblings will pack into the corridor). Rungs run from `k_max = min(3, max(1, N // 2))` down to 0, and each rung that routes is scored

```
score = probe length + (k_max − k) × (track + clearance) × 40
```

so a corridor with room for the whole group wins unless it costs much more length. The winning centreline becomes the attraction path for **every** member, the guide included. Probes commit nothing.

- **Laminar probe.** The probe pays `KICAD_BUS_CORRIDOR_PROBE_VIA_MULT` (default 20) times the via cost, so the corridor stays on one layer unless a change is unavoidable. Members route at the normal via cost.
- **Demotion.** A centreline that still needs more than `KICAD_BUS_MAX_CORRIDOR_LAYER_CHANGES` (default 1) layer changes away from its ends (changes within 3 mm of either end are the river's on-ramps and do not count) has no lane to offer: the group is **demoted**, with no corridor, no attraction and no bus ordering, and its members route as plain nets.
- **No corridor.** A group none of whose rungs route keeps its bus ordering and falls back to neighbour attraction.

## Multi-Point Bus Members

A bus member with 3+ terminals (e.g. a strobe line with pull-up resistors) routes in the multipoint phases: one **main edge** now, the remaining taps in Phase 3, *after every other net's main route*. Two bus-specific behaviours keep such members on the corridor:

- **Corridor-spanning main edge.** The MST normally realizes each connection by the *closest* terminal pair, which for a BGA-to-chip bus net is typically resistor-to-trunk, leaving the dense ball tap deferred until the surrounding balls' own escapes seal it in. When the member has an attraction path, the main edge is instead re-realized as the terminal pair that spans the corridor's two endpoints (e.g. the BGA ball and the far chip pin), and only off-corridor taps (the pull-ups) defer to Phase 3. Disable with `KICAD_BUS_MULTIPOINT_SPAN=0`.
- **Corridor attraction on the main edge.** The multipoint main routes with the same attraction path (and cross-layer share) as point-to-point members.

## Attraction

A member's attraction path is its group's planned corridor, else (no corridor) the path of an already-routed adjacent net in the bus's physical order, the left neighbour first (`get_attraction_neighbor()`). The path is sampled at every grid step and each point carries the direction to the next (`set_attraction_path()`).

During A*, `get_path_attraction_bonus()` takes a **percentage off the step's cost** for a move near the path:

```
discount % = min(bonus / 50, 90) × proximity² × alignment × layer share
```

- **Proximity**: `1 − distance / --bus-attraction-radius` to the nearest path point (squared in the formula): full at the path, zero at the radius.
- **Alignment** (the move against the path direction at that point): 100% for a diagonal move along a diagonal stretch, 70% for any other move with a forward component (straight along a straight stretch, or 45° off), **zero** for perpendicular or opposing moves. This rewards running alongside the path without rewarding circling it.
- **Layer share**: 100% when a path point on the move's own layer is within reach; otherwise, measured to the nearest point on another layer, `KICAD_BUS_XLAYER_PCT` (default 35) percent, so the corridor still guides a member while it travels a different layer.

A discount, unlike a subtracted bonus, can never make a step free, so the gradations survive. Clearances are enforced as normal: attraction only changes which legal path is cheapest.

## Interaction with Other Features

- **Net ordering**: `--bus` reorders whatever `--ordering` produced (bus groups first). `--ordering bus` gives the same reorder over an MPS base without corridors or attraction.
- **Failures**: a failed member gets the normal rip-up and retry. Ripped members re-route *with* their corridor (or routed-neighbour) attraction. Other members continue; a failed neighbour simply provides no attraction (the next routed neighbour is used, if any).
- **Differential pairs**: not bus-routed (see Overview).

## Tuning

- Increase `--bus-detection-radius` if a connector's pins span more than 5 mm and aren't being grouped; decrease it if unrelated nets are being pulled into a bus.
- `--bus-attraction-bonus` above 4500 changes nothing (the 90% cap); lower it if members hug the corridor at the cost of unnecessary detours. `--bus-attraction-radius` sets how far from the corridor the pull reaches.
- The run prints the detected groups and their physical order; `--verbose` adds the routing order, the guide, which neighbour each member follows, and every corridor rung.
- Env knobs on the default behaviour: `KICAD_BUS_STRICT=0` turns the strict filter off (raw cliques), `KICAD_BUS_XLAYER_PCT` sets the cross-layer share (0 = same layer only), `KICAD_BUS_CORRIDOR_PROBE_VIA_MULT` and `KICAD_BUS_MAX_CORRIDOR_LAYER_CHANGES` set the laminar probe and the demotion rule, `KICAD_BUS_MULTIPOINT_SPAN=0` turns the corridor-spanning main edge off.

## Limitations

- Grouping is geometric: two unrelated buses that leave a connector side by side and run parallel can be grouped together.
- Attraction is strongest on the path's own layer; other layers get the 35% share.
- A bus member with no corridor and no routed neighbour (e.g. all earlier members failed) routes without attraction.

## Buses between two arrays: the topo bus step (experimental)

`awx/route_bus.py` is the topological bus router of the `awx` research tree. Rather than routing nets one at a time, it plans a bus between two parts whole (the order of every lane, which layer each runs on, where each changes layer, and the fanout at both arrays) and then lays it. It is experimental. The [plan-pcb-routing](claude-skills.md#plan-pcb-routing) skill runs it as Step 1a, on the poured board and before any other fanout, for every bus between two ball arrays (or a ball array and a QFN/QFP), and its Step 1a is the guide to choosing modes and checking the result.

```bash
# Name the buses on the board (read-only), taken or not and why
python3 -X utf8 awx/route_bus.py board.kicad_pcb --find

# Route them in one call, one --bus each, largest first
python3 -X utf8 awx/route_bus.py board.kicad_pcb board_bus.kicad_pcb \
    --rounds 2 --inproc \
    --bus U1:U2:joint:F.Cu,B.Cu,In1.Cu,In2.Cu \
    --clearance-ceiling 0.1 --track-width 0.1 --fanout-track-width 0.0975 \
    --via-size 0.35 --via-drill 0.15
```

- **`--bus SRC:DEST[:MODE[:LAYERS]]`**, repeated, routes the buses in the order given. `MODE` is `joint` (a bus between two arrays: each round also plans the arrays' other nets and plane drops round the bus, so later fanouts leave those arrays alone), `escape` (every ball of both arrays dog-boned; for a second bus on an array an earlier bus already fanned out) or `plain` (a bus to a row part such as a QFN). `DEST` may name several parts, `U1:U4+U3:plain`, for one source with several ends. `LAYERS` are the routing layers, comma-separated; omitted, the step routes on F.Cu and B.Cu only, so name every copper layer, the planes' included. Without `--bus` (or `--src`/`--dest`), every bus found between two ball arrays is routed, largest first.
- **Sizes** are the routing CLIs' flags (`--clearance`, `--clearance-ceiling`, `--track-width`, `--via-size`, `--via-drill`, `--hole-to-hole-clearance`, `--board-edge-clearance`), an omitted one resolved as `route.py` resolves it; `--fanout-track-width` sets the escapes' track under the arrays. In a chain, pass `--clearance-ceiling` as everywhere else.
- **What it hands on.** The output is graded before it is handed on: the bus nets connected, no DRC violation on a bus net, and the board no worse than it came. When it passes, the output is the routed board, with its routed nets recorded as protected (#521) so `route.py`'s rips and the plane finalize leave them alone, and any net it left open is left to `route.py`. Otherwise the output is the board as it came and `route.py` routes the whole bus. The summary (`--json-out`, default `<OUT stem>.bus/summary.json`) names the nets routed and refused, and why. Exit code 0 means the bus was handed on routed and clean, 1 that the output is the input board, 2 a usage error.
- Every ball of a bus array must leave the step served (escaped, dropped to its plane or strapped): `route.py` cannot reach a bare ball in a ball field, and a fanout run after the bus step finds the field already full. The skill's Step 1a says how to count them.
- The step has no GUI action, and `tests/stress/manifest_to_plan.py` skips it, so a chain with a bus step replays from its `.sh` manifest only. `python3 awx/route_bus.py --help` lists every option.

# Bus Routing

This document describes bus routing: automatic detection of net groups with clustered endpoints (buses) and routing them as parallel tracks, each net following its already-routed neighbor.

Implementation: `bus_detection.py` (detection and ordering), `single_ended_loop.py` (neighbor selection), `rust_router/src/router.rs` (attraction cost).

## Overview

Without bus routing, each net in a data bus is routed independently and the results can fan out along unrelated paths. With `--bus`, nets whose endpoints cluster together are detected as a group, routed middle-out, and each net is *attracted* toward its neighbor's path — producing clean parallel traces that follow a common corridor.

```bash
python route.py board.kicad_pcb --nets "DATA*" --bus
```

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--bus` | false | Enable bus detection and routing |
| `--bus-detection-radius` | 5.0 | Max endpoint distance for nets to form a bus (mm) |
| `--bus-min-nets` | 2 | Minimum number of nets to form a bus group |
| `--bus-attraction-radius` | 5.0 | Attraction radius around the neighbor's track (mm) |
| `--bus-attraction-bonus` | 5000 | Cost bonus for moving parallel to the neighbor's track |

## Bus Detection

`detect_bus_groups()` looks for *cliques* of nets whose endpoints cluster: a group qualifies if either all of its **source** endpoints or all of its **target** endpoints lie pairwise within `--bus-detection-radius` of each other. Endpoints are stub free-ends where stubs exist, otherwise pad positions.

The clique search (`_find_largest_clique()`) is greedy: candidate pairs are sorted closest-first, and each seed pair is grown by adding nets within radius of *every* current member. The largest clique (≥ `--bus-min-nets`) becomes a bus; its nets are removed and detection repeats until no further group can be formed.

**Both-end coherence:** a clique found at one end is then sub-clustered by where its members' *other* endpoints go (greedy centroid clustering at 2× the detection radius). A 20-net grab-bag at a BGA corner splits into the real rivers — the SD-card group, the SDIO group, the PCM group — because members of a true bus travel together at both ends. Sub-groups below `--bus-min-nets` fall back to individual routing.

## Routing Order: Middle-Out

Within a bus, nets are ordered by physical position along the bus's dominant axis (left-to-right if the endpoints spread more in X, else top-to-bottom). Routing starts from the middle net and alternates outward (`get_bus_routing_order()`):

```
Physical order:   A  B  C  D  E
Routing order:    C, B, D, A, E
```

The middle net routes freely and defines the corridor; each subsequent net hugs the previously routed neighbor on its side. Bus members are reordered within the overall net order produced by the ordering strategy (MPS etc.) — non-bus nets are unaffected.

## Corridor Planning

Before any member routes, each group's representative is probe-routed at a ladder of inflated clearances (the wide-power `track_margin` mechanism — per-net inflation on the shared map, no extra map builds). Each rung that succeeds is scored by `length + ROOM_WEIGHT x missing sibling-rooms`: a centerline that fits the representative at (n-1) track pitches of extra clearance has room for the whole group beside it. The winning centerline becomes the attraction path for **every** member, the guide included — so the corridor is chosen with room for the group instead of being whatever the guide's solo path happened to be. Selection is soft: groups with no routable rung keep plain neighbor attraction. Vias inside a planned corridor cost extra (`KICAD_BUS_CORRIDOR_VIA_MULT`, default 4.0) so members stay in the river instead of hopping layers out of it.

## Multi-Point Bus Members

A bus member with 3+ terminals (e.g. a strobe line with pull-up resistors) routes in the multipoint phases: one **main edge** now, the remaining taps in Phase 3 — *after every other net's main route*. Two bus-specific behaviors keep such members on the corridor:

- **Corridor-spanning main edge.** The MST normally realizes each connection by the *closest* terminal pair, which for a BGA-to-chip bus net is typically resistor-to-trunk — leaving the dense ball tap deferred until the surrounding balls' own escapes seal it in. When the member has an attraction path, the main edge is instead re-realized as the terminal pair that spans the corridor's two endpoints (e.g. the BGA ball ↔ the far chip pin), and only off-corridor taps (the pull-ups) defer to Phase 3. Disable with `KICAD_BUS_MULTIPOINT_SPAN=0`.
- **Corridor attraction on the main edge.** The multipoint main routes with the same attraction path (and cross-layer discount) as point-to-point members; it previously routed blind.

Relatedly, `KICAD_MULTIPOINT_DENSE_FIRST=1` (experimental, off by default, not bus-specific) applies the same seal-risk reasoning to *any* multipoint net: MST edges landing on a fanned-out high-density package (BGA/QFN/QFP, 16+ pads) are routed ahead of longer edges elsewhere.

## Neighbor Attraction

When a bus member routes, `get_attraction_neighbor()` finds an already-routed adjacent net in the bus's physical order (preferring the left neighbor) and passes its path to the Rust router. The path is densely sampled (every grid step) and loaded with per-point direction vectors (`set_attraction_path()`).

During A*, `get_path_attraction_bonus()` reduces the cost of moves near the neighbor's path, scaled by two factors:

- **Direction alignment** (dot product of the candidate move with the path direction at the nearest path point): 100% bonus for parallel moves, 70% for moves 45° off-axis, **zero** for perpendicular or opposing moves. This rewards running alongside the neighbor without rewarding circling it.
- **Proximity** (quadratic falloff): full strength at the path, fading to zero at `--bus-attraction-radius`.

The final bonus is `attraction_bonus × proximity² × alignment`, subtracted from the move cost. Attraction applies only on the neighbor's layer.

## Interaction with Other Features

- **Net ordering**: bus detection runs after MPS/inside-out ordering; bus members are then regrouped middle-out within the sequence.
- **Failures**: if a bus member fails, normal rip-up and retry applies. Other bus members continue independently — a failed neighbor simply provides no attraction (the next routed neighbor is used instead, if any).
- **Differential pairs**: `route_diff.py` accepts the `--bus` flags, but bus routing is not implemented for differential pairs — the parameters are ignored there.
- **Obstacle handling**: attraction is a soft bonus; clearances are still enforced normally, so parallel nets pack at legal spacing.

## Tuning

- Increase `--bus-detection-radius` if a connector's pins span more than 5mm and aren't being grouped; decrease it if unrelated nets are being pulled into a bus.
- Increase `--bus-attraction-bonus` (up to ~10000) if bus nets keep taking individual shortcuts instead of following the corridor; decrease it if they hug the corridor at the cost of unnecessary detours.
- `--verbose` prints the detected groups and the middle-out routing order.
- Experimental env knobs: `KICAD_BUS_XLAYER_PCT` (default 35 with `--bus`) sets how much of the attraction discount applies on layers other than the neighbor's — the corridor guides a member even while it travels a different layer; `KICAD_BUS_CORRIDOR_VIA_MULT` (default 4.0) scales via cost inside a planned corridor. `KICAD_BUS_RIP_RESISTANCE` (default 1.0 = off) makes the rip-up ladder reluctant to rip bus members: their blocker scores are divided by this factor, so later nets prefer ripping bystanders over tearing up a settled river (validator-proved blockers stay exempt). `KICAD_BUS_MULTIPOINT_SPAN` (default on; `0` disables) controls the corridor-spanning main edge for multi-point members, and `KICAD_MULTIPOINT_DENSE_FIRST=1` (default off) prefers dense-package MST edges for any multipoint net — see "Multi-Point Bus Members". Promotion to real flags is tracked in #465.
- Ripped bus members re-route *with* their corridor (or routed-neighbor) attraction: a rip no longer degrades the river by rerouting the member blind.

## Limitations

- Grouping is purely geometric: nets that happen to cluster (e.g. two unrelated buses on the same connector edge) can be grouped together.
- Attraction is strongest on the neighbor's layer; other layers receive `KICAD_BUS_XLAYER_PCT` (35%) of the discount, so cross-layer guidance exists but is weaker.
- A bus member with no routed neighbor (e.g. all earlier members failed) routes without attraction.

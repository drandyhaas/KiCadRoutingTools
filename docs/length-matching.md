# Length Matching and Time Matching

This document describes how the router matches route lengths (or propagation times) within groups of nets, e.g. DDR4 byte lanes. Length matching adds trombone-style meanders to shorter routes until every net in a group is within tolerance of the longest one.

Implementation: `length_matching.py`, with propagation-delay calculations in `impedance.py`.

## Overview

Length matching runs after the nets in a group have been routed:

1. Measure each route's length (or propagation time), including via barrels.
2. Find the longest route in the group — it becomes the target.
3. For each shorter route, add meanders to a straight run of the route until its length is within tolerance of the target.

For multi-point nets this happens between Phase 1 and Phase 3 of the MST-based approach: the longest MST edge is routed first, meanders are applied to that clean 2-point path, and the remaining taps are routed afterwards (see [Routing Architecture](routing-architecture.md)).

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--length-match-group <patterns...>` | none | Define a match group by net name patterns (repeatable, one group per flag). The special value `auto` enables DDR4 auto-grouping |
| `--length-match-tolerance` | 0.1 | Acceptable length variance within a group (mm) |
| `--meander-amplitude` | 1.0 | Maximum meander bump height perpendicular to the trace (mm) |
| `--meander-spacing` | 2.0 | Centre-to-centre spacing of adjacent meander arms, in multiples of the net's routed track width (2.0 = 2W pitch = 1W edge gap) |
| `--time-matching` | false | Match propagation delay instead of physical length |
| `--time-match-tolerance` | 1.0 | Acceptable delay variance within a group (ps) |
| `--diff-pair-intra-match` | false | (`route_diff.py`) Match P and N lengths *within* each differential pair |
| `--ac-couple-match` | false | (`route_diff.py`) End-to-end length-match AC-coupled pairs split by series DC-blocking caps (#196): match the concatenated P path vs N path across the caps |

Examples:

```bash
# Manual group: all DQ0-7 and DQS0 nets matched together
python py_router/route.py board.kicad_pcb --length-match-group "*DQ[0-7]" "*DQS0*"

# Two separate groups
python py_router/route.py board.kicad_pcb \
    --length-match-group "*DQ[0-7]" "*DQS0*" \
    --length-match-group "*DQ1[0-5]" "*DQ[8-9]" "*DQS1*"

# Auto-detect DDR4 byte lanes
python py_router/route.py board.kicad_pcb --length-match-group auto

# Match propagation time instead of length
python py_router/route.py board.kicad_pcb --length-match-group auto \
    --time-matching --time-match-tolerance 1.0
```

## Group Specification

Each `--length-match-group` flag defines one group; nets matching *any* pattern in the flag belong to that group. Patterns support `*` wildcards and `[0-9]` character ranges (`match_net_pattern()`).

### DDR4 Auto-Grouping

`--length-match-group auto` calls `auto_group_ddr4_nets()`, which groups by name:

- **DQ nets** (`DQ<n>`) are grouped by byte lane: `n // 8` (DQ0–7 → lane 0, DQ8–15 → lane 1, …)
- **DQS strobes** (`DQS0`, `DQS0_N`, `DQSP0`, `DQSN0`, …) join their byte lane's group
- **Command/address nets** (`CA*`, `CMD`, `ADDR`, `A<n>`, `BA<n>`, `BG<n>`, `CK`, `CS`, `ODT`, `CKE`, `RAS`, `CAS`, `WE`) form a single command/address group

## The Trombone Meander

Meanders are rectangular bumps perpendicular to the trace, with 45° chamfered corners, alternating up/down along the run. The chamfer is half the arm pitch: adjacent risers sit `--meander-spacing` × the net's routed track width apart centre-to-centre (#501), so wider (impedance/power) nets automatically spread their arms; the chamfer never drops below 0.1mm, which reproduces the historical geometry exactly at the default 0.1mm track width. Same-net copper is exempt from every clearance check by design, so this pitch arithmetic — not DRC — is what keeps the arms from forming a tightly coupled comb that would add less *delay* than *length*:

```
Original:  ────────────────────────>

Meander:   ──╮╭──╮╭──╮╭──>
             ││  ││  ││
             ╰╯  ╰╯  ╰╯
```

`apply_meanders_to_route()` finds all straight runs in the route (minimum length 2× amplitude) and tries them longest-first. If the best run cannot absorb the whole target — common at wider pitches, which fit fewer bumps per run — the remainder **spills across further straight runs**: after each run is meandered, the straight runs are re-derived from the modified segment list and the next-best run takes the rest, until the target is met, progress stalls, or a round cap is hit. Within a run, `generate_trombone_meander()` distributes bumps with alternating direction. Each bump of amplitude *A* adds roughly `2A − 2.34 × chamfer` of extra length (≈1.77mm at the default 1.0mm amplitude and 0.1mm chamfer).

### Per-Bump Clearance Checking

Before placing each bump, `get_safe_amplitude_at_point()` checks the bump area against nearby segments, vias, and pads using a spatial index (`ClearanceIndex`, 2mm cells):

- Clearance margin: `track_width + clearance` plus a corner allowance for the 45° chamfers and half a grid step for output merging.
- If the full amplitude doesn't fit, the amplitude is reduced in 0.7× steps (binary-search style) down to a 0.2mm minimum.
- If neither the up nor the down direction fits at all, the bump position is skipped forward 0.2mm and tried again.

If the resulting meander under-shoots the target, the algorithm iterates: more bumps are added, or the amplitude is rescaled, until the route is within tolerance or the run is exhausted (`_apply_meanders_to_net_with_iteration()`).

## Via Barrel Length

Route lengths include the vertical distance through vias so that results match KiCad's length measurements:

- `PCBData.get_via_barrel_length(layer1, layer2)` sums the board stackup's layer thicknesses between the two copper layers.
- **Stub via barrels**: when a BGA fanout stub switches layers, the barrel length from the *stub layer to the pad layer* is included (`calculate_stub_via_barrel_length()` in `connectivity.py`), not the via's full span.

Without stackup information in the board file, default thicknesses are assumed.

## Multi-Layer Routes

Routes containing vias are supported, with restrictions:

- Meanders are only placed on straight runs that stay on a single layer.
- Via positions are preserved; for differential pairs the P/N tracks are regenerated from the meandered centerline and GND return vias are re-created afterwards.

## Differential Pairs

Two forms of matching exist for differential pairs (`route_diff.py`):

- **Group matching** — multiple pairs in a `--length-match-group` are matched pair-to-pair by meandering each pair's *centerline*; both P and N are regenerated from the modified centerline so the pair stays symmetric. A **multipoint pair** (3+ terminals routed as a chain of coupled legs) is measured and meandered on its **longest MST leg** (#520): that leg's single-leg length is the pair's matching span, the centerline meanders are applied to that leg, and the result is spliced back into the chain.
- **Intra-pair matching** (`--diff-pair-intra-match`) — within one pair, meanders are added to the shorter of P/N to equalize them (`apply_intra_pair_length_matching()`). Bumps here are smaller (minimum amplitude 0.1mm) since they must fit between the pair and its surroundings; stub barrel lengths and polarity swaps are accounted for.

  **Every pair in the run is visited, and every pair produces a record (#766).** The pass is driven by the run's pair list (`run_intra_pair_matching()`), *not* by the shape of the routing results — it used to walk `routed_results` and require each result to carry an `is_diff_pair` / `p_net_id` stamp, which the **direct-hybrid escape** (coupled middle + point-to-point terminal legs) does not apply. A pair routed that way was skipped *without printing anything*, while the run still counted it routed and its P/N skew shipped unmeasured. Measured: a 10-pair LVDS board routed 4 pairs, all four via the hybrid escape, and matched **none** of them (1.19–7.25mm skew); the same run now matches all four (two inside a 0.05mm tolerance, the other two reported with their residual skew).

  The outcome per pair reaches `JSON_SUMMARY` as `intra_pair_matching`: `{pair, p_net, n_net, status, reason, delta_mm, delta_before_mm}`, with `status` one of `matched`, `improved` (skew reduced but short of tolerance), `within-tolerance`, `unmatched` (has pair copper, could not meander — see `reason`), `not-matchable` (no single in-run pair result to meander; `delta_mm` is then measured **off the board** so the skew is still reported), or `skipped` (handled end-to-end by the AC-coupled pass). Gate on this rather than on `successful`, which says nothing about skew.
- **End-to-end AC-coupled matching** (`--ac-couple-match`) — a pair split into two base-named pairs by series DC-blocking caps (the common PCIe/USB3/SATA TX case) is auto-detected as one *extended net* / XNet: a 2-pad cap bridging `A_P↔B_P` plus another bridging `A_N↔B_N`. Its concatenated P path (`A_P+B_P`) is matched against the concatenated N path (`A_N+B_N`) — the skew the receiver actually sees — with the compensating meander placed on whichever segment has room (`apply_ac_coupled_length_matching()`, sharing the meander engine with intra-pair via `_lengthen_net_with_meanders()`). This **supersedes** per-side intra-pair matching for the member pairs, reports the end-to-end skew in the JSON summary (`ac_coupled_xnets`), and is off by default. No-pop (DNP) caps are open circuits and are not stitched; only symmetric cap pairs (both polarities bridged) are joined. Detection requires the pair halves to be found first (so it inherits the `_P`/`_N` naming rules above).

## Time Matching

`--time-matching` matches propagation delay instead of physical length. Signals travel faster on outer layers (microstrip — field partly in air) than inner layers (stripline — field fully in dielectric), so equal lengths do not mean equal delays on multi-layer routes.

For each segment, the delay is `length × ps_per_mm(layer)` where:

- Microstrip (outer layers): `ε_eff = (ε_r + 1) / 2`
- Stripline (inner layers): `ε_eff = ε_r`
- `ps_per_mm = √ε_eff / c` with `c = 299.792458 mm/ns`

The dielectric constant `ε_r` per layer comes from the board stackup. Via barrels use a thickness-weighted average `ε_eff` across the spanned dielectric layers (`get_via_barrel_epsilon_eff()`). Without stackup data, FR4 is assumed (`ε_r = 4.3`, microstrip `ε_eff ≈ 2.65`, ≈5.9 ps/mm).

To size the meanders, the required extra delay is converted to extra length using the `ps_per_mm` of the route's *primary layer* (the layer carrying the most length).

## Limitations

- Meanders spill across multiple straight runs when one run cannot absorb the required extra length; if the runs together still cannot absorb it within clearance limits, the route may end outside tolerance (a warning is printed).

## Protected Nets (#521)

Matching is a per-step feature but chains are multi-step: a later retry that runs `--rip-existing-nets` over matched nets would rip the meanders and reroute at natural length, silently voiding the group. To prevent that, **every matched group member and every routed diff-pair member is recorded as a *protected net*** in the sibling `.kicad_pro` (under `kicad_routing_tools.protected_nets`, next to the DRC-floor writeback — so the list flows down the chain automatically and `copy_board.py` carries it):

- `route.py --rip-existing-nets` **excludes protected nets from collateral rips** (a printed line lists the exclusions);
- `repair_planes.py --rip-blocker-nets` never selects a protected net as a blocker to rip;
- **override**: naming a net *exactly* (no glob) in `--nets` or `--rip-existing-nets` is the deliberate signal and lifts its protection for that step. There is no flag; edit the `.kicad_pro` to remove entries permanently.
- **KiCad-locked copper**: a net with any `(locked yes)` segment or via is never rip-eligible, **with no override** — locked means never. This is read straight from the board (both parse paths), not from the `.kicad_pro`.

The GUI inherits the same behavior (engine-side reads via the board's sibling project file; the AI-plan executor and per-step floor updates persist new entries).

### Impedance declarations survive redos

Impedance-routed nets stay *rippable* — but a step run with `--impedance` records each targeted net's declaration (`ohms`, `differential`, `pair_gap`, `coplanar_gap`) in the `.kicad_pro` (`kicad_routing_tools.net_impedance`). A later step that touches those nets **without** `--impedance` recomputes the same widths from the stackup and applies them per-net (`route.py`; the declaration — not the widths — is stored, so a stackup change recomputes correctly). `route_diff.py` reapplies call-level when every targeted pair shares one stored differential spec (the diff engine's obstacle stamps carry one width map), and warns when specs are mixed. `check_impedance.py` auto-reads the same records and audits every net against **its own** declared coplanar gap — no `--coplanar-gap` flag needed on boards routed by this tool.
- Very tight surroundings can force the amplitude to its 0.2mm minimum, limiting how much length a run can add.
- Meanders are not placed across layer changes.
- `--ac-couple-match` runs after `--length-match-group`; a pair that is both an XNet member and in a match group may be meandered again by the end-to-end pass, perturbing its group-matched length. Keep AC-coupled pairs out of match groups.

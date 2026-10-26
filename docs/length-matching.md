# Length Matching and Time Matching

This document describes how the router matches route lengths (or propagation times) within groups of nets, e.g. DDR4 byte lanes. Length matching adds trombone-style meanders to shorter routes until every net in a group is within tolerance of the longest one.

Implementation: `length_matching.py`, with propagation-delay calculations in `impedance.py`.

## Overview

Length matching runs after the nets in a group have been routed:

1. Measure each route's length (or propagation time), including via barrels and each end pad's pad-to-die length (see [Pad-to-Die Length](#pad-to-die-length)).
2. Find the longest route in the group — it becomes the target.
3. For each shorter route, add meanders to a straight run of the route until its length is within tolerance of the target.

For multi-point nets this happens between Phase 1 and Phase 3 of the MST-based approach: the longest MST edge is routed first, meanders are applied to that clean 2-point path, and the remaining taps are routed afterwards (see [Routing Architecture](routing-architecture.md)).

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--length-match-group <patterns...>` | none | Define a match group by net name patterns (repeatable, one group per flag). The special value `auto` groups DRAM nets (DDR2 to DDR5, LPDDR4/5) by byte lane and command/address (see [DDR Auto-Grouping](#ddr-auto-grouping)) |
| `--length-match-tolerance` | 0.1 | Acceptable length variance within a group (mm) |
| `--meander-amplitude` | 1.0 | Maximum meander bump height perpendicular to the trace (mm) |
| `--meander-spacing` | 2.0 | Centre-to-centre spacing of adjacent meander arms, in multiples of the net's routed track width (2.0 = 2W pitch = 1W edge gap). On `route_diff.py` it sets the intra-pair and AC-coupled arms; pair-centreline meanders scale with the pair pitch instead (`--diff-chamfer-extra`) |
| `--diff-chamfer-extra` | 1.5 | (`route_diff.py`) Chamfer of a pair-centreline meander, as a multiple of the P/N half-pitch (>1 keeps P and N from crossing at the bends) |
| `--time-matching` | false | Match propagation delay instead of physical length |
| `--time-match-tolerance` | 1.0 | Acceptable delay variance within a group (ps) |
| `--diff-pair-intra-match` | false | (`route_diff.py`) Match P and N lengths *within* each differential pair |
| `--ac-couple-match` | false | (`route_diff.py`) End-to-end length-match AC-coupled pairs split by series DC-blocking caps (#196): match the concatenated P path vs N path across the caps |

Examples:

```bash
# Manual group: all DQ0-7 and DQS0 nets matched together
python3 py_router/route.py board.kicad_pcb --length-match-group "*DQ[0-7]" "*DQS0*"

# Two separate groups
python3 py_router/route.py board.kicad_pcb \
    --length-match-group "*DQ[0-7]" "*DQS0*" \
    --length-match-group "*DQ1[0-5]" "*DQ[8-9]" "*DQS1*"

# Auto-detect DRAM byte lanes and command/address
python3 py_router/route.py board.kicad_pcb --length-match-group auto

# Match propagation time instead of length
python3 py_router/route.py board.kicad_pcb --length-match-group auto \
    --time-matching --time-match-tolerance 1.0
```

## Group Specification

Each `--length-match-group` flag defines one group; nets matching *any* pattern in the flag belong to that group. Patterns support `*` wildcards and `[0-9]` character ranges (`match_net_pattern()`).

### DDR Auto-Grouping

`--length-match-group auto` calls `auto_group_ddr4_nets()` (classifier: `classify_ddr_nets()`), which groups by name. The same engine call serves the GUI: typing `auto` in the Length-match groups field reaches it through `batch_route`.

**Which nets it sees.** Only the nets this call was asked to route (its `--nets`, before the already-connected filter). A route step also registers pre-existing nets as rip candidates, and those are never grouped (#1247). A named member that an earlier step routed still joins and is measured from the board, as for named groups.

**Whole tokens, not substrings.** The name is split on non-alphanumerics (KiCad's `~{..}` overbar and `{slash}` are removed first) and read from the right. Short trailing tokens are polarity, active-low, channel or bit markers (`+`/`-`, `_P`/`_N`, `_t`/`_c`, `#`, `_n`, `_A`/`_B`), and are skipped. The first longer token must itself be the signal. So `WE` is never found inside `power`, `CK` never inside `JTAG_TCK`, and `Net-(U1-PadA15)` is not address line 15.

**An interface prefix must prove it is DRAM.** Everything before the signal token is the interface prefix (`RAM_`, `/DDR3/DDR3_`, `Net-(U1A-`). A generic token (`A<n>`, `D<n>`, `CS`, `WE`, `CK`/`CLK`, `CMD`) joins only under a prefix whose nets carry at least two DRAM-only kinds on the board. The kinds are DQ, DQS, a numbered or halved mask, BA, BG, CA, CKE, ODT and ACT, with RAS-or-CAS counted once. So `IO_A0`, `/sheetIO/A0`, `OLED_CS` and `SD0_CMD` stay out, and so does a NOR flash that names its data `DQ0`-`DQ15`. The prefix is judged over the whole board, so a step that routes only the address nets still recognises its interface.

**Groups:**

- **Byte lanes** `byte_lane_<n>`: data `DQ<n>`/`D<n>`/`DATA<n>` in lane `n // 8`. Strobes (`DQS<n>`, `RDQS<n>`, `LDQS`/`UDQS`) and masks (`DM<n>`, `DQM<n>`, `LDM`/`UDM`, `DQMH`/`DQML`, LPDDR4/5 `DMI<n>`) go in their numbered or lower/upper lane. `DMI` (data-mask-inversion) is sampled with its byte's DQ on that byte's strobe, so it is matched with the lane. A `DQ<byte>_<bit>` interface (bit index runs 0-7) is read by byte. LPDDR5's write clock `WCK<n>` joins lane `n` too: it latches that byte's write data and times its reads, so it is matched with the lane as a strobe is (WCK-to-CK skew is trained, not routed). DDR4/DDR5 check bits `CB<n>` are the ECC lane after the interface's data lanes: lane 8 on a 64-bit DIMM, with `DQS8`, and lane 4 on a 32-bit DDR5 sub-channel, with `DQS4`. A bare `DQS`/`DM`/`WCK` joins only when its interface's data spans exactly one lane (an x8 part). Otherwise it is listed as ungrouped by name.
- **`command_address`**: `A<n>`, `BA<n>`, `BG<n>`, `BS<n>`, `CA<n>`, `RAS`, `CAS`, `WE`, `CS`, `CKE`, `ODT`, `ACT`, `PAR`, `CMD`, and the clock `CK`/`CLK`. The clock is in this group because the command/address bus is sampled on it and is matched to it. A DDR2 DIMM's chip select `S#0`/`S0#` (JEDEC's `S0#`) is read as `CS0`, and a clock enable spelled `CK_E` as `CKE`.

Accepted spellings include the active-low `n` prefix (`nCAS`), `N`/`B` suffixes (`CSn`, `RASN`), and SDRAM-controller `S`/`SD` prefixes (Allwinner `SDQS0P`/`SA0`, i.MX `SDCLK0_P`, STM32 `SDNCAS`). The prefixes are never accepted on a data line (`SD0` is SD-card data) or on a bare `CK`/`CLK`/`CS` (`SCK`, `SCLK`, `SCS` are SPI).

**One set of groups per interface.** A byte lane is matched to its own strobe, so lanes of different interfaces are never one group. An interface is a prefix, plus a channel where the names carry one. A trailing `_A`/`_B` is a channel tag on a prefix where some signal carries both letters on the board (haasoscope's `DQ0_A`/`DQ0_B`); a lone `_B` stays active-low. Command/address is grouped per prefix. It splits by channel only when every channel that carries command nets also has its own clock. Otherwise the channels share one clock and one group. On haasoscope the `CK_t_A` net drives both channels' CK pins, so `CS0_B` is sampled on it.

The keys stay `byte_lane_<n>` and `command_address` on a board where that is unambiguous. Once two interfaces would share a key, every key names its interface: `byte_lane_0 @ Net-(U1A- channel A`, `command_address @ /LPDDR5/LPDDR5_A.`. The decision is made over the whole board, so a step's keys do not change with its `--nets`. Two prefixes on the two sides of a series resistor (`/GPIO/SDRAM.D0` and `/GPIO/SDRAM_D0_R`) are two interfaces as well, so each side is matched within itself.

**A second net reading as the same signal is the other side of a resistor.** On zynq7020_som, series resistor R53 joins `DDR_A0` (Zynq pin, about 4.5 mm away) to `DDR_A0_T` (DRAM pin, about 11 mm away). Both read as address line 0 under one prefix. A name that extends another by tokens that are no decoration of that signal is such a *variant*: `_T` here, where a single-ended signal takes only an active-low `N`/`B`/`L` or a channel letter, and `t`/`c`/`p` polarity belongs only to a clock or strobe. The variants that share a suffix form an interface of their own (`command_address @ /Zynq_B502_DDR/DDR_*_T`). So the long DRAM-side runs are matched to each other and never to the short Zynq-side stubs. `auto` does not match a line's total length through the resistor.

**What it reports.** When a group forms, its members are printed by name, before any meander. Next come the interface prefixes it accepted (with their channels), then every DRAM-looking net it left out, by name and reason:

- lane not readable;
- in the interface but no group token (`RESET`, `ZQ`, `VREF`);
- the name looks like DRAM but its prefix carries no DRAM-only signals.

It also names each variant and the net it extends.

`JSON_SUMMARY` carries the same membership as `length_match_groups` (one `{name, source, members}` record per group, named groups included, with `patterns`). The exclusions are in `length_match_auto`. The GUI's results carry `length_match_groups`. The #521 `length-matched` protected nets are drawn from these members: those routed this run or measured from earlier-step copper. The same members are safe within the run. Matching runs before Phase 3, whose seam re-ask (#444) rips a composed multipoint tree and re-routes it whole. That re-ask skips every member of a group this pass formed, `auto` or named, so a meandered main route is never re-routed at natural length.

A name the rules do not cover is listed as ungrouped, never guessed. Pass explicit `--length-match-group` patterns for it.

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

Before placing each bump, `get_safe_amplitude_at_point()` checks the bump area against nearby segments, vias, and pads using a spatial index (`ClearanceIndex`, 2mm cells), and against the board outline and cutouts:

- Clearance margin: half of each track's width (the foreign copper at its real width) plus the clearance, plus a corner allowance for the 45° chamfers and half a grid step for output merging. The clearance is the one `check_drc` grades that pair of nets at on that layer (net classes and `.kicad_dru` rules, #1136); a board with neither uses the flat `--clearance`.
- The board edge is kept at `--board-edge-clearance` (else the clearance) plus half the track width.
- If the full amplitude doesn't fit, the amplitude is reduced in 0.7× steps (binary-search style) down to a 0.2mm minimum.
- If neither the up nor the down direction fits at all, the bump position is skipped forward 0.2mm and tried again.

If the resulting meander under-shoots the target, the algorithm iterates: more bumps are added, or the amplitude is rescaled, until the route is within tolerance or the run is exhausted (`_apply_meanders_to_net_with_iteration()`).

## Via Barrel Length

Route lengths include the vertical distance through vias so that results match KiCad's length measurements:

- `PCBData.get_via_barrel_length(layer1, layer2)` sums the board stackup's layer thicknesses between the two copper layers.
- **Stub via barrels**: when a BGA fanout stub switches layers, the barrel length from the *stub layer to the pad layer* is included (`calculate_stub_via_barrel_length()` in `connectivity.py`), not the via's full span.
- **The graders** (`net_copper_length` / `net_match_length` in `board_score --length-groups` and check_drc's `.kicad_dru` length windows) charge each via over the layers it actually joins and cut runs at pad (and, from 10.0.4, via) outlines, as KiCad's length DRC does -- measured as the installed kicad-cli's release measures them, since the measure changed within KiCad 10.0 (`net_queries.kicad_net_length_nm`, `length_measure_release`, #1313, #1432). The router's own route lengths still charge a through via's full span, so on a board with partial-span vias the router's matched length and the graded one can differ by those barrels.

Without stackup information in the board file, via barrels count as zero length.

## Pad-to-Die Length

A pad may declare the length of the bond wire or package trace inside the part,
`(die_length <mm>)` in KiCad's pad properties ("Pad to die length"), and KiCad 10
a delay, `(die_delay <ps>)`. KiCad adds both to the net in its length tuners and
its length/skew DRC, so matching on copper alone ships the difference as skew
(#1248). Both parse paths read them into `Pad.die_length` / `Pad.die_delay`, and
every matching pass adds them before it picks the target and the deltas:

- **Group matching** adds each member's pad-to-die (`member_pad_to_die()`): every
  pad of a 2-pad net (KiCad's tuner adds the start and end pads, its DRC every
  pad, the same set here), the two pads of a multipoint net's matched main edge,
  every pad of a member measured from earlier copper (its length is the net's
  total copper, as in KiCad's DRC). The meanders still lengthen copper; the run
  prints `Pad-to-die included`.
- **A differential pair** is measured as KiCad's pair tuner measures it
  (`DP_MEANDER_PLACER::origPathLength`): per leg, the leg's own copper (its
  stub included) plus its own start and end pads' term, and the pair is the
  larger leg. So legs whose copper differs -- a pair that turns, matched
  without `--diff-pair-intra-match` -- are exact, where adding the larger leg's
  term to the pair's copper is exact only while the legs are matched. A
  multipoint pair is matched on its longest coupled leg, and only that leg's
  two terminals' pads count, not every pad of each net. A pair whose pads
  declare no term keeps its copper figure exactly (without intra-match, the
  centreline average), so a board without the tokens routes identical copper;
  the run prints each pair's legs (`pair, larger leg of P ..., N ...`).
- **Time matching** adds each pad's `die_delay` in ps, a pair per leg the same
  way. KiCad 10 never converts a `die_length` into a delay, and neither does
  this.
- **Intra-pair** (`--diff-pair-intra-match`) and **AC-coupled** matching add each
  leg's own pads' pad-to-die, as KiCad's skew tuner does.

A board that declares neither token matches exactly as before. The graders read
the same term: `board_score.py --length-groups` (`net_queries.net_match_length`).

## Multi-Layer Routes

Routes containing vias are supported, with restrictions:

- Meanders are only placed on straight runs that stay on a single layer.
- Via positions are preserved; for differential pairs the P/N tracks are regenerated from the meandered centerline and GND return vias are re-created afterwards.

## Differential Pairs

Two forms of matching exist for differential pairs (`route_diff.py`):

- **Group matching** — multiple pairs in a `--length-match-group` are matched pair-to-pair by meandering each pair's *centerline*; both P and N are regenerated from the modified centerline so the pair stays symmetric. A **multipoint pair** (3+ terminals routed as a chain of coupled legs) is measured and meandered on its **longest MST leg** (#520): that leg's single-leg length is the pair's matching span, the centerline meanders are applied to that leg, and the result is spliced back into the chain.
- **Intra-pair matching** (`--diff-pair-intra-match`) — within one pair, meanders are added to the shorter of P/N to equalize them (`apply_intra_pair_length_matching()`). Bumps here are smaller (minimum amplitude 0.1mm) since they must fit between the pair and its surroundings. The two legs are measured **on the board's copper** (#1303, `net_queries.net_signal_length`: a 2-pad member's pad-to-pad path, a larger one's copper with dead-end branches trimmed, plus pad-to-die), so the meander lands on the leg that is shorter on the board -- the routing result after a hybrid escape or seam re-ask carries no stub lengths, and on imx6ulz DQS0 it put the meander on the longer leg (skew 4.51 → 5.20 mm; measured on the board, 0.024 mm). A member whose pads are not joined yet is matched on the result's own sums (stub barrel lengths and polarity swaps accounted for). A multipoint pair's meander is spliced into the leg dict that held the replaced copper, as group matching's is (the writer emits the legs).

  **Every pair in the run is visited, and every pair produces a record (#766).** The pass is driven by the run's pair list (`run_intra_pair_matching()`), *not* by the shape of the routing results — it used to walk `routed_results` and require each result to carry an `is_diff_pair` / `p_net_id` stamp, which the **direct-hybrid escape** (coupled middle + point-to-point terminal legs) does not apply. A pair routed that way was skipped *without printing anything*, while the run still counted it routed and its P/N skew shipped unmeasured. Measured: a 10-pair LVDS board routed 4 pairs, all four via the hybrid escape, and matched **none** of them (1.19–7.25mm skew); the same run now matches all four (two inside a 0.05mm tolerance, the other two reported with their residual skew).

  The outcome per pair reaches `JSON_SUMMARY` as `intra_pair_matching`: `{pair, p_net, n_net, status, reason, delta_mm, delta_before_mm, matcher_status, matcher_delta_mm}`, with `status` one of `matched`, `within-tolerance`, `unmatched` (over tolerance — see `reason`), `not-matchable` (no single in-run pair result to meander), or `skipped` (handled end-to-end by the AC-coupled pass). Gate on this rather than on `successful`, which says nothing about skew.

  **Both `status` and `delta_mm` are read off the shipped copper** (#1303), once the board is final: `delta_mm` is the board's P/N skew as KiCad's skew DRC measures it (`net_queries.diff_pair_skew`: each member's `net_match_length`, mode `total`). The matcher measures its own result instead (segments plus stub lengths recorded at routing time), and after a seam re-ask that is not the copper that ships: on imx6ulz it read DQS0 as P 12.00 < N 13.08 and meandered P while the board had P 21.33 > N 16.82, so the skew went 4.51 → 5.20 mm under a printed "improved". `matcher_status` / `matcher_delta_mm` keep what the matcher reported; a matcher `matched` that the copper contradicts is `unmatched` with reason `copper-skew-over-tolerance`, and one whose pads are not joined is `unmatched` / `not-measured`. The GUI's completion dialog names the pairs not matched on the board. A hard skew budget belongs in the board's `.kicad_dru` as `(constraint skew (max ..) (within_diff_pairs))`: `check_drc.py` grades it as KiCad does (check 17 in [utilities](utilities.md#checks-performed)) and `board_score` counts it in `diff_pairs`.
- **End-to-end AC-coupled matching** (`--ac-couple-match`) — a pair split into two base-named pairs by series DC-blocking caps (the common PCIe/USB3/SATA TX case) is auto-detected as one *extended net* / XNet: a 2-pad cap bridging `A_P↔B_P` plus another bridging `A_N↔B_N`. "2-pad" counts copper pads, not paste-only windows (#1251: a 0201 resistor drawn with two `F.Paste` windows beside its copper pads is still a series part). Its concatenated P path (`A_P+B_P`) is matched against the concatenated N path (`A_N+B_N`) — the skew the receiver actually sees — with the compensating meander placed on whichever segment has room (`apply_ac_coupled_length_matching()`, sharing the meander engine with intra-pair via `_lengthen_net_with_meanders()`). This **supersedes** per-side intra-pair matching for the member pairs, reports the end-to-end skew in the JSON summary (`ac_coupled_xnets`), and is off by default. No-pop (DNP) caps are open circuits and are not stitched; only symmetric cap pairs (both polarities bridged) are joined. Detection requires the pair halves to be found first (so it inherits the `_P`/`_N` naming rules above).

## Time Matching

`--time-matching` matches propagation delay instead of physical length. Signals travel faster on outer layers (microstrip — field partly in air) than inner layers (stripline — field fully in dielectric), so equal lengths do not mean equal delays on multi-layer routes.

For each segment, the delay is `length × ps_per_mm(layer)` where:

- Microstrip (outer layers): `ε_eff = (ε_r + 1) / 2`
- Stripline (inner layers): `ε_eff = ε_r`
- `ps_per_mm = √ε_eff / c` with `c = 0.299792458 mm/ps`

The dielectric constant `ε_r` per layer comes from the board stackup. Via barrels use a thickness-weighted average `ε_eff` across the spanned dielectric layers (`get_via_barrel_epsilon_eff()`). At FR4's `ε_r = 4.3` an outer layer runs ≈5.4 ps/mm (`ε_eff = 2.65`) and an inner layer ≈6.9 ps/mm. A layer the stackup does not describe (or a board with no stackup) takes `ε_eff = 4.0`, ≈6.7 ps/mm, on every layer alike, so on such a board time matching orders routes exactly as length matching does.

To size the meanders, the required extra delay is converted to extra length using the `ps_per_mm` of the route's *primary layer* (the layer carrying the most length).

## Limitations

- **A `.kicad_dru` length window is aimed at only by a match group** (#1313, #1391). `(constraint length (min ..) (max ..))` rules are read by `check_drc.py` (check 16 in [utilities](utilities.md#checks-performed)), which reports each net outside its window as `length-out-of-range`, as KiCad reports `length_out_of_range`; `board_score` counts them in its `drc` component. A `--length-match-group` reads the window that binds its members and moves the group target into `[min + tolerance, max - tolerance]`, so short members meander up to the window and never past it; a member already longer than the window cannot be shortened and is reported. No other step lengthens or shortens a net to meet a window. Every member's verdict is in `length_match_groups[].member_status` (`matched`, `unmatched`, `unmeanderable`, `over_target`, `unmatched_board_copper`, with `length_mm`, `target_mm`, `shortfall_mm` and, under a window, `window_mm` and `in_window`). Board copper that matches the group's patterns joins the group and sets its target, so one group routed in two calls keeps one target. check_drc's grade is still what confirms the window.
- Meanders spill across multiple straight runs when one run cannot absorb the required extra length; if the runs together still cannot absorb it within clearance limits, the route may end outside tolerance (a warning is printed).
- Very tight surroundings can force the amplitude to its 0.2mm minimum, limiting how much length a run can add.
- Meanders are not placed across layer changes.
- `--ac-couple-match` runs after `--length-match-group`; a pair that is both an XNet member and in a match group may be meandered again by the end-to-end pass, perturbing its group-matched length. Keep AC-coupled pairs out of match groups.

## Protected Nets (#521)

Matching is a per-step feature but chains are multi-step: a later retry that runs `--rip-existing-nets` over matched nets would rip the meanders and reroute at natural length, silently voiding the group. To prevent that, **every matched group member and every routed diff-pair member whose shipped copper is coupled (#1303: a pair routed as two single-ended members is not) is recorded as a *protected net*** in the sibling `.kicad_pro` (under `kicad_routing_tools.protected_nets`, next to the DRC-floor writeback — so the list flows down the chain automatically and `copy_board.py` carries it). The GUI also keeps the whole `kicad_routing_tools` namespace on the board itself (a top-level `(property "kicad_routing_tools" "...")` that KiCad's save writes into the `.kicad_pcb`), because KiCad's own project save rewrites the `.kicad_pro` from the copy it loaded and drops a record written after load; each copy carries `records_rev`, and every reader takes the newer (`py_router/tool_records.py`). "Routed diff-pair member" includes a pair taken by the 2-terminal DIRECT HYBRID escape (#906). That escape's result carries `hybrid_escape` and *not* `is_diff_pair` — the latter means a *coupled* constructor committed the copper, which custody reads for a different question — but a hybrid pair is a coupled middle with point-to-point terminal legs, and its geometry is exactly the invariant this record exists to protect. (A multipoint leg that took the hybrid produces a merged result carrying **both** keys; it was always recorded.) The one hybrid that is *not* protected is the self-graze fallback, kept when no layer couples cleanly and therefore shipping P/N below clearance — protecting that would make the step whose job is to fix it skip the pair. The record itself:

- `route.py --rip-existing-nets` **excludes protected nets from collateral rips** (a printed line lists the exclusions);
- `repair_planes.py --rip-blocker-nets` never selects a protected net as a blocker to rip;
- **override**: naming a net *exactly* (no glob) in `--nets` or `--rip-existing-nets` is the deliberate signal and lifts its protection for that step. There is no flag; to remove entries permanently, edit the copy readers take: the `.kicad_pro`, or the board's `kicad_routing_tools` property when that carries the higher `records_rev` (after a GUI session and a KiCad save).
- **KiCad-locked copper**: a net with any `(locked yes)` segment or via is never rip-eligible, **with no override** — locked means never. This is read straight from the board (both parse paths), not from the `.kicad_pro`.

The GUI inherits the same behavior (engine-side reads via the board's sibling project file; the AI-plan executor and per-step floor updates persist new entries).

### Impedance declarations survive redos

Impedance-routed nets stay *rippable* — but a step run with `--impedance` records each targeted net's declaration (`ohms`, `differential`, `pair_gap`, `coplanar_gap`) in the `.kicad_pro` (`kicad_routing_tools.net_impedance`), plus **`applied`** (#906). `applied: false` means no layer width was actually solved, so the nets routed at the plain track width and the copper is *not* impedance-controlled: `check_impedance` skips such a declaration rather than grading nets against an impedance the router never attempted, and the step says so on the console. The declaration is still recorded, because it is the declaration — not the widths — that makes a later recompute possible. Note `applied` is decided by whether a width **solved**, not by whether the board has a stackup: a stackup listing copper with no adjacent dielectric, or with names that do not match the routed layers, solves nothing and falls back silently. Both `route.py` and `route_diff.py` record it. A later step that touches those nets **without** `--impedance` recomputes the same widths from the stackup and applies them per-net (`route.py`; the declaration — not the widths — is stored, so a stackup change recomputes correctly). `route_diff.py` reapplies call-level when every targeted pair shares one stored differential spec (the diff engine's obstacle stamps carry one width map), and warns when specs are mixed. `check_impedance.py` auto-reads the same records and audits every net against **its own** declared coplanar gap — no `--coplanar-gap` flag needed on boards routed by this tool.

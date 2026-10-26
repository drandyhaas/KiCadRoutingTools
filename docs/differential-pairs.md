# Differential Pair Routing

This document describes how the router handles differential pairs, including P/N pairing, centerline routing, and via placement.

## Overview

Differential pairs are routed using a **centerline + offset** approach with **pose-based A* routing**:

1. **Setback position finding** - Find clear launch positions in front of the stubs, over a ladder of setback distances, each scanned at 9 angles (0°, ±max/4, ±max/2, ±3max/4, ±max)
2. **Pose-based centerline routing** - Route using orientation-aware A* with Dubins path heuristic
3. **Path simplification** - Remove collinear points and in-place turns for cleaner geometry
4. **P/N offset generation** - Create P and N paths as perpendicular offsets from centerline
5. **Via handling** - Add approach/exit tracks at layer changes to keep P/N parallel

The pose-based approach treats routing as searching through (x, y, θ) space, ensuring routes properly respect entry/exit angles at pads.

## Identifying Differential Pairs

The router recognizes common differential pair naming conventions:

| Convention | Example P | Example N |
|------------|-----------|-----------|
| `_P` / `_N` suffix | `LVDS_CLK_P` | `LVDS_CLK_N` |
| `_PX` / `_NX` indexed suffix | `FE_CLK_P0`, `FE_CLK_P1` | `FE_CLK_N0`, `FE_CLK_N1` |
| `_t` / `_c` suffix (case-insensitive) | `DQS0_t_A`, `CK_T` | `DQS0_c_A`, `CK_C` |
| `_tX` / `_cX` (no-separator channel) | `DQS0_TA` | `DQS0_CA` |
| `P` / `N` suffix (after a digit, `_` or capital) | `DATA0P`, `TXP` | `DATA0N`, `TXN` |
| `DP` / `DM` / `DN`, `DPLUS` / `DMINUS` (USB) | `USB_DP`, `USB_DPLUS` | `USB_DM`, `USB_DN`, `USB_DMINUS` |
| `+` / `-` suffix | `CLK+` | `CLK-` |
| `+` / `-` before an `_`-led suffix | `D+_L` | `D-_L` |

For the indexed `_PX` / `_NX` convention the trailing index is kept in the base
name, so each index pairs only with its own twin (`FE_CLK_P0` pairs with
`FE_CLK_N0`, never with `FE_CLK_N1`). For the `DP`/`DM`/`DN` USB convention, the
`P` half is positive and both `N` and `M` are treated as the negative half (so
`/USB_DP` pairs with either `/USB_DN` or `/USB_DM`).

KiCad auto-named nets such as `Net-(U12-USB_D+)` / `Net-(U12-USB_D-)` are
recognized too: the wrapping `Net-(…)` is peeled before the suffix is matched.

Pairing is suffix-style aware: nets only pair when both use the **same**
convention. For example `/CLK+` pairs with `/CLK-` but never with an unrelated
`/CLK_N` net, even though both reduce to the base name `/CLK`. This prevents
boards that mix conventions (e.g. an LVDS input pair `CLK+`/`CLK-` alongside a
single-ended inverted output `CLK_N`) from producing corrupted pairs.

## Usage

Use `route_diff.py` for differential pair routing. All nets specified are treated as differential pairs:

```bash
# Route a specific diff pair
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds_rx1_11*"

# Route with debug visualization
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds_rx1_11*" \
    --debug-lines

# Route all LVDS nets with custom gap
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --diff-pair-gap 0.1 \
    --diff-pair-centerline-setback 1.5

# Keep diff pairs out of a keepout polygon drawn on a User layer (issue #27)
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --keepout --keepout-layer User.2

# 4+ layer board: pass EVERY copper layer (issue #116)
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --layers F.Cu In1.Cu In2.Cu B.Cu
```

Nets with _P/_N, P/N, or +/- suffixes will be paired automatically.

A `--nets` (or positional) pattern token may list several patterns separated by
commas, e.g. `--nets "/DVI_CK_P,/DVI_CK_N"`; the token is split so each pattern
is matched on its own rather than as a single glob containing a comma (which
would match nothing). `route_diff.py` also warns loudly about any pattern that
matches no real net on the board, instead of silently dropping it (issue #143).

**Escape layers (4+ layer boards):** `--layers` defaults to `F.Cu B.Cu` only.
When a pair was escaped onto an INNER layer by `bga_fanout.py`, `route_diff.py`
can only launch from those escaped stubs if that inner layer is in `--layers` —
omitting it strands the inner-layer stubs and silently drops those pairs
(butterstick: 8/40 pairs without, 22/40 with the full layer list). Pass the same
copper-layer list you gave `bga_fanout.py`; drop `--layers` only for true 2-layer
boards (issue #116).

Differential pairs respect user-drawn keepout polygons (`--keepout`, default layer
`User.2`), KiCad native keep-out rule areas (`(zone … (keepout …))`, honoured
automatically) and net-filtered keep-outs declared by `disallow` rules in the
board's `.kicad_dru` (#1278). See [configuration.md](configuration.md#keepout-zone-options) for details.

## Centerline Routing

### Endpoint Detection

The router finds paired P/N endpoints by:

1. Getting free ends of P and N stub groups
2. Finding closest P-N pairs on the same layer
3. Designating one pair as source, other as target

### Setback from Stubs

The centerline starts a configurable distance ("setback") in front of the stubs, in the direction the stubs are pointing:

```
Stub tips:  P---o       o---P
                 \     /
Centerline:       o---o
                  ^
                  Setback distance (diff-pair-centerline-setback)
```

The centerline endpoints are positioned:
- At the midpoint between P and N stub tips
- Offset by the setback distance in the stub direction

### Bare-Pad Endpoints (No Fanout Stubs)

When a pair connects directly to bare SMD pads (no stub segments, e.g. SOIC
pins or a termination resistor), there is no stub direction to derive the
setback from. In that case the router synthesizes an escape direction:

1. Perpendicular to the P-N pad axis
2. Pointing away from the owning component's pad centroid (so pin-row pads
   escape away from the chip body)
3. For symmetric parts (e.g. a 2-pad resistor), pointing toward the other end
   of the route
4. If the chosen side is blocked, the opposite side is tried

Additionally, the pair's **own pads** are added as obstacles for the
centerline (`add_diff_pair_own_pads_as_obstacles`), since the pair's nets are
otherwise excluded from the obstacle map and the offset P/N tracks could cross
the partner polarity's pad mid-route. Capsule-shaped corridors from each
pad-pair center out past the setback position stay open so the route can still
reach its endpoints and fan out to the pads.

### Bare-Pad Target on a Different Layer (Connector Fanout)

When the target is a bare outer-layer pad (F.Cu/B.Cu) with no stub and the
pair's source copper is on a different layer, the surface approach can be
blocked - e.g. a connector pin in the **front row of a 2-row header** is boxed
in between the board edge and the tall back-row pads, so an inner-layer pair
can't reach it through the surface channel. The upfront layer-swap pass handles
this with a **bare-pad target swap**: it drops a through-via on each pad and
grows a short stub on the source layer (aimed toward the source), turning the
bare pad into a stub the router lands on while the via carries the connection
back to the pad's outer layer. See the layer-optimization options in
[configuration.md](configuration.md#layer-optimization-options).

### Multi-Point Differential Pairs

A diff pair cannot tap onto the middle of an existing pair of tracks - the
tapping leg's P/N tracks would have to cross the existing pair. So pairs with
3+ pad-pair terminals (e.g. connector -> termination resistor -> IC pins) are
routed as a **chain** of 2-point legs (`diff_pair_multipoint.py`):

1. The pair's pads are grouped into (P pad, N pad) terminals by nearest
   matching (connector pins, IC input pairs, termination resistors). A pad of
   a declared Kelvin sense branch (a `sense:` group, #1279) is never a
   terminal; it joins its net at the branch's join pad.
2. Terminals are ordered as the shortest open chain. Each terminal has only
   two usable sides (the +/- directions of its escape axis), so the topology
   must be a path, not an MST tree. Orderings whose interior terminals have
   both neighbors on the same side of the escape axis are penalized (they
   force a wrap-around leg).
3. Legs are routed sequentially. A continuation leg leaves a shared terminal
   on the **opposite side** from the leg that arrived - the chain passes
   "through" the pads. The forced side gets a connector corridor exemption in
   the obstacle map (own pads AND the previous leg's tracks).
4. Polarity is resolved per leg: connector flips at the fresh terminal, and -
   when polarity fixing is enabled - pad swaps at **chain-fresh terminals
   only** (each leg's far terminal; never a shared terminal, which already
   has a routed leg attached). Swap and flip candidates compete by routed
   length. If a chain attempt is ripped out, its pad swaps are undone too.
5. If a leg fails (unroutable, or its P/N tracks cross), the attempt's legs
   are ripped out and the next-best chain ordering is tried - the side
   constraints depend on routing order, so a reversed chain can succeed
   where the forward one wraps itself into a corner.

### Blocked-Terminal Stub Switch (Last Resort)

A multipoint chain whose terminal sits in a dense ball field can fail every
chain ordering: the terminal's escape stubs are walled in on their own layer,
and no coupled launch swath exists there. Before the pair is deferred to
single-ended, the router MOVES such terminals' escape stubs (own layer ≥45%
blocked nearby) onto the most-open other layer with the standard stub layer
switch, validated by `validate_swap`. The switch's pad via sizes itself down
the fab-tier ladder when the nominal via doesn't fit the field (see
[Configuration](configuration.md), "Layer Optimization Options"). Moving a
stub also removes the copper that boxed its coupled twin — the two USB twins'
F.Cu escapes are each other's nearest walls, so switching the pair's stubs
unboxes both at once. If the chain retry still fails, the switch is reverted
exactly.

### Electrically-Short Pairs (Single-Ended Deferral)

A coupled pair only earns its keep over an *electrically long* run. A leg
shorter than a few millimetres has no real coupled middle section — you spend
~1 setback fanning in and ~1 fanning out — so coupling buys nothing and only
tangles the pair through clustered connector pads (e.g. a USB connector's
D+/D-). Such legs are routed **single-ended** instead: the router defers them,
and the downstream `route.py` single-ended pass connects them as plain tracks.

A leg is electrically short when `min(P-run, N-run) < threshold`, where
(`diff_pair_min_coupled_length`):

```
threshold = max( 5 × setback,        # geometric: keep short fan-ins from tangling
                 3.0 mm )            # electrical: λ/10 at 5 GHz on FR4
setback   = centerline_setback (if set) else 4 × (track_width + diff_pair_gap)/2
```

- A **2-terminal** pair whose single leg is short is deferred whole.
- A **multi-point** pair defers short legs individually; if *every* leg is
  short the whole pair is left for single-ended.

For a 2-terminal pair a run counts the member's **own copper** (#1362): the
copper already on its pads (a fanout escape, a layer-swap stub) plus the
straight gap from that copper's free tips, the points the pair router launches
from (`member_run_mm`). A pair is deferred only when it is short both pad to
pad and along those runs. Escapes that lead away from the far terminal make a
pair long however close its pads are, and it is routed coupled from the escape
tips, where the single-ended fallback could join one member through its ball
via and the other along its escape. Copper only lengthens a run, so this never
defers a pair that the pad-to-pad test routes coupled. The log says
`Short pad to pad, but its own escape copper ...` when it applies.

Being short is not the only way a pair is left whole for single-ended: a
multi-point pair whose coupled chain cannot be routed at all is deferred as
the fallback (#165), and so is a 2-terminal pair whose coupled route failed.
Each such pair is listed in `single_ended_diff_pairs` with `pair_reports[]`
`outcome: "deferred"`, and its `failure_reason` says which: `electrically-short`,
`coupled-route-failed` (the multi-point fallback), or the 2-terminal pair's
own failure code. The end-of-run `Single-ended:` line and the GUI's completion
dialog count the deferred pairs by that reason.

**Why the 3 mm absolute floor.** The geometric term alone (`5 × setback =
10 × (width+gap)` in the auto case) drops to only ~2 mm at tight pitch, which
is *below* the length at which coupling matters electrically. Below ~λ/10 at
the design's top frequency a pair is electrically short — SE vs coupled is
indistinguishable (no meaningful reflection, skew-induced common-mode, or
radiation difference). On FR4, `v = c/√εr_eff ≈ 145–173 mm/ns`
(stripline…microstrip), so at **5 GHz**, `λ = v/f ≈ 29–35 mm` and
`λ/10 ≈ 3 mm`. The floor therefore governs all pairs with
`width + gap ≤ 0.3 mm` (most real diff pairs); only wider pairs
(e.g. 0.2/0.2 → 4 mm) are bounded by the larger geometric term. The floor
assumes a ≤5 GHz design; a much faster board would warrant a smaller value.

This rule is shared by the CLI (`route_diff.py`) and the GUI. In the GUI's
Differential tab, the **"Hide short routes"** option (on by default) uses the
same test to drop these pairs from the differential pair list, and keeps their
nets visible on the Route tab — even under "Hide differential" — so they get
routed single-ended.

### Pose-Based Centerline Routing

The centerline is routed using orientation-aware A* search with state space (x, y, θ, layer):

```
Source stub                                              Target stub
    |                                                        |
    o----> setback ======== Dubins path ========= setback <---o
           position                               position
```

**State Space**: Each node is defined by position AND heading (θ discretized to 8 directions at 45° intervals).

**Dubins Heuristic**: Instead of Euclidean distance, the heuristic uses Dubins path length - the shortest curve connecting two poses with prescribed headings and minimum turning radius.

**Transitions**:
- Move forward in current direction (cost = distance)
- Turn in place by ±45° (cost based on arc length at min turning radius)
- Layer change via (keeps position and heading)

This produces routes that:
- Start in the stub direction at the source
- End approaching from the correct direction at the target
- Smoothly curve between orientations respecting the minimum turning radius

### Setback Position Finding

The setback (`--diff-pair-centerline-setback`, default 2× the P-N centre pitch) is the first rung of a ladder of distances (#90): the configured setback, then 0.75×, 0.5×, the geometric floor (the room the connector fan needs to taper from the pad separation to the pair pitch at ≤45°, at least 2× the half-pitch) and 1.5× and 2× the setback. At each distance the router scans 9 angles about the escape direction (`--max-setback-angle`, default 45°):

```python
# With max_setback_angle = 45° (default):
angles_deg = [0, 11.25, -11.25, 22.5, -22.5, 33.75, -33.75, 45, -45]
```

For each angle:
1. Check if the position is blocked in the obstacle map
2. Check if the connector path from stub center is clear

The first distance with a clear angle wins (0° is preferred unless it sits too close to a neighbouring stub). If no distance works, the ladder is re-run with the launch direction rotated 90°, −90° and 180°, so a terminal buried in a dense row can launch sideways or backwards; such a launch routes like a flipped end (it must wrap around its endpoint).

### Extra Clearance

The centerline is routed with extra clearance to accommodate both P and N tracks:

```python
# Extra clearance = offset from centerline to the P/N track centres
extra_clearance = (track_width + diff_pair_gap) / 2
```

It is added to the ordinary single-track keep-out (half the track width plus the clearance), so each offset track keeps its clearance from every obstacle.
The map carries this one offset, the call's, for every layer. A layer wider than the
pair's own width (an `--impedance` outer layer above `--track-width`) runs at its own
wider pitch (below), so a segment of it that comes inside a neighbour's clearance is
necked there when it is emitted, the same fab-floored neck the terminal legs get.

## P/N Path Generation

### Perpendicular Offsets

P and N paths are created as perpendicular offsets from the simplified centerline.
The offset is half the P/N centre pitch, `(width + gap) / 2`. With per-layer widths
(an `--impedance` solve, a stored impedance declaration, or a layer-scoped
`.kicad_dru` width rule) it is taken per layer from that layer's own width
(`layer_pair_spacing`), so a narrower stripline layer keeps the declared gap:

```python
def create_parallel_path_from_float(centerline_path, sign, spacing_mm, start_dir, end_dir):
    for i, (gx, gy, layer) in enumerate(centerline_path):
        # Calculate perpendicular direction using bisector at corners
        if i == 0:
            dx, dy = start_dir      # Use stub direction at start
        elif i == len(centerline_path) - 1:
            dx, dy = end_dir        # Use stub direction at end
        else:
            dx, dy = compute_bisector(prev, curr, next_pt)  # Bisector at corners

        # Apply perpendicular offset with corner scaling for miters
        perp_x = -dy * sign * spacing_mm * corner_scale
        perp_y = dx * sign * spacing_mm * corner_scale
        result.append((x + perp_x, y + perp_y, layer))
```

A layer-scoped `.kicad_dru` rule such as
`(rule "strip" (layer "In2.Cu") (constraint track_width (opt 0.154mm)))` sets the
pair width on that layer when neither `--track-width` nor `--impedance` is given
(the GUI: the width override unchecked) and no stored impedance is reapplied, as
`route.py` does for single-ended nets. Each member is resolved as KiCad resolves a
coupled-pair track: its net class's `diff_pair_width`, then the rules that match it
on that layer, clamped into their min/max and the fab floor. A layer no rule scopes
therefore keeps the class pair width, not the class's single-ended track width. The
run prints `Design-rule layer widths: ...`. The engine draws one width per layer per
call, so when the targeted pairs resolve different widths on a layer (pairs in
different classes, a rule scoped to some of them) the rules are not applied and the
run prints a NOTE naming the layer; route each group of pairs in its own call.

### Corner Handling (Miters)

At corners, the perpendicular offset is scaled using the bisector angle to maintain constant P-N spacing:

```python
# Corner compensation: scale offset by 2/length to maintain perpendicular distance
# When summing two unit vectors, length = 2*cos(theta/2)
corner_scale = min(2.0 / bisector_length, 3.0)  # Cap at 3x to avoid extreme miters
```

### Polarity Detection

P is assigned +1 or -1 based on which side of the centerline it's on at the source:

```python
# Cross product with path direction determines side
cross = path_dir_x * to_p_dy - path_dir_y * to_p_dx
src_p_sign = +1 if cross >= 0 else -1
```

The router also detects if polarity differs between source and target (P on the other side at the target end) and says how it resolves it:

```
  Polarity swap needed - will swap target pad and stub nets in output
  Polarity mismatch - trying target connectors out the opposite side
```

**Note:** For a pair `--polarity-swap-nets` allows (none by default), a mismatch can be fixed by swapping the target pad net assignments (P↔N) so polarity matches. The swap is applied consistently in three places:

- the in-memory `pcb_data` (pads and stub segments/vias), so post-route cleanup
  and subsequent routes see the swapped nets - without this, the appendix
  cleanup would collapse the connectors ending on the swapped pads, leaving
  gaps between tracks and pads
- the output file (CLI mode, via `write_routed_output`)
- the live pcbnew board (plugin mode, via `kicad_routing_plugin/board_swaps.py`)

The swap changes pad net assignments on the **board only** - update the
schematic to match (the CLI can do this with `--schematic-dir`).

There is a second, purely geometric resolution: re-routing with the
connectors taken out the **opposite side at one end** (flipping one end flips
its P/N handedness; flipping both would reintroduce the mismatch). Only
bare-pad endpoints can flip - stub directions are fixed by existing copper.
Flipped attempts get a full-loop turn budget since they must wrap around
their endpoint, and every candidate is validated for P/N track crossings.

For a pair with swaps allowed (`--polarity-swap-nets`), the pad-swap and
connector-flip
candidates **compete by routed length** and the shortest clean route wins; if
only one mechanism succeeds, it is used. (When no end can flip - e.g. both
ends have stubs - the swap is committed directly without re-routing.)

For a pair NOT matching `--polarity-swap-nets` (or with the flag absent
entirely - the default), pad swaps never
happen: only the flip resolution is tried, and if no flip produces a clean
route the pair is **skipped** with a warning (no crossing tracks are ever
written).

## Via Placement

### Collinear Via Constraint

The Rust router enforces a **collinear via constraint** for differential pair routing to ensure clean via geometry. The constraint requires:

1. **2 steps minimum before via** - At least 2 grid steps must exist before placing a via
2. **Approach direction within ±45°** - The approach direction must be within ±45° of the previous direction
3. **Exit same as approach** - After the via, must continue in the same direction
4. **Then ±45° allowed** - After the exit step, can turn up to ±45°

This creates symmetric geometry around vias: `±45° → D → VIA → D → ±45°`

### Via Cost

The via cost is **doubled** for differential pairs since each layer change requires placing two vias (P and N). This discourages unnecessary layer transitions.

### Via Exclusion Zones

The router tracks via positions along the centerline path and enforces exclusion zones to prevent P/N offset tracks from conflicting with P/N vias. When a via is placed:

1. **Re-entry blocked** - After escaping the exclusion radius, the route cannot re-enter
2. **Perpendicular drift blocked** - Within half the exclusion radius, perpendicular movement is blocked
3. **Only escape allowed** - Within the exclusion zone, only moves that increase distance from the via are allowed

This prevents the centerline from drifting near its own via locations, which would cause the offset P/N tracks to intersect the offset P/N vias.

### Via Positions

P and N vias are placed perpendicular to the centerline direction at layer changes:

```python
# Average perpendicular direction from incoming and outgoing segments
perp_x = (-in_dir_y + -out_dir_y) / 2
perp_y = (in_dir_x + out_dir_x) / 2

# Via offset from the centerline (_pair_via_offset): at least the track
# half-pitch, half the minimum via-to-via distance, and far enough that the
# partner's track clears the via, all at the P-against-N clearance (#1218)
track_via_clearance = (pn_clearance + track_width / 2 + via_size / 2) * routing_clearance_margin
via_spacing = max(spacing_mm, min_via_spacing / 2, track_via_clearance - spacing_mm)

# Place vias perpendicular to centerline
p_via = (cx + perp_x * p_sign * via_spacing, ...)
n_via = (cx + perp_x * n_sign * via_spacing, ...)
```

### Approach and Exit Tracks

To keep the main P/N tracks parallel while routing to vias, approach and exit segments are added:

```
Main P track ────┐
                 │ approach
             [Via P]
                 │ exit
Main P track ────┘

Main N track ────┐
                 │ approach
             [Via N]
                 │ exit
Main N track ────┘
```

The approach/exit positions are calculated to:
1. Maintain track-via clearance from the inner via
2. Keep P and N tracks at constant diff pair spacing

With per-layer widths both are per layer: the approach runs at the incoming
layer's pitch and clears the via at that layer's width, the exit likewise on the
outgoing layer, and the via offset is large enough for every routable layer's
tracks to clear the partner via at their own pitch.

```python
# Determine which via is "inner" (closer to turn direction)
cross = in_dir_x * out_dir_y - in_dir_y * out_dir_x
inner_is_p = (p_sign > 0) if cross >= 0 else (p_sign < 0)

# Position outer track with clearance from inner via
outer_approach = inner_via + perpendicular * track_via_clearance
```

### GND Via Placement

By default, the router places GND vias adjacent to each differential pair signal via. This provides a return current path and improves signal integrity.

```
         GND via
            ○
            │
    P via ○─┼─○ N via    (signal vias)
            │
            ○
         GND via
```

**How it works:**

1. **Rust router checks clearance** - During A* search, when evaluating via positions, the router also checks if GND via positions are clear
2. **Ahead or behind** - GND vias can be placed ahead of or behind the signal vias along the route direction. The router tries both and picks the clear option
3. **Direction stored** - The chosen direction (1=ahead, -1=behind) is stored per layer change
4. **Python places vias** - After routing, Python uses the direction info to place GND vias connected to the GND net
5. **Each via clears its own pair** - The router checks a site on the grid, at the floored offsets, and lets the pair turn soon after the via; the via lands at the exact (float) offset, where the pair's own tracks as drawn -- a turn toward the via, the smoothed path -- can pass inside clearance. The via is moved straight outward up to 0.25 mm until it clears the pair's new tracks and vias and the pair's copper already on the board (stubs, pads) (#1207). Where nothing outward clears (the pair wraps round the site), the site mirrored across the signal via -- the router's other candidate, as close to the signal via -- is tried. A moved or mirrored site must also clear every other net's copper. A via neither site clears is **not placed** rather than shipped grazing: a console line `GND return via at (x,y) not placed: no site clears the pair's own copper ...` names it and the copper it would touch, and the run totals them in `JSON_SUMMARY.gnd_vias_not_placed` (also in the GUI's completion dialog).
6. **The final site is drill-checked** (#1341) - That final site is checked against every drill at hole-to-hole: every via (any net, so another pair's GND via too), the pair's own signal vias, every pad hole, and the GND vias already placed. A site that fails is not placed, with a console line `GND return via at (x,y) not placed: ...` naming the blocking drill (and a `drill` record in `JSON_SUMMARY.gnd_vias_not_placed`); usually another pair's GND via is already there to carry the return current.

**GND via positions** (`_gnd_via_offsets()`):
- Perpendicular offset: `P/N spacing + track_width/2 + clearance + via_size/2`
- Along-heading offset: `via_size + clearance` (ahead or behind signal vias)

The clearance is the one `check_drc` grades GND against the pair at (the pair's net class or `.kicad_dru` value, #1207), not just the run's base clearance. After the tracks are drawn, a GND via that sits too close to them is moved outward by up to 0.25 mm, and only to a site clear of every other net's copper.

Use `--no-gnd-vias` to disable this feature. GND vias are recommended for all high-speed
differential pairs (LVDS, USB, Ethernet, PCIe). Use the `/find-high-speed-nets` skill to
identify which pairs on your board are high-speed if uncertain.

## Connectors

Simple straight connectors link the original stub endpoints to the corresponding P/N track start/end points.

## Inner-Layer Launch via Escape Vias

The setback search (above) launches the coupled pair from the **stub's own
layer**. When that layer's launch corridor is jammed — e.g. a QFN/BGA fanout
whose escape stubs face *into* a dense pad field on `F.Cu` — but the terminal
sits on a **through-via or through-hole pad**, the pair can launch on any
routing layer that barrel spans instead (issue #195). The setback search retries
on the via-reachable inner/back layers and prefers a clean launch there:

```
source: F.Cu corridor jammed - launching on In1.Cu (reachable through the endpoint via)
```

A via is associated with a terminal when it sits within ~a via-radius of the
pad/stub tip (`_launch_assoc_tol`), so a hand- or auto-placed escape via that is
slightly offset still counts. This makes under-pad fanout (`bga_fanout` /
`qfn_fanout --escape-method underpad`) compose with coupled routing: the fanout
drops the escape vias, and the diff route picks the pair up on the open inner
layer.

When a launch can be found only by rotating the escape direction (the stub faces
away from the route), the search also tries launching **toward the other
terminal** and keeps a clean launch there if one exists, so the centerline heads
down the open corridor instead of U-turning off the stub.

## Hybrid Escape (Direct Coupled Middle + Point-to-Point Legs)

Some terminal geometries have **no clean connector join** for the coupled
centerline at all: the straight P/N connectors graze the partner's escape via
(issue #165), or the pair must **swap sides** between the two ends (P above N at
one terminal, below at the other), or no valid setback exists in any direction.
The normal centerline+connector pipeline fails these.

As a **last resort** — only after rip-up and the polarity/flip retries are
exhausted — the router falls back to a hybrid that decouples the coupled middle
from the terminal escapes (`_route_direct_coupled_middle`):

1. **Direct coupled middle.** Route the centerline *straight* from the source
   via-midpoint to the target via-midpoint on the best open layer — no
   escape-direction setback, no connectors, no polarity stage. Candidate layers
   are all routing layers (preferring those an existing via barrel spans, then
   inner, then `F.Cu`/`B.Cu`). Offset the centerline into a clean parallel P/N
   pair. The pair runs the open corridor as close to each terminal as it can.

2. **Polarity by the pads' sides.** There is no coupled polarity stage. Each end
   votes for the offset side its P pad lies on, relative to the local centerline
   direction. A genuine side-swap still costs one crossing leg either way, so the
   hybrid is then assembled with **both** polarities and the cleaner one kept
   (fewest P/N overlaps, then fewest vias, then shortest copper; #248).

3. **Point-to-point legs.** Attach each terminal to its middle near-end with a
   short single-ended A* leg (`_route_hybrid_leg`), trying leg orders in turn
   (P's legs first, then N's first, then both source legs before either target
   leg; #244) so a first leg cannot box out its partner's in a dense terminal
   field. The second leg at each end is attracted to its partner's just-routed
   leg, and the first keeps room beside itself for it, so the two escape
   together (`KICAD_HYBRID_COUPLE=0` disables this). The leg starts on the
   terminal's own through-via (any layer) or the bare pad's layer and **drops its
   own escape via** for the layer change to the middle. The **partner net's
   copper** — its middle, its terminal vias, the first net's just-routed legs, and
   the partner's **original fanout stub** (excluded from the diff-pair obstacle
   map, so re-added here) — is added as an obstacle, so a side-swap leg routes
   *around* it. That is where polarity is resolved: at the pads, by independent A*,
   with no coupled crossing.

   **Boxed-endpoint handling.** A bare-pad stub tip can sit in a foreign keep-out
   tight enough that the leg can't get a via in *at the tip* — but a via still
   fits a short walk away. This used to need a local escape (`apply_endpoint_escape`),
   because the obstacle map's approximations near a diagonal stub were wrong: the
   conservative **square** track stamp blocked the tip's Euclidean-legal neighbours
   (so the leg couldn't step off it), and the bresenham via-block **under-covered**
   the diagonal stub (so an escape via landed a hair too close). Both are now fixed
   *globally* by the **exact (capsule) obstacle keep-out** (`build_base` measures
   track and via clearance from the true float segment, matching the cache —
   issue #173). With exact geometry the boxed tip's clear cells are no longer
   falsely blocked and the escape via lands clean, so the special-case escape code
   was retired. This is what routes the stock *surface-fanned* watchy `USB_D` pair
   (boxed by the adjacent `USB_DET` stub, ~0.225 mm away) 1/1 with no hand-placed
   vias — issue #197.

   **The middle must carry the signal (#1303).** A net's own copper is no
   obstacle to its legs, so a leg that cannot reach the middle's near end
   directly can reach its FAR end and run back along it. Both terminals then meet at that via
   and the middle hangs off the signal path as a loop; the post-route cleanup
   correctly breaks the loop and removes the dangling middle (imx6ulz DQS0 shipped
   0 mm coupled with 12 mm of skew that way). A leg plan whose legs join a net's
   terminals without its middle is refused (`_middle_bypassed`), and the next
   plan, polarity or layer pair is tried (`hybrid F.Cu: rejected -- 4 leg plan(s)
   joined the terminals without the coupled middle` under `--debug-lines`).

4. **Seam re-ask (#444), never splitting the pair (#1303).** Each net's
   assembled copper (escape → middle → escape) is re-asked as ONE whole-connection
   A* against its partner's copper, and the pair of replacements is taken when
   it scores better: length + 2 per via + 3 per mm of copper farther than
   1 mm from the partner. A candidate that SPLITS the pair is rejected
   outright (`#444 seam re-ask rejected: it splits the pair (42% -> 0% of the
   route alongside its partner)`, `seam_splits_pair`): the share of its route
   that runs alongside the partner -- KiCad's coupling geometry, parallel
   facing spans on one layer at any distance, as KiCad counts them when the
   board sets no maximum gap -- has fallen below half the composed route's.
   On imx6ulz DQS0 and CLK0 the re-ask used to swap the middle for P and N on
   different layers (0 mm coupled) on length alone.

   The test is a share, not a length, and not at the routed gap. A re-ask
   exists to remove the composed route's detours, so a shorter route carries
   less coupled length by construction (ottercast EPHY_TX: a 44 mm composed
   leg against a 6 mm re-ask); and a re-ask is one single-ended A* per net,
   attracted to its partner, so it runs alongside at the routing clearance or
   wider, a few times the pair gap. How far apart it runs is the score's
   business (the split term). A gate at 1.5x the gap that refused any loss of
   coupled length rejected every re-ask on a coupled middle and shipped the
   composed detours instead: 13 corpus boards lost a net or more
   (esp32s3_m2_som 1 → 6: SPICLK's F.Cu middle swung across the corridor its
   GPIOs need, where the re-ask ran straight on B.Cu at a 0.3 mm gap), and
   fewer pairs ended within 0.1 mm of skew (112 → 96). The chain re-ask of a
   multipoint leg (`seam_reask_chain_leg`) carries the same gate. It runs on a
   leg already on the board, so its connectivity check leaves out the copper
   the candidate would replace: counting it let an EMPTY candidate "replace"
   daisy_pedal_board /USBCONN-D's leg, and the pair shipped disconnected.

The result is a fully-connected, DRC-clean pair: coupled where it matters (the
parallel run) and single-ended only on the last millimetre at each terminal,
where coupling buys nothing and flexibility is everything.

**Scope.** The hybrid needs each terminal to have *room* for inner-layer access —
a through-via/THT pad, or a bare pad with a clean via spot within a short walk of
the tip (the exact keep-out lets the leg reach it even when the tip itself is packed
against a neighbour stub). It does **not** rewrite placement: a terminal with **no
via-legal cell anywhere near** it (a neighbour stub hard against the pad on every
side) genuinely has nowhere to escape, and no amount of routing fixes that — fan
such pairs out **under-pad** (escape vias instead of surface stubs) so they are
not boxed, or open the obstruction in placement. The hybrid is gated by
`diff_pair_hybrid_escape` (default on) and returns the pair to the normal failure
path (single-ended follow-up) when it can't lay a clean route, so it never makes a
pair worse.

**How it is reported (#766).** A hybrid-routed pair is genuinely routed — its pads
connect and its copper is DRC-clean — so it keeps its place in `routed_diff_pairs`
and in `successful`, as long as the copper it ships still carries its coupled
middle (#1303, [below](#how-a-pair-is-graded-the-copper-it-ships-1303)). But its **terminal legs are point-to-point single-ended
copper**, which is not what `outcome: "coupled"` claims ("both members routed
coupled"), and the terminals are exactly where P/N geometry breaks and intra-pair
skew is born. The pair report therefore discloses it rather than reclassifying it:

```
"outcome": "coupled", "escape": "hybrid", "coupled_terminals": false
```

plus a `Hybrid escape: N pair(s) ...` line in the printed summary. **If you hold an
intra-pair skew budget, gate on `coupled_terminals`** — a `false` there is the
signal that the pair's terminals want a coupled fanout (see `bga_fanout.py
--escape-method underpad --diff-pairs`) rather than more routing effort.
Deliberately *not* demoted to `partial`: a partial pair's terminals are peeled to a
downstream single-ended pass, whereas a hybrid's are already routed, so demoting
would drop it out of the routed column and imply follow-up work that does not exist.

## How a Pair Is Graded: the Copper It Ships (#1303)

A pair's outcome used to come from its routing RESULT: both members had a
coupled-route result, so the pair was `coupled`. On imx6ulz that reported DQS0
`coupled` -- "coupled middle on F.Cu" -- while the copper written put P on In2.Cu
and N on B.Cu: **0 mm coupled, 4.5 mm of P/N skew** (the #444 seam re-ask had
swapped the coupled middle for two single-ended routes). Every claim is now read
off the board the run ships (`pcb_data` after the post-route cleanup, which the
`KICAD_BOARD_LEDGER=1` audit holds equal to the written file), engine-side, so
the GUI diff tab gets the same answer:

- **The measure** (`net_queries.diff_pair_coupling` / `diff_pair_skew`, see
  [the API](api-net-analysis.md)) is KiCad's: coupled length as KiCad 10's
  diff-pair coupling DRC computes it, in its integer geometry, and P/N skew as
  its skew DRC does, from lengths measured as the installed kicad-cli's release
  measures them (#1432). Both matched kicad-cli 10.0.0 to 0.1 µm.
- **Coupled** means the pair's coupled run -- at a gap no wider than 1.5x the
  gap the pair was routed at -- is at least one pitch (track width + gap) long
  AND covers at least 10% of the pair's length
  (`diff_pair_custody.measure_pair_copper`, `meets_coupled_floor`; the share is
  `COUPLED_SHARE_FLOOR`, #1429). The router lays its coupled run AT the gap (a
  45° run lands ~12 µm wider after grid snapping); its via approach tracks and a
  hybrid's single-ended legs sit 3-6x apart and do not count. The pair's length
  is the longer member's track length, `max(length_p_mm, length_n_mm)` -- the
  length KiCad's coupling test measures against (its `diff_pair_uncoupled`
  actual is that length minus the coupled run), so the share is
  `coupled / (coupled + uncoupled)`. The pitch rejects two legs that only pass
  each other at a corner (2 µm to 0.15 mm); the share rejects a coupled sliver
  on a long pair (0.5 mm of a 21.5 mm pair used to read `coupled`). A hybrid's
  coupled middle (1.4 mm of a 3 mm pair) passes both.
- **Every claimed pair carries its record** as `pair_reports[].copper`:
  `coupled_mm`, `uncoupled_mm`, `length_p_mm` / `length_n_mm`, `layers_p` /
  `layers_n`, `skew_mm`, the gap window, `coupled_floor_mm`, `coupled_share`
  (coupled run over the pair's length, capped at 1), `coupled_share_floor` and
  `coupled`.
- **A pair routed but not coupled** is outcome `uncoupled`, listed in
  `JSON_SUMMARY.uncoupled_diff_pairs` (and the GUI's completion dialog), and is
  **not** counted in `routed_diff_pairs` / `successful`. The end-of-run summary
  names it with what does ship:

  ```
  Not coupled:   1 pair(s) routed whose shipped copper is not coupled (#1303: under one pitch or 10% of the pair's length, #1429; not credited as routed; protected only while the members still run alongside, #1361): /DRAM/DRAM_DQS0 (0.000mm coupled of 21.081mm (0.0%); P on F.Cu/In2.Cu, N on B.Cu/F.Cu; skew 4.510mm; 0.000mm alongside within 1mm, not protected)
  ```

  A `partial` pair with no coupled copper is flagged `no_coupled_trunk` and
  moves to `partial_diff_pairs`, as before.
- **#521 protection follows the copper**: a pair whose copper is SPLIT is not
  recorded in `kicad_routing_tools.protected_nets` -- it is two single-ended
  routes a later step can redo, so freezing it would only take rip room away.
  Not coupled at the routed gap is not split (#1361). A pair that is not
  coupled also carries `alongside_mm`: KiCad's coupled length again, at any gap
  up to 1 mm (`diff_pair_custody.ALONGSIDE_GAP_MM`, the radius beyond which the
  seam re-ask's score counts copper as split), and its share `alongside_share`.
  When that run passes the same two floors as `coupled` -- one pitch and 10% of
  the pair's length (#1429) -- and both members end terminal to terminal, the
  pair stays protected and its report says `"protected": true`. The case is the seam re-ask: on ulx3s it
  routes GPDI straight from the AC caps to the HDMI pins, P and N 0.8 mm apart
  (the connector's pin pitch), with 0.17-0.40 mm of skew. Unprotected, the
  chain's next steps ripped GPDI_D1 and laid P on In2.Cu beside N on F.Cu.
  Split layers (DQS0) run alongside nowhere, and two members routed to
  separate connectors 20 mm apart are not alongside either, although KiCad's
  geometry with no maximum gap counts them. A pair with a member left open
  stays unprotected (#1107).
- **The intra-pair status** (`--diff-pair-intra-match`) is re-read off the
  shipped copper too; see [Intra-Pair P/N Length Matching](#intra-pair-pn-length-matching).

Graders see the same numbers: `check_drc.py` grades a board's `.kicad_dru`
`skew` and `diff_pair_uncoupled` rules (check 17 in
[utilities](utilities.md#checks-performed)), and `board_score.py`'s `diff_pairs`
component measures every pair KiCad recognises and counts those rule rows. With
no such rule the component is UNGRADED, as KiCad grades no pair without one; its
console line still names the largest skews and any pair with no coupled copper.

## Debug Visualization

With `--debug-lines`, debug geometry is output on User layers as graphic lines:

| Layer | Content |
|-------|---------|
| `User.3` | Connectors (stub to P/N track) |
| `User.4` | Stub direction arrows (1mm arrows from midpoint at src/tgt) |
| `User.5` | BGA exclusion-zone rectangles (inner zone + proximity outer) and stub/pad proximity circles |
| `User.6` | Boundary position labels |
| `User.7` | DRC violation lines (from `check_drc.py --debug-lines`) |
| `User.8` | Simplified centerline path |
| `User.9` | Raw A* centerline path |

This helps visualize the routing structure without affecting the actual routed copper layers.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `--diff-pair-gap` | board class, else 0.101 | Gap between P and N traces (mm). Omitted (the GUI: override unchecked), each pair takes its net class's `diff_pair_gap` and the call the Default class's; a class that leaves it unset inherits Default's, and a 0 is unset, falling back to the class clearance (the same rule makes an omitted `--track-width` the class `diff_pair_width`, else the class track width). Raised, never lowered, to the clearance (#441), the pair's net class (#530) and any `.kicad_dru` clearance rule that binds the pair on a layer it routes on (#1145): KiCad grades P against N as two nets. A pair a coupled-pair rule grades -- a clearance rule whose condition ANDs in `AB.isCoupledDiffPair()`, e.g. `AB.isCoupledDiffPair() && A.hasNetclass('USB')` -- keeps the gap asked for, floored at that rule alone (#1293). A "keep pairs apart" rule (`!AB.isCoupledDiffPair()`) never raises it: KiCad exempts a pair's own partner, by name (#1295) |
| `--track-width` | board class, else 0.3 | Each leg's width (mm). Omitted, the net class's `diff_pair_width` (see above). With `--impedance` it is the min-width floor and the no-stackup fallback |
| `--impedance` | - | Target differential impedance (ohms, e.g. 100): solves each layer's leg width from the board stackup at `--diff-pair-gap` |
| `--coplanar-gap` | - | Declares that every pair in the call runs through a same-layer ground pour this far (mm) from each trace's outer edge, so outer layers use the coplanar-waveguide model (requires `--impedance`; see [configuration.md](configuration.md#differential-pair-options-route_diffpy-only)) |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--max-setback-angle` | 45 | Half-angle of the setback angle scan (degrees) |
| `--routing-clearance-margin` | 1.0 | Multiplier on the track-to-via distance that sets the P/N via offset and the centerline's via keep-out (1.0 = minimum DRC) |
| `--diff-chamfer-extra` | 1.5 | Chamfer of pair-centreline meanders, as a multiple of the P/N half-pitch (>1 keeps P and N from crossing) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-turn-angle` | 180 | Max cumulative turn angle (degrees) to prevent U-turns |
| `--polarity-swap-nets` | (none = deny all) | Glob patterns naming the pairs ALLOWED to resolve a P/N mismatch by a pad-net swap (#279). `'*'` = all pairs; omit for no swaps ever |
| `--no-gnd-vias` | false | Disable GND via placement near signal vias |
| `--swappable-nets` | - | Glob patterns for diff pair nets that can have targets swapped |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--debug-lines` | false | Output debug geometry on User.3/4/5/6/8/9 layers |
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize routing (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty near stubs (mm equivalent) |
| `--bga-proximity-radius` | 7.0 | Radius around BGA edges to penalize routing (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty near BGA edges (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize (mm, same layer) |
| `--track-proximity-cost` | 0.0 | Cost penalty near routed tracks (0 = disabled) |
| `--mps-reverse-rounds` | false | Route most-conflicting MPS groups first (instead of least) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair (meander shorter track) |
| `--ac-couple-match` | false | Match an AC-coupled pair end to end across its series caps (see [Length Matching](length-matching.md#differential-pairs)) |

## Track Proximity Avoidance

The `--track-proximity-distance` and `--track-proximity-cost` options penalize routes that run close to previously routed tracks on the same layer. This spreads routes apart, which can rescue a congested board, but across the corpus it ADDS DRC violations (#584: +119 to +244 at cost 2), so it is a retry lever: re-run with it, compare, and keep whichever grades better. Disabled by default (cost = 0).

**Note:** Track proximity works correctly for differential pair routing (pose-based A*).

## Target Swap Optimization

For swappable diff pairs (e.g., memory data lanes where any source can connect to any target), the router can optimize target assignments to minimize crossings.

### Usage

```bash
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*rx*" \
    --swappable-nets "*rx*"
```

### How It Works

1. **Chip boundary detection** - Identifies source and target chips from pad positions
2. **Boundary position computation** - "Unrolls" each chip's boundary into a linear ordering
3. **Crossing detection** - Two routes cross if their source order is inverted relative to their target order
4. **Hungarian algorithm** - Finds optimal assignment minimizing total cost (distance + crossing penalty)
5. **Pad net swapping** - Swaps target pad net assignments in the PCB data before routing

The crossing penalty (default 1000) heavily discourages crossing assignments, prioritizing non-crossing routes even if they're slightly longer.

## Length Matching for Differential Pairs

Differential pairs support length matching with trombone-style meanders. This works for both single-layer and multi-layer routes:

- **Single-layer routes**: Meanders are added to straight sections of the centerline, then P/N paths are regenerated
- **Multi-layer routes**: Meanders are applied to same-layer straight sections, preserving via positions. GND vias are regenerated after meander application
- **Via barrel length**: Route length calculations include via barrel length (parsed from board stackup) for accurate length matching that matches KiCad's measurements
- **Pad-to-die length**: each leg also carries its pads' `(die_length ...)` (#1248), the term KiCad's skew tuner and skew DRC add, so inter- and intra-pair matching equalize the lengths KiCad compares. In a match group a pair is its larger leg, each leg its own copper plus its own end pads' term, as KiCad's pair tuner measures it, so unequal legs (no intra-match) are exact too; a multipoint pair counts only its matched leg's terminals. Copper alone matches KiCad only on a board that declares none
- **Stub via barrel length**: BGA pad vias in stubs are included using the actual stub-layer-to-pad-layer distance (not the full via span)

### Intra-Pair P/N Length Matching

Use `--diff-pair-intra-match` to match P and N track lengths within each differential pair. This is useful when P and N have different lengths due to:
- Connector regions near pads
- Curves (inner vs outer radius)
- Different via positions

When enabled:
1. After routing and inter-pair length matching, the router measures P and N
   **on the board's copper** (#1303, `net_queries.net_signal_length`): the
   pad-to-pad copper path of a 2-pad member, the copper of a larger one with
   every dead-end branch trimmed, plus pad-to-die -- what the post-route cleanup
   will leave of it. The routing result alone is not that copper: a hybrid
   escape or seam re-ask returns no stub lengths, and on imx6ulz DQS0 the result
   read P 12.00 < N 13.08 while the board had P 21.31 > N 16.82. A member whose
   pads are not joined yet falls back to the result's own sums (segments plus
   recorded stub lengths), as before
2. If the difference exceeds `--length-match-tolerance` (default 0.1mm), meanders are added to the shorter track
3. The meanders are placed with clearance checking against the other track of the pair
4. For polarity-swapped pairs, stub lengths are correctly recalculated using post-swap assignments (P gets P_source + N_target stubs, N gets N_source + P_target stubs) -- in the fallback; the board copper already carries the swap
5. A multipoint pair's meander lands in the **leg** that held the copper it
   replaced (`length_matching._splice_into_legs`), as group matching's does: the
   writer emits the per-leg dicts, so a meander written only to the merged
   bookkeeping dict shipped beside the leg's original straight run, which
   shorted the bump out (imx6ulz CLK0; `KICAD_BOARD_LEDGER=1` reported the net
   diverged)
6. **The verdict is read off the shipped copper** (#1303). The board the matcher
   measures (step 1) is the one BEFORE the post-route cleanup, so what ships can
   still differ by what the cleanup trims or collapses (imx6ulz CLK0: the matcher
   reached 0.001 mm, the board shipped 0.214 mm).
   Once the board is final, `length_matching.regrade_intra_pair_reports` sets each
   `intra_pair_matching` record's `delta_mm` to the shipped skew (KiCad's
   measure, `net_queries.diff_pair_skew`) and its `status` from it: `matched` /
   `within-tolerance` inside `--length-match-tolerance`, `unmatched` over it
   (`reason` keeps the matcher's own, or `copper-skew-over-tolerance` when the
   matcher believed it matched, or `not-measured` when a member's pads are not
   joined). `matcher_status` / `matcher_delta_mm` keep what the matcher reported;
   `delta_before_mm` stays the matcher's estimate. The `Intra-pair (shipped
   copper)` console line prints the regraded verdict

```bash
# Enable intra-pair matching
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" \
    --diff-pair-intra-match

# Combine with inter-pair matching
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" "*CK*" \
    --length-match-group auto \
    --diff-pair-intra-match
```

**Execution order**: Inter-pair matching runs first (on centerline), then intra-pair matching adds meanders to individual P/N tracks. This order is intentional - inter-pair regenerates P/N from the meandered centerline, so intra-pair must run last to preserve its meanders.

### Time Matching

Use `--time-matching` as an alternative to length matching when routes traverse layers with different dielectric properties. Time matching accounts for the different signal propagation speeds:

- **Outer layers (microstrip)**: ~5.4 ps/mm for typical FR4
- **Inner layers (stripline)**: ~6.9 ps/mm for typical FR4

This is useful when differential pairs in a group are routed on different layers. Time matching ensures equal propagation delay rather than equal physical length.

```bash
# Time match differential pairs
python3 py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*DQS*" \
    --length-match-group auto \
    --time-matching \
    --time-match-tolerance 1.0
```

## Limitations

1. **Polarity swap** - Off by default; allow it per pair with `--polarity-swap-nets <patterns>` (`'*'` = every pair; the GUI's "Polarity-swap allowed nets" field, empty by default). Only allow pairs an endpoint can compensate (FPGA generic I/O, polarity-tolerant SerDes) - never USB/MIPI/TMDS (#279). For a denied pair, a mismatch is re-routed with the connectors out the opposite side at one end (bare-pad endpoints only), and skipped with a warning if that is not possible; denied-but-wanted swaps are listed in `polarity_swap_denied_pairs` in the JSON summary
2. **Fixed spacing** - Spacing is constant along each layer of the route (no tapering); with per-layer widths each layer keeps its own pitch
3. **Grid snapping** - Centerline endpoints snap to grid for the search; the
   terminal endpoints of the generated geometry are un-snapped back to the
   exact setback positions so the connector fan stays centered between the
   pads (snapping could bias it half a grid cell toward one pad and graze it)
4. **Multi-point pairs** - Routed as a chain of terminals; mid-track taps are
   geometrically impossible for a pair, and each terminal supports at most
   two legs (one per side of its escape axis)

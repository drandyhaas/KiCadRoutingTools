# Configuration Reference

This document describes all configuration options for the KiCad Grid Router.

## Command-Line Options

### Basic Usage

```bash
# Single-ended routing (all nets by default)
python route.py input.kicad_pcb [output.kicad_pcb] [OPTIONS]  # Default output: input_routed.kicad_pcb
python route.py input.kicad_pcb --overwrite [OPTIONS]         # Overwrite input file

# Differential pair routing (all nets by default)
python route_diff.py input.kicad_pcb [output.kicad_pcb] [OPTIONS]  # Default output: input_routed.kicad_pcb
python route_diff.py input.kicad_pcb --overwrite [OPTIONS]         # Overwrite input file
```

Use `route.py` for single-ended nets and `route_diff.py` for differential pairs. By default, all nets are routed. Use `--nets` to filter specific patterns.

`route.py` always writes the output file, even when nothing routes (no valid nets, or all already connected) — it writes an unchanged copy of the input in that case, so output→input pipelines don't break on a missing file.

Pads with no pad number (paste/thermal-via artifacts KiCad doesn't netlist individually) are not used as routing targets; they remain copper obstacles.

### Net Selection

Net names support glob wildcards. Use `--nets` (or `-n`) to specify patterns:

```bash
# Exact net names
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_0)" "Net-(U2A-DATA_1)"

# Wildcard patterns
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(U2A-DATA_*)"

# Multiple patterns
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(*CLK*)" "Net-(*DATA*)"

# Route all nets on a component (auto-excludes GND/VCC/VDD/unconnected)
python route.py in.kicad_pcb out.kicad_pcb --component U1
```

### Ripping Pre-Existing Routes

| Option | Default | Description |
|--------|---------|-------------|
| `--rip-existing-nets` | off (untouched) | Net name patterns of **pre-existing** routed nets that may be ripped up and re-routed when they block a net being routed |

By default the router **never** rips committed tracks that were already on the
input board — only nets it routed *in this run* are candidates for rip-up (see
[Rip-Up and Reroute](rip-up-reroute.md)). `--rip-existing-nets PATTERN` lifts
that restriction for the matching pre-existing routed nets, so the router may
tear them up and re-route them when they block a net it is trying to route (for
example on a board already routed by a previous run). Use `'*'` to allow any
non-plane net.

```bash
# Let the router rip and re-route any pre-existing DATA net that gets in the way
python route.py in.kicad_pcb out.kicad_pcb --nets "*CLK*" --rip-existing-nets "*DATA*"

# Allow ripping any pre-existing (non-plane) net
python route.py in.kicad_pcb out.kicad_pcb --nets "*" --rip-existing-nets "*"
```

### Geometry Options

| Option | Default | Description |
|--------|---------|-------------|
| `--track-width` | 0.3 | Track width in mm (ignored if `--impedance` specified) |
| `--impedance` | - | Target single-ended impedance in ohms (calculates width per layer from stackup) |
| `--clearance` | 0.25 | Track-to-track clearance in mm |
| `--via-size` | 0.5 | Via outer diameter in mm |
| `--via-drill` | 0.3 | Via drill diameter in mm |
| `--grid-step` | 0.1 | Grid resolution in mm |

**Impedance-controlled routing:** When `--impedance` is specified, track widths are automatically calculated per layer using IPC-2141 formulas based on the board stackup. Outer layers use microstrip formulas (typically wider tracks) and inner layers use stripline formulas (typically narrower tracks). Via clearance calculations account for the varying track widths per layer.

### Fab Tier Options

The **fab tier** is the JLCPCB manufacturing floor every routing step shrinks tracks,
vias and clearances *down toward* when it needs to. It is shared by every CLI
(`route.py`, `route_diff.py`, `route_planes.py`, `route_disconnected_planes.py`,
`bga_fanout.py`, `qfn_fanout.py`, `check_drc.py`, `fix_kicad_drc_settings.py`,
`list_nets.py`) and the GUI (one selector on the Basic tab). Values are sourced from
[jlcpcb.com/capabilities](https://jlcpcb.com/capabilities).

| Option | Default | Description |
|--------|---------|-------------|
| `--fab-tier` | `standard` | `standard` (no extra fab cost) or `advanced` (tighter, "more costly") |
| `--fab-overrides` | - | Path to a file overlaying the tier's floors (only the keys it lists) |

The tier is a **floor ladder**:

- **`standard`** — the cheap floor (per layer count: 2-layer track/clearance 0.127/0.127,
  4+ layer 0.0889/0.10; via 0.45 / drill 0.20; via hole-to-hole 0.20). Routing prefers
  it but **auto-escalates to the `advanced` floor — printing a one-line warning** — when a
  fine-pitch fan-out genuinely cannot escape at the standard floor.
- **`advanced`** — the JLC "more costly" floor (track/clearance 0.10/0.10 on 2-layer,
  0.0762/0.09 on 4+; via 0.25 / drill 0.15). A **hard** floor: no escalation.

| Floor (per layer count) | standard | advanced |
|---|---|---|
| via diameter / drill | 0.45 / 0.20 | 0.25 / 0.15 |
| track / clearance (2-layer) | 0.127 / 0.127 | 0.10 / 0.10 |
| track / clearance (4+ layer) | 0.0889 / 0.10 | 0.0762 / 0.09 |
| via hole-to-hole / pad hole-to-hole | 0.20 / 0.45 | 0.20 / 0.45 |

**Override file** (`--fab-overrides`) — a plain, human-editable file overlaying the
selected tier. Only the floor values it lists change; the rest come from the base tier.
Supplying one **disables escalation** (the floor becomes exactly *base tier + file*),
since the file states your exact fab limits. One `key = value` (or `key: value`, or
`key value`) per line; `#` starts a comment; keys are `track_width`, `clearance`,
`via_diameter`, `via_drill`, `hole_to_hole`, `pad_hole_to_hole`, `annular`:

```
# my_fab.txt — JLC + a tighter drill I've confirmed
via_drill = 0.15
clearance = 0.09
```

A ready-to-copy, fully-commented template listing every key and the built-in tier
values lives at **[`fab_overrides.example.txt`](../fab_overrides.example.txt)** in the
repo root. (`track_width` / `clearance` are layer-dependent in the tier tables, but an
override sets one fixed value for every board — override them only if you want that.)

```bash
# Route to the cheap floor (default); dense fan-outs warn when they escalate
python route.py in.kicad_pcb out.kicad_pcb --nets "Net*"

# Opt the whole board into the tighter, more-costly floor
python route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-tier advanced

# Declare your own fab capability
python route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-overrides my_fab.txt
```

**Floor enforcement.** The CLI **errors** if `--track-width`, `--clearance`,
`--via-size`, `--via-drill` or `--hole-to-hole-clearance` is set below the active tier's
floor (raise the value, or declare a smaller capability with `--fab-overrides`). The GUI
instead **pins** the corresponding Basic-tab spin control to the floor and warns. Grade
verification (`check_drc.py`) defaults its size/clearance floors to the same tier, so
legitimately-escalated fine geometry is not flagged.

### Post-Route DRC Settings

As their final step, all four routing CLIs (`route.py`, `route_diff.py`,
`route_planes.py`, `route_disconnected_planes.py`) rewrite the output's sibling
`.kicad_pro` so KiCad's Board Setup floors match the clearances/sizes just routed
— a manual DRC in KiCad then flags only genuine problems instead of stock-default
noise (issue #160). The [DRC Settings Fixer](utilities.md#drc-settings-fixer-fix_kicad_drc_settingspy)
does the work; these flags tune it. The GUI plugin applies the equivalent on the
live board via the pcbnew API.

| Option | Default | Description |
|--------|---------|-------------|
| `--no-fix-drc-settings` | off (fix is on) | Do **not** adjust the output's `.kicad_pro` DRC constraints afterwards; leave KiCad's stock floors |
| `--keep-thermal` | off | Leave `starved_thermal` (thermal-relief) severity untouched instead of demoting it to a warning |
| `--no-clamp-netclasses` | off (clamp is on) | Do **not** cap/clamp non-Default net classes down to `--clearance` — build the class map but preserve the full spec (routing honors it, writeback keeps it). By **default** each non-Default class is routed and graded at `min(class, --clearance)` and the output `.kicad_pro` clamps it to the routed floor (#439 — stock classes are largely *aspirational*; the human-routed references violate their own class, so keeping them manufactures phantom sub-class DRC). Pass this only for a **final** impedance-controlled board whose net-class rules *are* the spec and are actually met |
| `--enable-used-layers` | off | Add any layer the board uses but is missing from its `(layers)` table back into the `.kicad_pcb`, so KiCad stops flagging `item_on_disabled_layer`. Off by default because it edits the board, not just DRC settings |

### Power Net Options

| Option | Default | Description |
|--------|---------|-------------|
| `--power-nets` | - | Glob patterns for power nets (e.g., `"*GND*" "*VCC*"`) |
| `--power-nets-widths` | - | Track widths in mm for each power-net pattern (must match `--power-nets` length) |
| `--neckdown-length` | 2.5 | Narrow track length in mm from the pads when a failed wide power route is retried at the default width (neck-down) |
| `--neckdown-taper-length` | 0.5 | Narrow-to-wide width taper length in mm (0 = abrupt) |
| `--no-power-tap-neckdown` | off | Disable the neck-down retry of failed wide power routes |

```bash
# Route with wider tracks for power nets
python route.py input.kicad_pcb output.kicad_pcb --nets "Net*" \
  --power-nets "*GND*" "*VCC*" "+3.3V" \
  --power-nets-widths 0.4 0.5 0.3
  # default track-width is 0.3 for non-power nets
```

Patterns are matched in order - first match determines width. Obstacle clearances automatically adjust for wider power traces.

See [Power Net Analysis](power-nets.md) for automatic detection, AI-powered analysis, and IPC-2152 track width guidelines.

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--via-cost` | 50 | Via penalty in 0.1mm grid steps, i.e. 50 = 5mm of path; mm-equivalent at any `--grid-step` (effectively doubled for diff pairs since two vias are placed) |
| `--max-iterations` | 200000 | A* iteration limit per route |
| `--max-probe-iterations` | 5000 | Quick probe per direction to detect stuck routes |
| `--heuristic-weight` | 1.9 | A* greediness (>1 = faster, <1 = more optimal) |
| `--turn-cost` | 1000 | Penalty for direction changes (encourages straighter paths) |
| `--max-ripup` | 3 | Max blockers to rip up at once during rip-up and retry |
| `--ripup-abandon-metric` | `stranded` | Keep-retry vs abandon rule for multipoint tap rip-ups (see [rip-up-reroute.md](rip-up-reroute.md#abandon-metrics)) |
| `--routing-clearance-margin` | 1.0 | Multiplier on track-via clearance (1.0 = minimum DRC) |
| `--hole-to-hole-clearance` | 0.20 | Minimum drill hole edge-to-edge clearance (mm) |
| `--board-edge-clearance` | 0.0 | Clearance from board edge in mm (0 = use track clearance) |
| `--proximity-heuristic-factor` | 0.02 | Factor for proximity-aware A* heuristic (higher = faster but may find suboptimal paths, 0 = disabled) |
| `--ripped-route-avoidance-radius` | 1.0 | Radius around ripped route corridors to apply soft penalty (mm) |
| `--ripped-route-avoidance-cost` | 0.1 | Cost penalty for routing through ripped corridors (0 = disabled) |

See [Rip-Up and Reroute](rip-up-reroute.md) for how failed routes trigger rip-up, how blockers are identified and ranked, and how ripped nets are rerouted.

### Routing Strategy Options

| Option | Default | Description |
|--------|---------|-------------|
| `--ordering` / `-o` | mps | Net ordering: `mps`, `inside_out`, or `original` |
| `--direction` / `-d` | forward | Direction: `forward` or `backward` |
| `--layers` / `-l` | all copper layers | Routing layers. For `route.py` the default is all of the board's copper layers; `route_diff.py` and `bga_fanout.py` default to `F.Cu B.Cu` |
| `--layer-costs` | (see below) | Per-layer cost multipliers (1.0-1000). Default: all 1.0 for 4+ layers; F.Cu=1.0, B.Cu=3.0 for 2 layers |
| `--no-bga-zones [REFS...]` | (auto-detect) | Disable BGA exclusion zones. No args = all. With refs (U1 U3) = only those |

### Proximity Penalty Options

| Option | Default | Description |
|--------|---------|-------------|
| `--stub-proximity-radius` | 2.0 | Radius around stubs to penalize (mm) |
| `--stub-proximity-cost` | 0.2 | Cost penalty at stub center (mm equivalent) |
| `--via-proximity-cost` | 10.0 | Via cost multiplier in stub proximity zones (0 = block vias) |
| `--bga-proximity-radius` | 7.0 | Radius around BGA edges to penalize (mm) |
| `--bga-proximity-cost` | 0.2 | Cost penalty at BGA edge (mm equivalent) |
| `--track-proximity-distance` | 2.0 | Radius around routed tracks to penalize on same layer (mm) |
| `--track-proximity-cost` | 0.0 | Cost penalty near routed tracks (0 = disabled) |
| `--vertical-attraction-radius` | 1.0 | Radius for cross-layer track attraction (mm) |
| `--vertical-attraction-cost` | 0.0 | Cost bonus for aligning with tracks on other layers (0 = disabled) |

### Bus Routing Options

Detects groups of nets with clustered endpoints (buses) and routes them as parallel tracks, each net attracted to its already-routed neighbor. See [Bus Routing](bus-routing.md) for the detection algorithm, middle-out routing order, and tuning guidance.

| Option | Default | Description |
|--------|---------|-------------|
| `--bus` | off | Enable bus detection and routing |
| `--bus-detection-radius` | 5.0 | Max endpoint distance for nets to form a bus (mm) |
| `--bus-min-nets` | 2 | Minimum number of nets to form a bus group |
| `--bus-attraction-radius` | 5.0 | Attraction radius around the neighbor's track (mm) |
| `--bus-attraction-bonus` | 5000 | Cost bonus for moving parallel to the neighbor's track |

### Guide Corridor Options (preferred route)

Draw a polyline on a User layer in KiCad, then enable `--guide-corridor` to route the
selected nets so they **follow** that line. The router takes the polyline's vertices as
waypoints (`--guide-corridor-spacing > 0` subdivides long segments for tighter curves) and
routes source → waypoints → target. It keeps the route on a single layer where it can — a
waypoint it can't reach cleanly on the current layer is skipped rather than forcing a layer
change, so a corridor doesn't add vias the direct route wouldn't need. It applies to all nets
routed in the run and works in either draw direction; multiple nets following the same
corridor pack alongside each other without overlapping. Supported by `route.py` and the
plugin ("Follow User-layer guide path" checkbox); not currently supported by `route_diff.py`
or `route_planes.py`.

| Option | Default | Description |
|--------|---------|-------------|
| `--guide-corridor` | off | Route selected nets to follow a user-drawn guide polyline |
| `--guide-corridor-layer` | User.1 | User layer the guide polyline is drawn on |
| `--guide-corridor-spacing` | 0.0 | Max mm between waypoints. `0` uses only the drawn segment endpoints (vertices); `>0` subdivides long segments to follow curves more tightly |

```bash
# Route two nets so they follow a corridor drawn on User.1
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(A)" "Net-(B)" --guide-corridor
```

Notes:
- **Endpoints/topology are untouched.** The waypoints only steer the path *between* the
  pads the router already chose to connect. For a multi-point net the existing MST is kept,
  and each waypoint steers the MST segment it is nearest to (so the net follows the whole
  drawn line across its topology); a waypoint used by one segment is not reused by others.
- **A corridor never makes a route fail.** It is strictly best-effort: a waypoint that
  can't be reached (or that would strand the route) is dropped, and if no waypoints can be
  followed the segment routes directly — identical to routing with no corridor. With the
  flag off, routing is byte-for-byte unchanged.
- Routes are placed one at a time; each becomes an obstacle for the next, which keeps
  multiple corridor nets from overlapping. Draw the corridor wide enough (or use separate
  lines) if you want several nets to follow it comfortably.
- The guide applies to every net routed in the run, so select only the nets you want it to
  steer. Nets routed as a detected bus (`--bus`) follow their neighbor instead of the guide.
- The route is built by routing A* legs between the waypoints and concatenating them. A
  multi-point tap or a detour can cross an earlier leg of the **same** net. KiCad allows
  same-net copper to overlap, so the net stays connected and DRC-clean against other nets,
  but the trace may be less tidy than a hand-route.
- Tested by `tests/test_guide_corridor.py` — follow, blocked-waypoint snap, shared-corridor
  non-overlap, flag-off regression, never-blocks-routing, spacing subdivision, and a real
  bundled board (`kicad_files/lvds_converter_dualclk.kicad_pcb`) with a `User.1` guide.

### Keepout Zone Options

Draw one or more closed polygons (`gr_poly`) or rectangles (`gr_rect`) on a User layer in KiCad,
then enable `--keepout` to keep routed tracks **out** of those areas. This is a **hard** keepout:
routes (and vias) cannot enter any of the polygons on any copper layer. It applies to every net
routed in the run.

| Option | Default | Description |
|--------|---------|-------------|
| `--keepout` | off | Keep routed tracks out of polygons drawn on a User layer |
| `--keepout-layer` | User.2 | User layer the keepout polygons are drawn on |

```bash
# Route a net so it avoids keepout polygons drawn on User.2
python route.py in.kicad_pcb out.kicad_pcb --nets "Net-(A)" --keepout
```

Notes:
- All closed polygons (`gr_poly`) and rectangles (`gr_rect`) found on the keepout layer are
  blocked — draw as many as you need. Use the polygon or rectangle tool, not the line tool: open
  lines (`gr_line`) don't bound an area and are ignored.
- The default keepout layer is `User.2`, distinct from the guide-corridor layer (`User.1`), so
  you can use both features at once.
- **Coverage:** keepouts are respected across single-ended, multipoint, and **differential-pair**
  routing (`route_diff.py --keepout`), and across plane via-stitching and plane repair. KiCad
  native keep-out rule areas (`(zone … (keepout …))`) are honoured automatically everywhere too,
  with no flag. (Guide corridors, by contrast, steer single-ended/multipoint routing only.)
- It's a non-copper User layer, so the polygon never affects manufacturing — it only steers the
  router.
- **Don't draw a zone over a pad you need to route.** Cells inside the polygon are blocked
  unconditionally, so a zone covering a routed net's pad can make that net unroutable. Zones are
  meant for open board area.
- Unlike a guide corridor (which is best-effort), a keepout is absolute — if a zone walls off the
  only path to a pad, that net will fail to route. With the flag off, routing is unchanged.
- **Plugin only:** the Basic tab has optional "Clear guide layer after routing" / "Clear keepout
  layer after routing" checkboxes (unchecked by default). When ticked, a *successful* route deletes
  the drawn guide/keepout graphics from that User layer so you can draw fresh ones — it only acts
  for the feature it pairs with, and never runs if nothing routed.
- Tested by `tests/test_keepout.py` — flag-off regression, single-net detour, multi-net avoidance,
  and a real bundled board (`kicad_files/lvds_converter_dualclk.kicad_pcb`).

### Differential Pair Options (route_diff.py only)

These options are only available in `route_diff.py`. All nets passed to route_diff.py are treated as differential pairs.

| Option | Default | Description |
|--------|---------|-------------|
| `--impedance` | - | Target differential impedance in ohms (calculates width per layer from stackup) |
| `--diff-pair-gap` | 0.101 | Gap between P and N traces (mm), also used as spacing for impedance calculation |
| `--diff-pair-centerline-setback` | 2x P-N dist | Distance in front of stubs to start centerline (mm) |
| `--min-turning-radius` | 0.2 | Minimum turning radius for pose-based routing (mm) |
| `--max-turn-angle` | 180 | Max cumulative turn angle (degrees) to prevent U-turns |
| `--max-setback-angle` | 45.0 | Maximum angle for setback position search (degrees) |
| `--polarity-swap-nets` | (none = deny all) | Glob patterns naming pairs allowed to polarity-swap; `'*'` = all (#279) |
| `--no-gnd-vias` | false | Disable GND via placement near signal vias |
| `--diff-chamfer-extra` | 1.5 | Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair |
| `--swappable-nets` | - | Glob patterns for diff pair nets that can have targets swapped |
| `--crossing-penalty` | 1000.0 | Penalty for crossing assignments in target swap optimization |
| `--mps-reverse-rounds` | false | Route most-conflicting MPS groups first (instead of least) |
| `--mps-segment-intersection` | auto | Force segment intersection method for MPS (auto-enabled when no nets on BGAs) |

### Layer Optimization Options

These options control stub layer switching, which moves stubs to different layers before routing to avoid vias. Works for differential pairs, single-ended nets, and multi-point nets (3+ endpoints).

| Option | Default | Description |
|--------|---------|-------------|
| `--no-stub-layer-swap` | false | Disable stub layer switching (enabled by default) |
| `--can-swap-to-top-layer` | false | Allow swapping stubs to F.Cu (off by default for diff pairs due to clearance) |

**How it works:**
- When source and target stubs are on different layers, a via is normally required
- Stub layer switching moves one stub to match the other's layer, eliminating the via
- For **swap pairs**, two nets exchange layers to help each other (e.g., Net1 src:A→B and Net2 src:B→A)
- For **solo switches**, a single stub moves when it doesn't conflict with other stubs
- For **bare-pad target swaps**, a diff-pair target that is a bare outer-layer pad
  (F.Cu/B.Cu) with no stub - e.g. a connector pin boxed in by neighbouring pads
  such as the front row of a 2-row header - is fanned out onto the pair's source
  layer: a through-via is dropped on each pad and a short stub grown on the source
  layer, so an inner-layer pair can land on the pad via the via instead of fighting
  through the congested surface channel
- Multiple swap options are tried: src/src, tgt/tgt, src/tgt, tgt/src
- For **multi-point nets** (3+ endpoints, #265), all movable stubs are collapsed onto one
  common layer when every endpoint allows it (through-hole and via-in-pad pads reach any
  layer; bare SMD pads fix their copper layers; already-connected pad-to-pad copper never
  moves), picking the destination layer that needs the fewest new pad vias. Nets with no
  feasible common layer are left unchanged

### Length Matching Options

Length matching adds trombone-style meanders to match route lengths within groups. Useful for DDR4 DQ/DQS signals.

| Option | Default | Description |
|--------|---------|-------------|
| `--length-match-group` | - | Group of nets to length match. Use `auto` for DDR4 auto-grouping, or specify patterns |
| `--length-match-tolerance` | 0.1 | Acceptable length variance within group (mm) |
| `--meander-amplitude` | 1.0 | Height of meanders perpendicular to trace (mm) |
| `--diff-chamfer-extra` | 1.5 | Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings) |
| `--diff-pair-intra-match` | false | Match P/N lengths within each diff pair (meander shorter track) |

**Auto-grouping for DDR4:**
```bash
--length-match-group auto
```
Groups nets by byte lane: DQ0-7 + DQS0, DQ8-15 + DQS1, etc.

**Manual grouping:**
```bash
--length-match-group "Net-(*DQ[0-7]*)" "Net-(*DQS0*)"
--length-match-group "Net-(*DQ[8-9]*)" "Net-(*DQ1[0-5]*)" "Net-(*DQS1*)"
```

**How it works:**
- Routes all nets first, then adds meanders to shorter routes
- Calculates pad-to-pad length including existing stub segments and via barrel lengths (parsed from board stackup)
- Stub via barrel lengths use actual stub-layer-to-pad-layer distance (not full via span for through-hole BGA pad vias)
- Via barrel length matches KiCad's length calculation for accurate matching
- Finds longest straight segment for meander insertion
- Iteratively adds meander bumps until length exceeds target, then scales down amplitude to hit exact target
- Per-bump clearance checking reduces amplitude to avoid conflicts with other traces
- Uses 45° chamfered corners for smooth trombone patterns
- Supports multi-layer routes: meanders are applied to same-layer sections, preserving via positions

See [Length Matching](length-matching.md) for the full algorithm: meander geometry, per-bump clearance checking, via barrel lengths, and DDR4 auto-grouping details.

### Time Matching Options

Time matching is an alternative to length matching that matches propagation delay instead of physical length. This accounts for different signal speeds on different layers:
- **Outer layers (microstrip)**: Faster propagation, effective dielectric = (Er + 1) / 2
- **Inner layers (stripline)**: Slower propagation, effective dielectric = Er

| Option | Default | Description |
|--------|---------|-------------|
| `--time-matching` | false | Match propagation time instead of length |
| `--time-match-tolerance` | 1.0 | Acceptable time variance within group (ps) |

**When to use time matching:**
- When routes in a group use different layers with different dielectric properties
- For high-speed signals where propagation delay matters more than physical length
- Typical FR4: outer layers ~5.4 ps/mm, inner layers ~6.9 ps/mm (~28% difference)

**Example:**
```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "*DQ*" \
    --length-match-group auto \
    --time-matching \
    --time-match-tolerance 1.0
```

See [Length Matching](length-matching.md#time-matching) for how propagation delay is computed from the stackup, including via barrel delays.

### Visualization Options

Available in `route.py` only (not `route_diff.py` or `route_planes.py`). See the [PyGame visualizer documentation](../pygame_visualizer/README.md) for the full list of interactive controls (pause/step, zoom/pan, layer filtering, speed). Requires pygame (`pip install pygame-ce`); without it, routing falls back to normal batch mode.

| Option | Default | Description |
|--------|---------|-------------|
| `--visualize` / `-V` | false | Show real-time visualization of routing (requires pygame) |
| `--auto` | false | Auto-advance to next net without waiting (with `--visualize`) |
| `--display-time` | 0.0 | Seconds to display completed route before advancing (with `--visualize --auto`) |

### Debug Options

| Option | Default | Description |
|--------|---------|-------------|
| `--debug-lines` | false | Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*) |
| `--verbose` / `-v` | false | Print detailed diagnostic output (setback checks, bus routing order, etc.) |
| `--skip-routing` | false | Skip actual routing, only do swaps and write debug info |
| `--debug-memory` | false | Print memory usage statistics at key points during routing |
| `--stats` | false | Print A* search statistics (cells expanded, heuristic efficiency) |
| `--add-teardrops` | false | Add teardrop settings to all pads in output file |

## GridRouteConfig Class

The `GridRouteConfig` dataclass holds all routing parameters:

```python
@dataclass
class GridRouteConfig:
    # Track geometry (see routing_defaults.py for values)
    track_width: float = 0.3      # mm (default for non-power nets)
    clearance: float = 0.25       # mm between tracks
    via_size: float = 0.5         # mm via outer diameter
    via_drill: float = 0.3        # mm via drill
    power_net_widths: Dict[int, float] = {}  # net_id -> width for power nets
    power_tap_neckdown: bool = True   # retry failed wide power routes at default width
    neckdown_length: float = 2.5      # mm of narrow track from the pads on necked routes
    neckdown_taper_length: float = 0.5  # mm narrow->wide width taper (0 = abrupt)

    # Grid
    grid_step: float = 0.1        # mm grid resolution

    # A* algorithm
    via_cost: int = 50            # via penalty in 0.1mm grid steps = 5mm of path (diff pairs place 2 vias)
    max_iterations: int = 200000
    max_probe_iterations: int = 5000  # quick probe per direction to detect stuck routes
    heuristic_weight: float = 1.9
    turn_cost: int = 1000         # penalty for direction changes (straighter paths)
    max_rip_up_count: int = 3     # max blockers to rip up at once (progressive N+1)
    ripup_abandon_metric: str = 'stranded'  # tap rip-up abandon rule (docs/rip-up-reroute.md)
    max_setback_angle: float = 45.0  # degrees
    routing_clearance_margin: float = 1.0  # multiplier on track-via clearance (1.0 = min DRC)
    hole_to_hole_clearance: float = 0.20  # mm - drill-to-drill fab floor
    board_edge_clearance: float = 0.0    # mm - clearance from board edge (0 = use clearance)
    proximity_heuristic_factor: float = 0.02  # factor for proximity-aware heuristic (0 = disabled)
    ripped_route_avoidance_radius: float = 1.0  # mm - radius around ripped route corridors
    ripped_route_avoidance_cost: float = 0.1    # cost for routing through ripped corridors

    # Layers
    layers: List[str] = ['F.Cu', 'B.Cu']
    layer_costs: List[float] = None  # per-layer cost multipliers (default: 1.0 for 4+, F.Cu=1.0/B.Cu=3.0 for 2)

    # BGA zones
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = []

    # Stub proximity
    stub_proximity_radius: float = 2.0   # mm
    stub_proximity_cost: float = 0.2     # mm equivalent
    via_proximity_cost: float = 10.0     # multiplier for vias near stubs

    # BGA proximity
    bga_proximity_radius: float = 7.0   # mm from BGA edges
    bga_proximity_cost: float = 0.2      # mm equivalent

    # Track proximity (same layer)
    track_proximity_distance: float = 2.0  # mm
    track_proximity_cost: float = 0.0      # mm equivalent (0 = disabled)

    # Vertical track alignment (cross-layer attraction)
    vertical_attraction_radius: float = 1.0  # mm
    vertical_attraction_cost: float = 0.0    # mm equivalent bonus

    # Direction
    direction_order: str = "forward"     # forward or backward

    # Differential pairs
    diff_pair_gap: float = 0.101         # mm between P and N
    diff_pair_centerline_setback: float = None  # mm in front of stubs (None = 2x P-N spacing)
    min_turning_radius: float = 0.2      # mm for pose-based routing
    max_turn_angle: float = 180.0        # degrees, prevents U-turns
    stub_layer_swap: bool = True         # enable stub layer switching optimization
    gnd_via_enabled: bool = True         # place GND vias near signal vias
    target_swap_crossing_penalty: float = 1000.0  # penalty for crossing assignments
    crossing_layer_check: bool = True    # only count crossings on same layer

    # Length matching
    length_match_groups: List[List[str]] = []  # groups of net patterns to match
    length_match_tolerance: float = 0.1        # mm - acceptable length variance
    meander_amplitude: float = 1.0             # mm - height of meander perpendicular to trace
    diff_chamfer_extra: float = 1.5            # chamfer multiplier for diff pair meanders
    diff_pair_intra_match: bool = False        # match P/N lengths within each diff pair

    # Time matching (alternative to length matching)
    time_matching: bool = False                # match propagation time instead of length
    time_match_tolerance: float = 1.0          # ps - acceptable time variance

    # Debug
    debug_lines: bool = False
    verbose: bool = False
    debug_memory: bool = False
```

## Parameter Guidelines

### Via Cost

The `via_cost` parameter controls how much the router penalizes layer changes:

| Value | Effect |
|-------|--------|
| 0-25 | Many vias, shorter paths |
| 50 (default) | Balanced, discourages unnecessary vias |
| 75-100 | Few vias, longer paths |

For BGA escape routing, lower values (10-25) work well since vias are necessary.

All cost knobs (via cost, proximity costs, attraction bonuses) are calibrated at a 0.1mm
reference grid and scale internally so the cost per mm of path is the same at any
`--grid-step` - changing the grid resolution does not change what the router optimizes for.

### Heuristic Weight

Controls A* behavior:

| Value | Effect |
|-------|--------|
| 1.0 | Optimal paths (slower) |
| 1.9 (default) | Good balance of speed and quality |
| 2.5+ | Faster but may miss tight routes |

### Grid Step

Smaller grid steps allow finer routing but increase computation:

| Value | Use Case |
|-------|----------|
| 0.05 | Fine-pitch BGAs, tight clearances |
| 0.1 (default) | General purpose |
| 0.2 | Fast routing, less detail |

### Layer Costs

The `--layer-costs` parameter controls per-layer routing preferences. Higher costs make the router avoid that layer:

| Value | Effect |
|-------|--------|
| 1.0 | No penalty, layer used freely |
| 3.0 | Moderately avoided (3x cost per grid step) |
| 10.0+ | Strongly avoided, used only when necessary |

**Defaults:**
- 4+ layers: all 1.0 (inner layers available for balanced routing)
- 2 layers: F.Cu=1.0, B.Cu=3.0 (prefer top layer for cleaner assembly)

**Example:** To keep signals on F.Cu and only use B.Cu when necessary:
```bash
--layers F.Cu B.Cu --layer-costs 1.0 5.0
```

### Stub Proximity

Penalizes routes that pass near unrouted stubs:

```
Cost = stub_proximity_cost * (1 - distance/stub_proximity_radius)
```

This encourages routes to avoid blocking future routing paths.

## Layer Configuration

### Standard 4-Layer Stack

```bash
--layers F.Cu In1.Cu In2.Cu B.Cu
```

### 2-Layer Board

```bash
--layers F.Cu B.Cu
```

### 6-Layer with Ground Planes

```bash
# Skip In1.Cu and In4.Cu (ground planes)
--layers F.Cu In2.Cu In3.Cu B.Cu
```

## Example Configurations

### BGA Fanout (Dense, Many Vias)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering inside_out \
    --via-cost 10 \
    --heuristic-weight 1.2 \
    --stub-proximity-radius 2.0 \
    --stub-proximity-cost 5.0
```

### Long Routes (Few Vias)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --ordering mps \
    --via-cost 50 \
    --heuristic-weight 2.0
```

### Differential Pairs (LVDS)

```bash
python route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*" \
    --diff-pair-gap 0.1 \
    --track-width 0.1 \
    --clearance 0.1 \
    --no-bga-zones
```

### Fast Routing (Large Boards)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.2 \
    --heuristic-weight 2.0 \
    --max-iterations 50000
```

### Fine-Pitch BGA

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "Net-(*)" \
    --grid-step 0.05 \
    --track-width 0.075 \
    --clearance 0.075 \
    --via-size 0.2 \
    --via-drill 0.1
```

### Power and Signal Mixed (Wider Power Tracks)

```bash
python route.py input.kicad_pcb output.kicad_pcb --nets "*" \
    --power-nets "*GND*" "*VCC*" "*VDD*" "+*V" \
    --power-nets-widths 0.5 0.4 0.4 0.3 \
    --track-width 0.2 \
    --clearance 0.2
```

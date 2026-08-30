# Power/Ground Plane Via Connections

The `route_planes.py` script creates copper pour zones (plus optional via features: area stitching and GND return vias).

> **Since #562 the plane step does NO ROUTING — unconditionally.** This
> tool places **no tap vias and draws no traces** for pads that need to
> reach the plane; there is no flag or environment variable that re-enables
> the old tap mode (the machinery is deleted, not gated). Every such pad is
> left for the **route step**, which welds it into the pour with the full
> routing machinery (pour-launch) and taps whatever the fill cannot reach
> in its **in-run plane finalize**. Consequence for readers: a bare pour
> "leaving pads unconnected" is expected — run `route.py` over ALL nets
> (plane nets included, passed in `--power-nets`) and grade after that,
> not after the pour.
>
> Exposed/thermal pads are the one exemption: they still get their **via
> array** here (#487), because that stamps vias without drawing traces and
> nothing else in the chain places one. If the array cannot be fitted, the
> pad is deferred to the route step rather than traced to.

## Overview

When creating a ground or power plane on an inner or bottom layer, SMD pads on other layers need via connections to reach the plane. This tool automates:

1. **Zone creation** - Creates a copper pour zone covering the board. Replaces an existing zone for the same net/layer with new parameters; coexists with other nets' pours on that layer (warning + explicit fill priority)
2. **Pad classification** - Identifies which pads need vias vs direct zone connection (the ones needing vias are handed to the route step; see the note above)
3. **Thermal via arrays** (#487) - Exposed/thermal pads get a lattice of vias into the plane
4. **Area via stitching / GND return vias** - Optional via features (`--stitch-vias`, `--add-gnd-vias`)
5. **Resistance analysis** - Calculates and displays plane resistance and max current capacity


## Basic Usage

```bash
# Create GND plane on bottom layer (outputs to input_routed.kicad_pcb)
python py_router/route_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu

# Create GND plane, overwrite input file
python py_router/route_planes.py input.kicad_pcb --overwrite --nets GND --plane-layers B.Cu

# Create GND plane to specific output file
python py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu

# Create multiple planes at once (each net paired with corresponding plane layer)
python py_router/route_planes.py input.kicad_pcb --nets GND +3.3V --plane-layers In1.Cu In2.Cu

# Create VCC plane on inner layer with larger vias
python py_router/route_planes.py input.kicad_pcb --nets VCC --plane-layers In2.Cu --via-size 0.5 --via-drill 0.4

# Preview what would be placed without writing
python py_router/route_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu --dry-run
```

## Command-Line Options

### Required Options

| Option | Description |
|--------|-------------|
| `--nets`, `-n` | Net name(s) for the plane(s). Can specify multiple (e.g., "GND" "+3.3V") |
| `--plane-layers`, `-p` | Plane layer(s) for the zone(s), one per net (e.g., "In1.Cu" "In2.Cu") |

When specifying multiple nets, each net is paired with its corresponding plane layer. For example, `--nets GND VCC --plane-layers In1.Cu In2.Cu` creates a GND plane on In1.Cu and a VCC plane on In2.Cu.

### Via/Trace Geometry

| Option | Default | Description |
|--------|---------|-------------|
| `--via-size` | 0.5 | Via outer diameter in mm |
| `--via-drill` | 0.3 | Via drill size in mm |
| `--track-width` | 0.3 | Track width for via-to-pad connections in mm |
| `--clearance` | 0.25 | Clearance from other copper in mm |

### Zone Options

| Option | Default | Description |
|--------|---------|-------------|
| `--zone-clearance` | 0.2 | Zone clearance from other copper in mm |
| `--min-thickness` | 0.1 | Minimum zone copper thickness in mm |
| `--skip-existing-zones` | off | Keep an existing same-net zone on the target layer instead of replacing it (place stitching vias only). When off (the CLI default), an existing same-net zone on the target layer is replaced. **Other nets' pours on the layer are tolerated either way** — see below |

### Algorithm Options

| Option | Default | Description |
|--------|---------|-------------|
| `--grid-step` | 0.1 | Grid resolution in mm |
| `--hole-to-hole-clearance` | 0.2 | Minimum clearance between drill holes in mm (fab floor) |
| `--same-net-pad-clearance` | project record, else -1 | Edge-to-edge clearance (mm) between placed vias and same-net pads. `> 0` keeps **all** of this step's vias (stitching, taps, region joins) off same-net pads at that clearance, and records the value in the sibling `.kicad_pro` (`kicad_routing_tools.same_net_pad_clearance`) so later chain steps — `route.py`, `route_diff.py`, `bga_fanout`/`qfn_fanout`, `repair_planes` — inherit it (#581). `0` keeps its legacy stitching-only meaning; `-1` explicitly allows via-in-pad. Unset = use the project's recorded value if one exists |
| `--layers`, `-l` | F.Cu + plane-layers + B.Cu | All copper layers for routing and via span (auto-computed) |
| `--layer-costs` | 1.0 per layer (4+) or F.Cu=1.0/B.Cu=3.0 (2-layer) | Per-layer routing cost multipliers (1.0-1000) |

### Multi-net Plane Layer Options

These options control MST-based routing between vias when multiple nets share the same plane layer.

| Option | Default | Description |
|--------|---------|-------------|
| `--plane-proximity-radius` | 3.0 | Radius around other nets' vias for proximity cost (mm) |
| `--plane-proximity-cost` | 2.0 | Maximum proximity cost around other nets' vias (mm equivalent) |
| `--plane-track-via-clearance` | 0.8 | Clearance from MST track center to other nets' via centers (mm) |
| `--voronoi-seed-interval` | 2.0 | Sample interval for Voronoi seed points along routes (mm) |
| `--plane-max-iterations` | 200000 | Maximum A* iterations for routing plane connections |

The `--plane-track-via-clearance` parameter ensures MST routes don't pass through narrow gaps between other nets' vias. A larger value ensures more room for polygon fill but may cause routing failures on dense boards.

A net earns a Voronoi zone on a shared layer if it has **any** connection point there — a stitching via *or* a pad. A net whose pads are all through-hole or already on the layer places zero stitching vias, but its zone is still poured (seeded from those pads' positions); the partition is not gated on placing ≥1 via (issue #114).

A net with **no copper at all** on the layer still earns one. Because pours run before routing (#562), a virgin *inner* layer carries neither vias nor pads — every pad is SMD on an outer layer — so a requested inner split plane used to produce **zero zones** for *both* nets, with only a buried `no vias or pads on layer, skipping zone` warning to show for it (issue #598). Such a net is now seeded from its pads **projected onto the layer** (their x/y, whatever layer the copper sits on, off-board pads excluded): those positions exist regardless of routing state, they are where the route step's pour-launch vias come down, and they partition the layer the way the components themselves are spread over the board. These are partition seeds only — they never enter the via list, so no MST edge or plane route is invented for them, and a net that has seeds of its own is untouched.

### Re-routing Options

| Option | Default | Description |
|--------|---------|-------------|
| `--no-bga-zone` | off | Disable BGA auto-exclusion zones when re-routing ripped nets — use when the original signal route used `--no-bga-zones`, so the reroute uses compatible parameters |
| `--power-nets` | - | Glob patterns for power nets to route with wider tracks |
| `--power-nets-widths` | - | Track widths in mm for each power-net pattern |

The pour step does **no tapping and no ripping** (#562). Every pad that would need a tap via is deferred to the route step, which welds it into the pour with the full routing machinery (pour-launch) and taps whatever the fill cannot reach in its in-run plane finalize. The one exception is an exposed/thermal pad, whose via **array** (#487) stamps vias without drawing any trace. The blocker rip-up knobs (`--rip-blocker-nets`, `--max-rip-nets`, `--reroute-ripped-nets`) and the via-search radii (`--max-search-radius`, `--max-via-reuse-radius`, `--close-via-radius`) are **removed** from this script accordingly. Of those, only `--rip-blocker-nets` and `--max-search-radius` still exist on `repair_planes.py` (the standalone repair utility; `--reroute-ripped-nets` survives there as a documented no-op) — the other radii exist nowhere anymore. Recorded manifests were migrated in place by `tests/stress/migrate_manifests.py` (54 of 459 chains, 63 occurrences, all `--rip-blocker-nets`); passing a removed flag here now fails in argparse, which is the intended behavior.

After writing output, `route_planes.py` runs a **geometric verification** pass: it re-parses the board and reports, per plane net, how many pads are actually joined to the plane (via `check_net_connectivity`), and prints a NOTE when this disagrees with the via-placement counters. This surfaces pads whose stitching via is not electrically joined and TH pads on multi-net Voronoi layers that fell in the other net's region.

**Note:** The plane nets being processed are protected and will never be ripped up, even if they block each other during multi-net processing.

### Debug Options

| Option | Description |
|--------|-------------|
| `--dry-run` | Analyze and report without writing output file |
| `--verbose`, `-v` | Print detailed debug messages |
| `--debug-lines` | Draw MST routes on User.1, User.2, etc. per net |

### Area Via Stitching (#485)

A deliberate, periodic lattice of vias bonding a plane net's pours across the
copper layers it owns — the standard SI/EMI/thermal practice of stitching a
plane pair, rather than relying on wherever pad taps happened to land.

| Option | Default | Description |
|--------|---------|-------------|
| `--stitch-vias` | off | Enable area via stitching on this run's plane nets |
| `--stitch-pitch` | 20.0 | Lattice pitch (mm) |
| `--stitch-max-freq` | — | Maximum frequency of interest (MHz): derives the pitch as λ/20 using the largest dielectric ε_r in the board's stackup (FR-4 4.5 if none), overriding `--stitch-pitch` |
| `--stitch-edge-fence` | off | Board-edge via fence: a via row tracking the board outline(s) (EMI guard ring); works with or without `--stitch-vias` |
| `--stitch-fence-pitch` | *lattice pitch* | Via spacing along the fence (mm) |
| `--stitch-inset` | *auto* | Fence distance from the board edge to the via centers (mm). Auto = the board edge clearance plus the fill-margin ring — as close as a via can sit and keep the pour intact |

The stitched nets are always the `--nets` that own **two or more** of the
`--plane-layers` — there is deliberately no net-selection flag. Each lattice
site is accepted only when:

1. The predicted zone fill (the same `ZoneFillModel` the tap placement uses)
   contains the via **plus its clearance pocket plus a `min_thickness` ring**
   inside the *main* fill component on at least 2 of the net's layers — so a
   stitch never necks the pour below minimum thickness locally and never taps
   an isolated fill island.
2. The via obstacle map pad taps use clears the site (foreign copper at
   cross-class clearance, drill hole-to-hole, board edge, per-layer
   `.kicad_dru` rules).

A site already within `pitch/2` of a same-net via or plated through-hole
barrel is coverage-satisfied and skipped; a blocked site is nudged outward up
to `pitch/4` before being given up. The pass reports a coverage metric (max
lattice-site distance to the nearest same-net bond, before → after) and runs
in `--dry-run` too. The GUI planes tab exposes the same controls ("Area Via
Stitching").

The **edge fence** samples the true `Edge.Cuts` polygon(s) — a panelized
board fences every outline — inset toward the interior, and validates each
site with the exact same fill and obstacle gates as the lattice. The fence
runs *before* the lattice, so fence vias count as existing bonds for the
lattice's coverage rule and the rim is not stitched twice. Interior cutouts
are not fenced.

### GND Return Via Placement

When a signal transitions between layers via a via, the return current on the GND plane must also
transition. Without a nearby GND via, the return current takes a longer path, creating a slot
antenna that causes reflections and EMI. GND return vias provide a low-impedance return current
path close to each signal via.

**Note:** Differential pair routing (`route_diff.py`) adds its own GND return vias automatically.
This feature is for single-ended signal vias placed by `route.py`.

| Option | Default | Description |
|--------|---------|-------------|
| `--add-gnd-vias` | off | Enable GND return via placement near signal vias |
| `--gnd-via-net` | *auto* | Pin every return via to this net. Empty (the default) matches each signal to **its own ground domain** |
| `--gnd-via-distance` | 2.0 | Maximum distance from signal via to place GND via (mm) |

The algorithm:
1. Finds all signal vias (nets that are not a ground net)
2. Skips signal vias that already have a **same-domain** GND via or through-hole GND pad within the distance threshold
3. For each remaining signal via, searches outward from the minimum viable distance (via-to-via clearance) in 24 angles (every 15°)
4. Places the GND via at the closest valid position that respects track clearances, on the signal's own ground net

#### Split grounds (#489 §5)

Ground used to collapse to one board-global net, so every signal via -- analog
included -- was stitched to it. On a split-ground board that is worse than doing
nothing: it can bridge the very domains the split exists to separate, while
connectivity and DRC both report green.

Each ground-family net (`GND*`, `AGND`, `DGND`, `PGND`, ...) is now a **domain**,
seeded with the components that touch it. The single-point tie between domains --
a ferrite, a 0 Ω link, a net tie -- is detected structurally (a populated 2-pad
part with one pad on each ground) and excluded from that seeding, so membership
cannot leak across the split. A signal's return domain is then the domain of its
endpoint components; anything ambiguous falls back to the board-global answer, so
behaviour never gets worse than before.

Boards with one ground resolve exactly as before (verified across the corpus set:
12 single-ground boards unchanged for every net, 3 split-ground boards corrected).
A run on a multi-domain board prints the domains, the tie, and the per-domain via
count. `--gnd-via-net AGND` still pins everything to one net board-wide.

Helpers: `net_queries.resolve_ground_domains`, `ground_domain_bridges`,
`resolve_return_net_id`, `describe_ground_domains`.

#### Sharing a layer with another net's pour

A pour belonging to a **different net** on the target layer used to abort the run
("Cannot create *net* zone on same layer. Aborting.") — so a plane could not be
added to any board that already had a pour on that layer, and the abort then
crashed the post-passes on the output file it never wrote. KiCad itself allows
zones of different nets to share a layer: it fills them by priority and holds
normal clearance between nets.

The run now **warns and continues**, naming the other nets, and gives the new
plane a fill priority **one above the highest incumbent** so KiCad resolves the
overlap in favour of the plane you just asked for — the existing pours pull back
around it rather than the outcome depending on zone UUID order. The incumbent
zones themselves are left untouched. Point `--plane-layers` at a free layer if
you want the opposite. Priority is chosen upward because KiCad zone priority is
non-negative: a value below the incumbent is written but read back as 0.

Footprint-owned pours (a shield or thermal pour inside a footprint) neither
contend nor get replaced (#478).

#### Replacing a same-net pour

An existing pour for the **same** net on the target layer is replaced by default
(`--skip-existing-zones` keeps it instead). Two cases used to go wrong silently:

- **KiCad 10 boards.** Zones carry `(net "NAME")`, and the replacement filter was
  only given numeric `(net <id>)` pairs, so it never matched: the old pour shipped
  next to its replacement while the log said "Replacing existing zone", and every
  re-run stacked another duplicate. The name pairs are passed now.
- **Multi-layer pours.** `(layers "F.Cu" "B.Cu")` is **one** zone. It is *not*
  deleted to replace one of its layers -- that would destroy the copper on the
  others. The run warns, keeps the pour, and pours the new plane over it at a
  higher fill priority (same net, so no short); replace it outright by running
  those layers together or by deleting it in KiCad.

A pour is therefore removed only when every layer it covers is being replaced.

#### Choosing `--gnd-via-distance`

The `--gnd-via-distance` parameter sets the maximum search radius. The algorithm always places
vias as close as physically possible; this parameter controls how far away is acceptable if
close placement is blocked. Higher-frequency signals need tighter placement.

| Signal Speed | Frequency | Recommended Distance | Rationale |
|-------------|-----------|---------------------|-----------|
| Ultra-high | >1 GHz (DDR3/4, PCIe, USB3) | 2.0 mm | Lambda/20 ~ 7 mm at 1 GHz in FR4; tight return paths essential |
| High | 100 MHz - 1 GHz (Ethernet, QSPI, SDIO) | 3.0 mm | Good return path coverage |
| Medium | 10 - 100 MHz (SPI, JTAG) | 5.0 mm | Return current less localized |
| Low | <10 MHz (I2C, UART, GPIO) | Skip | GND plane provides adequate return path |

**Minimum physical limit:** Do not set `--gnd-via-distance` below 3 x (via_size + clearance),
typically ~2.5 mm for standard 0.8 mm vias with 0.25 mm clearance. Below this, no valid
placement positions exist.

Use the `/find-high-speed-nets` Claude Code skill to analyze component datasheets and determine
the appropriate distance for your board. The `/plan-pcb-routing` skill includes this as a
standard step when GND planes are present.

#### Example

```bash
python py_router/route_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu --add-gnd-vias --gnd-via-distance 2.0
```

Output:
```
Added 8 GND vias near signal vias
Placement distances: min=0.800mm, avg=1.131mm, max=1.450mm (min_distance theoretical=0.750mm)
```

## How It Works

### Pad Classification

The tool classifies each pad on the target net into three categories:

1. **Through-hole pads** - Have drill holes that connect all layers. No via needed.
2. **SMD pads on plane layer** - Already on the zone layer. No via needed.
3. **SMD pads on other layers** - Need a via + trace to connect to the plane.

### Via Placement Algorithm

> Since #562 this machinery no longer places per-pad tap vias here — pads
> needing a via are deferred to the route step. It remains in this module
> as the placement engine for **thermal via arrays**, **area stitching /
> GND return vias**, and as library code the repair engine
> (`repair_planes.py`, called from route.py's in-run finalize) and the BGA
> fanout import (`find_via_position`, `route_via_to_pad`,
> `route_multi_source_to_pad`).

For each via those consumers place, the algorithm:

1. **Check for nearby existing via** - If a via on the same net exists close by, reuse it
2. **Try pad center first** - If the pad center is not blocked, place via there (no trace needed). Skipped when `--same-net-pad-clearance >= 0`, in which case same-net pads are treated as obstacles and vias are always placed outside the pad with the requested edge-to-edge clearance.
3. **Spiral search outward** - Search in expanding rings for a valid position that:
   - Has clearance from existing vias, tracks, and pads on ALL copper layers
   - Can be routed to the pad using A* pathfinding
4. **Fallback to farther via** - If no new via position works, try reusing any existing via farther out

#### Same-net pad clearance / via-in-pad

By default the CLI keeps the legacy "via-in-pad" behavior — `--same-net-pad-clearance -1` means same-net pads are not added as via-placement obstacles, so a stitching via lands on the pad center whenever possible (no trace needed). Set `--same-net-pad-clearance` to a non-negative value to force vias outside same-net pads with that edge-to-edge clearance. For example, `--same-net-pad-clearance 0.25` keeps the same edge-to-edge gap as the global `--clearance`.

**Board-wide semantics (#581).** A value `> 0` is a *board-wide assembly constraint*, not just a stitching option: it is recorded in the sibling `.kicad_pro` and every later chain step keeps **all** of its vias — routing escape vias, the #189 via-in-pad rescue (disabled), layer-swap pad vias (declined), plane tap/join/reconnect vias, and the sub-grid via nudge — at that clearance from same-net **SMD** pads. BGA fanout runs its under-pad escapes in dog-bone mode and QFN fanout refuses via-in-pad. The same flag is accepted by `route.py`, `route_diff.py`, `repair_planes.py`, `bga_fanout.py` and `qfn_fanout.py`; unset means "use the project's recorded value". `0` and `-1` reproduce the pre-#581 behavior exactly.

In the GUI, the **Allow via-in-pad** checkbox and **Same-net Pad Clearance** spin control live at the top of the **Route tab's Options box** (moved from the Planes tab) and apply to *every* step run from the dialog. The checkbox defaults to ticked (via-in-pad allowed, CLI parity); unticking enables the spin control. When the board's project already records a clearance (a CLI chain step set it), the dialog opens with the checkbox unticked and the recorded value loaded.

**Hole-to-hole is separate from via-in-pad.** `--same-net-pad-clearance` governs only the *copper* clearance to same-net pads. The *drill-to-drill* (hole-to-hole) minimum is a physical fab constraint and is always enforced against every drilled hole regardless of net — so even with via-in-pad enabled (`-1`), stitching vias still keep `--hole-to-hole-clearance` away from same-net **through-hole** pad drills (and from other vias). This is why via-in-pad applies to same-net **SMD** pads (no drill), but a stitching via will never be placed within the hole-to-hole minimum of a same-net through-hole pad (issue #125).

### Via Obstacle Checking

Since vias are through-hole (spanning all layers), the obstacle map checks for clearance on **all copper layers**, not just the plane layer. This prevents DRC violations from vias overlapping tracks on inner layers.

### A* Trace Routing

When a consumer of the placement engine (repair taps, fanout escapes) needs
a via that cannot sit at the pad center, a trace is routed from the via to
the pad using A* pathfinding. The pour itself never draws these traces
(#562); the code lives here as the shared implementation. The routing:

- Avoids other-net pads and tracks
- Respects clearance requirements
- Uses the pad's layer for the trace

### Blocker Rip-up (removed from this script)

The pour no longer places taps, so it never needs to rip a blocker.
The rip-up algorithm still exists in the repair engine
(`repair_planes.py`, documented below), which the route
step's in-run plane finalize calls -- with ripping OFF by default.

### Multi-Net Layer Zone Generation

When multiple nets share the same plane layer (e.g., `--nets "VA19|VA11" --plane-layers In5.Cu`), the layer is composed as a **grammar pour** (#662, the default): the dominant net — largest board-wide reach × consumer pad count, scored on the net's full pad set — owns the layer as a **background sheet**, and every other net becomes **compact hull islands** (single-linkage 5 mm clusters, convex hull + 2 mm inflation). The nested islands outrank the sheet via zone fill priorities, so KiCad's fill performs the subtraction — no polygon booleans. Two connectivity invariants are enforced at composition time:

- each island is one connected region containing all of its cluster's pads (by construction — convex hulls cover their cluster);
- the background sheet must remain **one connected region after every carve**, checked on a coarse raster: an island that would sever the sheet (leaving a detached piece holding dominant-net pads/seeds, or ≥25% of the sheet) first shrinks its inflation, then **demotes to tracks** (printed; the route step carries every plane net in its `--nets`, so a demoted cluster is still served by copper — just not by a zone). A detached *source-less* sliver is not a severing — fill island removal culls it.

Rationale (measured on orangecrab vs its human original): the previous pad-Voronoi partition scored 0.3–2.6 mm mean cell widths and split the dominant rail into 7 crumbs; the human's grammar is one deep sheet (15.8 mm mean width) plus compact islands. `KICAD_GRAMMAR_POUR=0` reverts to the Voronoi partition, which also remains the fallback when the grammar is degenerate (a single seeded net, or no identifiable dominant net). The Voronoi path uses MST-based routing to ensure connected zones:

1. **Compute MST** - For each net, computes a Minimum Spanning Tree between all its vias
2. **Route MST edges** - Routes each MST edge on the plane layer using A* pathfinding, avoiding other nets' vias and previously routed paths
3. **Retry with reordering** - If some edges fail to route, retries with failed nets processed first (up to 5 iterations), keeping the best result
4. **Sample routes for Voronoi** - Samples points along successful routes as additional Voronoi seed points
5. **Compute final zones** - Uses Voronoi diagram with augmented seeds to create non-overlapping zone polygons per net

The MST routes ensure that each net's zone polygons are connected (if routing succeeds). The `--debug-lines` option outputs the MST routes on User.1, User.2, etc. for visualization.

### Plane Resistance Analysis

After zone creation, the tool calculates and displays approximate plane resistance and maximum current capacity for each polygon:

**For single-net layers:**
- Uses the bounding box diagonal as the path length
- Samples polygon width perpendicular to the diagonal path

**For multi-net layers:**
- Uses the longest path through the MST (tree diameter) as the path length
- Follows the actual routed traces, not straight-line distance
- Samples polygon width perpendicular to the path at 1mm intervals

**Calculations:**
- **Resistance:** `R = ρ × L / (W × t)` where ρ = 1.68×10⁻⁸ Ω·m (copper), L = path length, W = average width, t = copper thickness
- **Max current:** the **IPC-2221** chart fit `I = k × ΔT^0.44 × A^0.725`, k = 0.024 internal / 0.048 external, A = cross-sectional area in mils². This was documented as IPC-2152, but the formula and both k values are 2221's (#489 §6). IPC-2152 came later and specifically **overturned** 2221's 2× derating of internal layers — an inner trace in FR4 runs *cooler* than an external one in still air — so `Imax` under-credits inner planes by roughly 2×. `plane_resistance.calculate_max_current_ipc2152()` gives the corrected estimate, and it is printed alongside for internal layers.

**Assumptions:**
- Copper weight is read from the board's own **stackup**, per layer (1 oz / 35 µm only when the board has no stackup)
- 10°C maximum temperature rise
- **Chart range:** the 2221 curves are drawn for cross-sections up to ~700 mils². A plane pour is far outside that, so a result past the range is reported as *not rated* instead of as a current, with the extrapolated number shown only for reference — this is why the example below no longer states "21.05 A" as fact.

**Example output (single-net layer):**
```
Plane Resistance Analysis (1 oz copper, 10°C rise):
  Path length: 117.0 mm (diagonal)
  Avg width:   52.2 mm
  Resistance:  1.075 mΩ
  Max current: not rated -- 2832 mils² cross-section is past the IPC-2221
               chart range (700 mils²); the fit would say 21.05 A by extrapolation
```

**Example output (multi-net layer):**
```
Plane Resistance Analysis (1 oz copper, 10°C rise):
--------------------------------------------------------------------------
Net                       Path(mm)   AvgW(mm)   R(mΩ)      Imax(A)
--------------------------------------------------------------------------
/fpga_adc/VA19            33.6       3.6        4.457      3.03
/fpga_adc/VA11            96.3       5.7        8.121      4.22*
/fpga_adc/VLVDS           28.4       78.1       0.175      28.18*
/fpga_adc/VD11            31.7       4.3        3.537      3.44
--------------------------------------------------------------------------
Path = longest MST route, AvgW = avg polygon width along path
Imax = IPC-2221 chart fit; inner layers carry 2221's 2x derating, which IPC-2152 overturned
* extrapolated past the IPC-2221 chart range (700 mils²) -- not a rating
```

This helps identify potential current bottlenecks in power distribution networks. Narrow polygon sections (low AvgW) will have higher resistance and lower current capacity.

Each zone's numbers are also returned with the zone data (`resistance_analysis`),
so a caller can gate on them instead of scraping the printout.

## Error Messages

The tool provides clear error messages in red:

### VIA PLACEMENT FAILED
```
VIA PLACEMENT FAILED - no valid position within 10.0mm of pad at (x, y)
```
No position was found where a via could be placed. All candidate positions within the search radius were blocked by existing vias, tracks, or pads.

### ROUTING FAILED
```
ROUTING FAILED - no path from via (vx, vy) to pad at (px, py)
```
A via position was found (or an existing via was selected), but the A* router could not find a valid path from the via to the pad. This typically happens on dense boards where routing channels are blocked.

## Tips for Dense Boards

1. **Reduce via size** - Smaller vias have more placement options
   ```bash
   --via-size 0.4 --via-drill 0.3
   ```

2. **Run after other routing** - Place plane vias last so they work around existing routes

3. **Let the route step do the work** - Plane pads the pour cannot reach are not a
   pour failure: the route step welds them (pour-launch) and its in-run finalize
   taps the rest. Grade plane connectivity AFTER the route step, never here.

## Example Output

### Bare pour (the #562 default)

```
Loading PCB from input.kicad_pcb...
Found net 'GND' with ID 91
Board bounds: (71.12, 55.88) to (228.60, 147.32)

============================================================
Processing net 'GND' on layer B.Cu
============================================================

Pad analysis for net 'GND':
  Through-hole pads (no via needed): 31
  SMD pads on B.Cu (no via needed): 16
  SMD pads on other layers (via needed): 76 -> deferred to the route step

Results for 'GND':
  Zone created on B.Cu
  Thermal via arrays: 1 pad (9 vias)

  76 pad(s) are not connected to their plane by the pour alone -- EXPECTED
  (#562): the plane step places no taps, so these are welded by the route
  step's pour-launch and completed by its in-run plane finalize. Grade the
  board AFTER the route step, not here.

  DRC settings: updated 19 value(s) in output.kicad_pro to match the routed
  floors (close+reopen in KiCad if it is open)
JSON_SUMMARY: {"min_clearance_used": 0.25, "plane_nets": ["GND"],
  "plane_resistance": [{"net": "GND", "resistance_ohms": 0.0011,
  "max_current_a": 56.6, ...}]}
```

Vias and traces appear at the **route step** (`route.py --nets '*'` with
the plane nets in `--power-nets`): pour-launch welds pads into the fill,
and the in-run plane finalize taps whatever the fill cannot reach. Rip-up
of blockers, when needed, happens there too -- never in this script.

## Post-Processing

After running the tool:

1. **Open in KiCad** - Load the output file
2. **Refill zones** - Press `B` or use Edit > Fill All Zones to generate the copper pour
3. **Run DRC** - Verify no design rule violations
4. **Manual cleanup** - Address any failed placements or re-routes manually if needed

## Code Organization

The plane generation code is organized into several modules:

| Module | Description |
|--------|-------------|
| `route_planes.py` | Main CLI and orchestration - loads PCB, coordinates via placement per-net, and writes output |
| `plane_io.py` | I/O utilities - zone extraction, PCB file reading/writing, net ID resolution |
| `plane_obstacle_builder.py` | Obstacle map construction - builds grid-based maps for via placement and routing |
| `plane_blocker_detection.py` | Blocker detection and rip-up - identifies which nets are blocking via placement |
| `plane_zone_geometry.py` | Voronoi zone computation - computes non-overlapping zone polygons for multi-net layers |
| `plane_resistance.py` | Resistance analysis - calculates plane resistance and max current capacity |

### Key Functions

**route_planes.py:**
- `create_plane()` - Main orchestration function
- `find_via_position()` - Searches for valid via positions with routing verification
- `route_via_to_pad()` - A* routing from via to pad
- `route_plane_connection()` - Routes MST edges between vias on multi-net layers

**plane_io.py:**
- `extract_zones()` - Reads existing zones from PCB file
- `check_existing_zones()` - Reports what already occupies the target layer: the same-net pour to replace, plus other nets' pours to coexist with (it no longer aborts the run over a foreign pour)
- `shared_layer_zone_priority()` - Fill priority for a plane sharing a layer with another net's pour: one above the highest incumbent, so the overlap resolves deterministically
- `write_plane_output()` - Writes vias, traces, and zones to output file

**plane_obstacle_builder.py:**
- `build_via_obstacle_map()` - Creates obstacle map for via placement (all layers)
- `build_routing_obstacle_map()` - Creates obstacle map for single-layer routing
- `identify_target_pads()` - Classifies pads by connection type

**plane_blocker_detection.py:**
- `find_via_position_blocker()` - Identifies net blocking a via position
- `find_route_blocker_from_frontier()` - Identifies net blocking A* routing
- `try_place_via_with_ripup()` - Iterative rip-up and retry logic

**plane_zone_geometry.py:**
- `compute_zone_boundaries()` - Computes Voronoi-based zone polygons
- `find_polygon_groups()` - Groups adjacent polygons for connectivity analysis
- `sample_route_for_voronoi()` - Samples route paths for Voronoi seeding

**plane_resistance.py:**
- `analyze_single_net_plane()` - Calculates resistance using bounding box diagonal
- `analyze_multi_net_plane()` - Calculates resistance using longest MST path
- `find_mst_diameter_path()` - Finds longest path through MST (tree diameter)
- `calculate_average_width_along_path()` - Samples polygon width perpendicular to path
- `calculate_resistance()` - Computes R = ρL/Wt
- `calculate_max_current_ipc2221()` - Max current from the IPC-2221 chart fit (was `calculate_max_current_ipc`, which is kept as an alias)
- `calculate_max_current_ipc2152()` - Same fit with IPC-2152's internal/external correction (no 2× inner-layer derating)
- `ipc2221_area_in_range()` - Whether the conductor is inside the 2221 chart's data range (False ⇒ the number is an extrapolation, not a rating)
- `stackup_copper_oz()` - Copper weight in oz for a layer, read from the board's stackup

## Repairing Disconnected Plane Regions

After power planes are created, regions may become effectively split due to vias and traces from other nets cutting through the plane. The `repair_planes.py` script detects these disconnected regions and routes tracks between them to ensure electrical continuity.

**Note (#562): in the default chain this runs for you.** `route.py`'s in-run
plane finalize calls this same engine, then the plane-copper cleanup and the
KiCad-oracle completion check, at the route step's own parameters — so the
standalone invocation below is for boards routed outside that chain.
`KICAD_PLANE_FINALIZE=0` disables the in-run pass.

Key features:
- **Per-net processing** - Zones with the same net on multiple layers (e.g., GND on B.Cu and In1.Cu) are processed together, avoiding redundant routes since vias connect all layers
- **Cross-layer connectivity** - Uses vias and through-hole pads to track connectivity across all zone layers for a net
- **Wide track routing** - Tries track widths from max (2.0mm) down to min, using the widest that fits

### Basic Usage

```bash
# Auto-detect all zones in PCB and repair disconnected regions (outputs to input_routed.kicad_pcb)
python py_router/repair_planes.py input.kicad_pcb

# Auto-detect all zones, overwrite input
python py_router/repair_planes.py input.kicad_pcb --overwrite

# Auto-detect all zones to specific output file
python py_router/repair_planes.py input.kicad_pcb output.kicad_pcb

# Specific nets and layers
python py_router/repair_planes.py input.kicad_pcb --nets GND --plane-layers B.Cu

# Customize track width and clearance
python py_router/repair_planes.py input.kicad_pcb --max-track-width 1.0 --clearance 0.2

# Increase iterations for difficult routes
python py_router/repair_planes.py input.kicad_pcb --max-iterations 500000
```

### Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--nets`, `-n` | auto | Net name(s) to process. If omitted, all nets with zones are processed |
| `--plane-layers`, `-p` | auto | Layer(s) to process. If omitted, all layers with zones are processed |
| `--layers`, `-l` | all Cu | Layer(s) available for routing |
| `--max-track-width` | 2.0 | Maximum track width for connections (mm) |
| `--min-track-width` | 0.2 | Minimum track width for connections (mm) |
| `--track-width` | 0.3 | Default track width for routing config (mm) |
| `--clearance` | 0.25 | Trace-to-trace clearance (mm) |
| `--zone-clearance` | 0.2 | Zone fill clearance around obstacles (mm) |
| `--track-via-clearance` | 0.8 | Clearance from tracks to other nets' vias (mm) |
| `--hole-to-hole-clearance` | 0.2 | Minimum clearance between drill holes (mm, fab floor) |
| `--board-edge-clearance` | 0.5 | Clearance from board edge (mm) |
| `--via-size` | 0.5 | Via outer diameter (mm) |
| `--via-drill` | 0.3 | Via drill diameter (mm) |
| `--grid-step` | 0.1 | Routing grid step (mm) |
| `--analysis-grid-step` | 0.5 | Grid step for connectivity analysis (coarser = faster) |
| `--max-iterations` | 200000 | Maximum A* iterations per route attempt |
| `--repair-pads` / `--no-repair-pads` | on | Also repair pad-level plane connection failures (see below) |
| `--max-search-radius` | 10.0 | Max radius to search for a via position during pad repair (mm) |
| `--rip-blocker-nets` | off | Connect a pad that can't reach its plane by tracing to a nearby same-net pad, ripping the signal net(s) blocking that trace (see below) |
| `--max-rip-nets` | 3 | Maximum blocker nets to rip per pad |
| `--reroute-ripped-nets` | off | **Deprecated no-op** (issue #141 reverted): ripped nets are always left unrouted for a later `route.py` pass, which does rip-up/restore safely. The old in-step reroute restored a failed net's original copper on top of copper meanwhile routed through its corridor, creating shorts the obstacle map never saw — which is why it was removed rather than fixed |
| `--power-nets` | — | Power net names needing wider tracks when re-routing ripped nets |
| `--power-nets-widths` | — | Track width (mm) per `--power-nets` entry, for re-routing ripped nets |
| `--no-bga-zone` | off | Disable BGA auto-exclusion zones when re-routing ripped nets (match the signal run) |
| `--dry-run` | off | Analyze without writing output |
| `--verbose`, `-v` | off | Print detailed debug messages |
| `--debug-lines` | off | Add debug lines on User.4 layer showing route paths |
| `--no-fix-drc-settings` | off | Skip rewriting the output project's DRC design rules to match the plane routing floors. By default they are made consistent (clearance, hole/edge, track/via floors + Default net class, non-routing severities demoted) so KiCad's manual DRC shows only genuine violations (issue #160; see [DRC Settings Fixer](utilities.md#drc-settings-fixer-fix_kicad_drc_settingspy)) |
| `--keep-thermal` | off | When fixing DRC settings, leave thermal-relief severity (`starved_thermal`) untouched instead of demoting it to a warning |

### Pad-Level Repair (`--repair-pads`, default on)

`route_planes.py` can leave a tail of pads it could not via down to the plane
(congested SMD neighborhoods). Before the region repair, the tool finds pads
of each plane net with no geometric connection to the plane — no same-net
via or segment touching the pad's copper (including vias landed inside the
pad), and the pad not sitting directly on a zone layer — and retries each
with a stitching via + short trace. The retry uses the same parameter
escalation as `route_planes.py`: the run parameters first, then scoped fine
parameters when the pad is fine-pitch (a same-component neighbor pad within
0.65mm, or pad min dimension below 0.35mm). The fine retry uses a finer grid
(0.05mm) and steps the clearance DOWN from the run value toward the
manufacturing floor for the board's layer count (the JLC fab floor, ~0.127mm
2-layer / 0.10mm 4+), narrowing the tap track to the fab track floor, and stops
at the loosest clearance that routes — there is no hard-coded "fine clearance"
(issue #226). The clearance it actually used is recorded into the output
`.kicad_pro` DRC floor and `JSON_SUMMARY` (`min_clearance_used`) so `check_drc`
grades the board at it. Obstacle maps for each retry are built on a small window
around the pad, so fine grids stay cheap on large boards. Per-pad outcomes are printed, and pads that still fail are listed in
the summary. Use `--no-repair-pads` to only reconnect zone islands.

**At defaults `route_planes.py` taps nothing** (#562, see the note at the top),
so it is the ROUTE step's in-run plane finalize that runs this repair —
including the fine-parameter retry (#104) — at the route step's own
parameters. The text above describes that engine; it applies wherever it
runs. The standalone script is for boards routed outside the chain.

### Rip-Blocker Pad Repair (`--rip-blocker-nets`)

Some plane-net pads can't take a via at all — a tiny outer-layer pad (e.g. a
0.4mm USB-connector GND pin) amid congestion, where the plane is on an inner
layer. (Under the #562 chain the route step routes the plane nets too, so
this is rarer: the pad usually welds into the pour directly.) The human connects these with
a short trace to an adjacent same-net pad (a connector GND pin to its shield
pad). With `--rip-blocker-nets`, the repair does the same: when no via fits and
no same-net via is within the close-reuse radius, it routes a trace to the
nearest same-net pad/via reachable on the pad's layer. If a signal net crosses
that corridor, it is **ripped** (up to `--max-rip-nets`), the pad connected, and
the ripped net left **unrouted** for a subsequent `route.py` pass to reconnect
(in-step rerouting was removed — issue #141 reverted — because restoring a
failed net's original copper on top of copper meanwhile routed through its
corridor created shorts the obstacle map never saw; `route.py` does
rip-up/restore safely). Pass `--power-nets`/`--power-nets-widths` so that
follow-up pass routes power nets at their proper width. Example:

```bash
python py_router/repair_planes.py step_planes.kicad_pcb out.kicad_pcb \
    --clearance 0.15 --via-size 0.5 --via-drill 0.3 --track-width 0.127 --grid-step 0.05 \
    --rip-blocker-nets
python py_router/route.py out.kicad_pcb out_reconnected.kicad_pcb --nets '*'   # reconnects the ripped nets
```

In the plugin, plane repair is no longer a tab of its own: it runs inside
every route step's in-run plane finalize (#562), where the same rip-blocker
arbitration applies. In the default chain you never invoke this
step — `repair_planes.py` is for boards routed outside the chain.

### How It Works

#### 1. Region Detection (Flood Fill)

Uses flood fill on a coarse grid (`--analysis-grid-step`) to identify disconnected regions:

1. Build obstacle grid from other nets' vias, traces, and pads
2. Mark cells blocked by obstacles (with clearance)
3. Find all anchor points (vias + through-hole pads) for the target net
4. Flood fill from each anchor to identify connected regions
5. Group anchors by their connected region

Pad-less **orphan islands** found by the fill model are classified by what
KiCad's filler would do on refill (#609/#611): an island the filler ERASES
(truly bare, `island_removal_mode` 0) is never strapped — that would ship
copper that is never poured — but it is reported as a `Zone SPLIT` with its
area, because a split reference plane is a return-path defect even when the
copper disappears. An island the filler KEEPS (it carries a same-net
track/via) is a real KiCad `Missing connection` and is **joined at any size**
(≥1 mm²; the 25 mm² area bar only guards clutter joins for erased copper).
On a multi-layer plane, every poured layer is scanned (#611): a kept island
cut off on a non-primary layer is reported by the first pass, then joined by
a **follow-up pass with that layer as the primary analysis layer**. An
island whose same-net via reaches anchored fill on another poured layer is
recognized as connected through the stack and left alone. This holds on both
discovery paths (#612): the raster fallback (used when the fill models can't
build) runs its own per-layer sweep, and the primary analysis layer is
auto-swapped to a layer whose fill model built rather than silently dropping
the whole net to the raster path.

#### 2. MST-Based Region Selection

Uses Kruskal's algorithm to build a Minimum Spanning Tree connecting all regions:

1. For each pair of regions, find the closest anchor points
2. Sort edges by distance
3. Build MST using union-find, selecting edges that connect previously unconnected regions

This ensures we connect all regions with minimum total trace length.

#### 3. Multi-Point A* Routing

For each MST edge connecting two regions, the router uses **multi-point A*** with all anchors from each region:

```
Region A (N anchors) <-> Region B (M anchors)
```

Instead of routing between the single closest pair, the router:
1. Sets ALL N anchors from region A as source cells
2. Sets ALL M anchors from region B as target cells
3. A* expands from all sources simultaneously, finding the best path to ANY target

This allows the router to find viable paths even when the closest anchor pair is blocked.

#### 4. Bidirectional Routing

If the forward search fails, the router tries the reverse direction:

1. **Direction 1:** Region A sources → Region B targets
2. **Direction 2:** Region B sources → Region A targets (if direction 1 fails)

A* can find different paths depending on which direction it expands from, so trying both directions increases success rate.

#### 5. Open-Space Fallback

If both directions fail, the router looks for "open space" points with maximum clearance:

1. Search near each region's centroid for cells with maximum distance from obstacles
2. Add these open-space points as additional anchors
3. Retry routing with the augmented anchor sets

If successful via an open-space point, a via is placed there to connect the route.

```
Example output:
[42/59] Region 6 (37 anchors) <-> Region 23 (2 anchors)... OK width=0.20mm, length=2.1mm (via open-space)
```

#### 6. Wide Track Routing

For power plane connections, wider tracks are preferred to minimize resistance. The router tries track widths from max to min:

1. Start with `--max-track-width` (default 2.0mm)
2. If routing fails, try half the width (1.0mm, 0.5mm, 0.25mm, ...)
3. Continue until `--min-track-width` (default 0.2mm) is reached
4. Use the widest width that successfully routes

This uses the Rust router's `track_margin` parameter to check extra clearance around the path without rebuilding the obstacle map, making width attempts fast.

```
Example output:
[1/3] Region 0 <-> Region 2... OK width=2.00mm, length=14.4mm
[2/3] Region 0 <-> Region 1... OK width=0.50mm, length=8.2mm (narrower due to obstacles)
[3/3] Region 0 <-> Region 3... OK width=1.00mm, length=12.1mm
```

#### 7. Via Placement at Layer Transitions

At layer transitions, the router checks if a via already exists at the transition point:

1. At each layer transition, check if an existing via is already at that exact position
2. If a via exists, no new via is added (the route uses the existing via)
3. If no via exists, a new via is placed at the transition point

Since routes can start and end on any layer at via locations, this simple approach avoids duplicate vias while keeping the routing logic straightforward.

#### 8. Incremental Obstacle Updates

After each successful route:

1. Add the route's segments to the obstacle map (blocks future routes)
2. Add new vias to the obstacle map with proper clearances:
   - Track-to-via clearance for routing
   - Hole-to-hole clearance for via placement
3. Add new vias to the reusable via list for subsequent routes

### Example Output

```
Loading PCB from input.kicad_pcb...
Board bounds: (71.12, 55.88) to (228.60, 147.32)

Routing disconnected plane regions
============================================================

[GND] on B.Cu, In1.Cu (clearances: B.Cu=0.3mm, In1.Cu=0.5mm):
  Building obstacle map... done
  Found 4 disconnected regions (131 total anchors)
  Routing 3 connection(s) to join regions...
    [1/3] Region 0 (2 anchors) <-> Region 2 (127 anchors)... OK width=1.00mm, length=52.2mm, 4 via(s)
    [2/3] Region 0 (2 anchors) <-> Region 1 (1 anchors)... OK width=0.25mm, length=14.4mm, 1 via(s)
    [3/3] Region 0 (2 anchors) <-> Region 3 (1 anchors)... OK width=0.50mm, length=13.0mm, 1 via(s)
  Result: All 3 route(s) succeeded

[+3.3V] on F.Cu, In2.Cu (clearances: F.Cu=0.2mm, In2.Cu=0.5mm):
  Building obstacle map... done
  Found 4 disconnected regions (128 total anchors)
  Routing 3 connection(s) to join regions...
    [1/3] Region 0 (114 anchors) <-> Region 1 (7 anchors)... OK width=1.00mm, length=10.3mm
    [2/3] Region 0 (114 anchors) <-> Region 3 (4 anchors)... OK width=2.00mm, length=14.4mm
    [3/3] Region 0 (114 anchors) <-> Region 2 (3 anchors)... OK width=2.00mm, length=16.5mm, 2 via(s)
  Result: All 3 route(s) succeeded

============================================================
SUMMARY
============================================================
  Zones processed: 4
  Total routes added: 6
  Total vias added: 8
```

Note: Zones with the same net on multiple layers are processed together (e.g., "GND on B.Cu, In1.Cu") since vias connect all layers. Per-layer zone clearances are shown and used for obstacle map construction.

### Why Routes Fail

Routes can fail for several reasons:

1. **All anchors blocked** - If every anchor point in a region is surrounded by obstacles with no path out
2. **Iteration limit reached** - Complex routes may exceed `--max-iterations` before finding a path
3. **No viable path exists** - Dense obstacle configurations may completely block all paths between regions

Increasing `--max-iterations` can help with complex routes. Reducing `--analysis-grid-step` provides finer region detection but is slower.

### Code Organization

| Module | Description |
|--------|-------------|
| `repair_planes.py` | CLI and orchestration - loads PCB, detects zones, coordinates region repair |
| `plane_region_connector.py` | Region detection and routing - flood fill analysis, multi-point A* routing, open-space fallback |

### Key Functions

**repair_planes.py:**
- `repair_planes()` (alias `route_planes` kept) - Main orchestration: loads PCB, iterates over nets, writes output
- `auto_detect_zones()` - Scans PCB for existing zones and returns net/layer pairs

**plane_region_connector.py:**
- `find_disconnected_zone_regions()` - Flood fill to identify regions and their anchors
- `find_region_connection_points()` - Builds MST edges between regions
- `route_disconnected_regions()` - Orchestrates routing for one net/layer
- `route_plane_connection_wide()` - Multi-point A* routing between regions
- `find_open_space_point()` - Finds cell with maximum clearance from obstacles
- `build_base_obstacles()` - Builds obstacle map for routing
- `add_route_to_obstacles()` - Incrementally updates obstacles after each route

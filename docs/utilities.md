# Utility Scripts

This document describes the utility scripts included with the KiCad Grid Router.

## DRC Checker (`check_drc.py`)

Checks a routed PCB for Design Rule Check violations.

### Usage

```bash
python check_drc.py output.kicad_pcb [OPTIONS]

Options:
  --clearance FLOAT    Track-to-track clearance in mm. Default: auto-detected from
                       the sibling .kicad_pro Default net-class clearance (the value
                       the routing steps recorded as actually used, incl. auto-stepped
                       fine-pitch taps); falls back to 0.2 if no project is found.
  --via-clearance FLOAT  Via-to-track clearance in mm (uses --clearance if not set)
  --hole-to-hole-clearance FLOAT  Minimum drill hole edge-to-edge clearance in mm
                                  (default: 0.20, the JLC fab floor — same as routing)
  --board-edge-clearance FLOAT    Minimum clearance from board edge in mm (0 = use --clearance)
  --clearance-margin FLOAT  Fraction of --clearance used as tolerance (default: 0.05 = 5%).
                            Violations smaller than clearance*margin are ignored — filters
                            grid-quantization noise (~8 µm artifacts); raise to e.g. 0.1 to
                            suppress more borderline grazes
  --nets PATTERN       Only check nets matching pattern
  --debug-lines        Output debug lines on User.7 showing violation locations
  --max-print INT      Max violations listed per type before "... and N more"
                       (0 = print all; default 20)
  --no-size-checks     Skip the track-width and via/hole-size fab-floor checks
  --min-track-width FLOAT   Minimum manufacturable track width in mm
                            (default: JLC fab floor for the board layer count)
  --min-via-diameter FLOAT  Minimum via outer diameter in mm (default: fab floor)
  --min-via-drill FLOAT     Minimum via drill diameter in mm (default: fab floor)
  --size-margin FLOAT       Absolute tolerance in mm for the size checks (default: 0)
  --check-pad-edge          Also check pad-to-board-edge clearance. Off by default:
                            pad-edge violations are almost always pre-existing
                            edge-connector pads, not router-introduced.
  --fab-tier {standard,advanced}  JLC fab capability floor the size checks grade against
                            (default: standard). Pass the tier the board was routed to so
                            legitimately-escalated fine geometry is not flagged
  --fab-overrides FILE      Fab-floor override file overlaying the selected --fab-tier
                            (see [Fab Tier Options](configuration.md#fab-tier-options))
```

The per-type listing is capped at `--max-print` (default 20); when a type has
more, a `... and N more (use --max-print 0 to show all)` marker is printed so
the listing is never silently truncated relative to the header count. Use
`--max-print 0` to dump everything for log-based triage.

### Examples

```bash
# Auto-grade at the clearance the board was routed to (read from its .kicad_pro)
python check_drc.py routed.kicad_pcb

# Override the grading clearance explicitly
python check_drc.py routed.kicad_pcb --clearance 0.15

# Override clearance and hole-to-hole explicitly (hole-to-hole defaults to the 0.20 fab floor)
python check_drc.py routed.kicad_pcb --clearance 0.2 --hole-to-hole-clearance 0.2

# Check specific nets only
python check_drc.py routed.kicad_pcb --nets "*DATA*"
```

### Checks Performed

The DRC checker validates:

1. **Track-to-track clearance** - Segments on the same layer maintain minimum clearance
2. **Track-to-pad clearance** - Tracks maintain clearance from pads on other nets
3. **Via-to-track clearance** - Vias maintain clearance from tracks on other nets
4. **Via-to-via clearance** - Vias maintain clearance from each other
5. **Pad-to-via clearance** - Vias maintain clearance from pads on other nets
6. **Pad-to-pad clearance** - Pads of different nets on a shared copper layer maintain clearance (catches overlaps/shorts, e.g. a placement step nudging a part onto a foreign pad). Pads of the *same* footprint are skipped — a component's own fine-pitch pin gaps are fixed library geometry, not a pipeline-introduced defect.
7. **Copper-to-hole clearance** - Tracks maintain clearance from NPTH (no-copper) drill holes of other nets — a real fab short the via-drill hole check misses (e.g. a track across a mounting hole)
8. **Hole-to-hole clearance** - Drill holes (via drills and through-hole pad drills) maintain edge-to-edge clearance, even on the same net (manufacturing constraint)
9. **Board edge clearance** - Tracks and vias maintain clearance from the real `Edge.Cuts` outline (outer ring **plus interior cutouts/slots**), so copper routed into a cutout — which sits inside the bounding box — is caught. Falls back to the bounding box when the parser finds no usable outline. With `--check-pad-edge`, pads are checked too.
10. **Same-net crossings** - Detects tracks crossing on the same layer within a net
11. **Track width** - Segments are at least the fab-floor minimum track width (the active `--fab-tier`'s deepest floor; standard = JLC 0.127mm on 2-layer, 0.0889mm on 4+ layer). Catches sub-fab copper a clearance-only check misses — a board's own `min_track_width` DRC rule can be lowered to match undersized tracks, so it never trips; the fab floor is the real limit.
12. **Via / hole size** - Via outer diameter and drill are at least the deepest fab via the tier can reach — the advanced (small/fine) via the router escalates to: JLC 0.25mm/0.15mm. Pass `--fab-tier` so grading matches how the board was routed (see [Fab Tier Options](configuration.md#fab-tier-options)).

Checks 11–12 are on by default; pass `--no-size-checks` to skip them, or override the floors with `--min-track-width` / `--min-via-diameter` / `--min-via-drill`. The floor is derived from the board's copper-layer count unless overridden.

### Clearance Margin

The DRC checker uses a 5% clearance margin by default to ignore tiny geometric artifacts from corner handling. For example, with a 0.1mm clearance, violations smaller than 0.005mm are ignored.

### Debug Visualization

With `--debug-lines`, the checker outputs debug lines on the `User.7` layer in the PCB file, showing the closest points between violating segments:

```bash
python check_drc.py routed.kicad_pcb --debug-lines
```

Output includes the distance of each violation:

```
Debug line: (10.200, 15.300) to (10.205, 15.305), distance: 0.0025mm
Writing 3 debug lines to User.7 layer
```

Open the PCB in KiCad and enable the `User.7` layer to visualize where clearance violations occur.

### Output

```
Checking clearances in routed.kicad_pcb...
Track clearance: 0.2mm
Via clearance: 0.2mm

Checking 1,234 segments and 89 vias...

Found 0 DRC violations
```

If violations are found:

```
VIOLATION: Track too close to track
  Net-(U2A-DATA_0) segment (10.2, 15.3)-(10.4, 15.5) on F.Cu
  Net-(U2A-DATA_1) segment (10.3, 15.2)-(10.5, 15.4) on F.Cu
  Distance: 0.185mm (minimum: 0.2mm)
```

## Connectivity Checker (`check_connected.py`)

Verifies that all nets are fully connected after routing. Detects two types of issues:

1. **Unrouted nets** - Nets with pads but no tracks (routing was never attempted or failed)
2. **Broken routes** - Nets with tracks that don't connect all pads (incomplete routing)

Unrouted detection is based on whether a net is actually covered by a copper
zone, not a hard-coded name list — a power net (GND, +3V3, …) with no plane on
this board *is* reported as unrouted. Same-net pads whose copper overlaps (e.g.
a castellated module's co-located through-hole + SMD pad pair) count as
connected even with no track between them.

### Usage

```bash
python check_connected.py output.kicad_pcb [OPTIONS]

Options:
  --nets PATTERN       Only check nets matching pattern
  --component, -C REF  Check all nets connected to a component (e.g., U1)
  --tolerance FLOAT    Connection tolerance in mm (default: 0.02)
  --verbose            Show detailed break location info
  --quiet              Only print summary line unless issues found
  --routed-only, -r    Only check routed nets (skip unrouted net detection)
```

### Examples

```bash
# Check all nets (detects both unrouted and broken routes)
python check_connected.py routed.kicad_pcb

# Check specific nets
python check_connected.py routed.kicad_pcb --nets "*lvds*"

# Check all nets on a component
python check_connected.py routed.kicad_pcb --component U102

# Check specific patterns on a component
python check_connected.py routed.kicad_pcb --component U1 --nets "*DATA*"

# Only check connectivity of routed nets (skip unrouted detection)
python check_connected.py routed.kicad_pcb --routed-only

# Verbose output with break locations
python check_connected.py routed.kicad_pcb --verbose
```

### Output

```
Checking connectivity in routed.kicad_pcb...

Checking 64 nets...

All nets are fully connected!
```

If issues are found:

```
============================================================
FOUND 3 ISSUES:

  Unrouted nets (2):
    /PROG- (2 pads)
    /PC-A3 (3 pads)

  Connectivity issues (1):

  Net-(U2A-DATA_5) (net 42):
    Segments: 12, Vias: 2, Pads: 3
    Disconnected components: 2
    Disconnected pads:
      (25.10, 30.40) on F.Cu [U2A]
============================================================
```

## Orphan Stub Checker (`check_orphan_stubs.py`)

Detects trace endpoints that end without a proper connection point (via or pad).

### Usage

```bash
python check_orphan_stubs.py input.kicad_pcb [OPTIONS]

Options:
  --net NET_NAME       Only check this net
  --layer LAYER        Only check this layer
  --compare            Compare two files to find new/removed orphans
```

### Examples

```bash
# Check all nets
python check_orphan_stubs.py board.kicad_pcb

# Check specific net and layer
python check_orphan_stubs.py board.kicad_pcb --net "+3.3V" --layer F.Cu

# Compare before/after to find new orphans
python check_orphan_stubs.py original.kicad_pcb modified.kicad_pcb --compare
```

### What It Detects

An orphan stub is a trace endpoint that:
1. Has only one connected segment (degree-1 node in the connectivity graph)
2. Is NOT near a via
3. Is NOT near a through-hole pad

These represent traces that end without a proper electrical connection.

### Output

```
Loading board.kicad_pcb...

Checking for orphan trace stubs...

============================================================
FOUND 2 ORPHAN STUBS:

  +3.3V on F.Cu: 1 orphans
    (142.50, 98.30)
  +3.3V on B.Cu: 1 orphans
    (155.20, 102.10)
============================================================
```

When no orphans are found:

```
Loading board.kicad_pcb...

Checking for orphan trace stubs...

============================================================
NO ORPHAN STUBS FOUND!
============================================================
```

When comparing two files:

```
Orphan Stub Comparison
============================================================
File 1 (original.kicad_pcb): 5 orphans
File 2 (modified.kicad_pcb): 3 orphans

New orphans in file 2: 0
Removed orphans (fixed): 2
```

## Pad Geometry Checker (`check_pads.py`)

Reports same-footprint, different-net pads whose copper **overlaps** - a short, and
almost always a sign that the pad's rotation or size is being modelled wrong. The
classic trigger is a QFN/QFP/BGA placed at a non-orthogonal board angle: if the pad
rectangles are mis-oriented, adjacent pads collide here and any fanout escapes its
stubs across them. Run this before fanout as a sanity check (the fanout tools also run
it automatically on their component and warn).

### Usage

```bash
python check_pads.py board.kicad_pcb [OPTIONS]

Options:
  --component, -c REF   Restrict the check to one footprint reference
  --cross-footprint     Also check overlaps across different footprints
                        (broader, noisier board-level short check)
  --tolerance MM        Minimum overlap depth to report (default 0.05)
  --quiet, -q           Print only the PASS/FAIL summary line
```

Only pads that share a copper layer are compared, so edge-connector fingers on opposite
sides and a part's top/bottom ground pads never false-trip. Net-0 (no-connection) pads -
fiducials, mechanical pads - are ignored. The exit code is the number of overlapping
pairs (0 = clean), so it gates a pipeline.

### Examples

```bash
# Whole board, checked per footprint
python check_pads.py board.kicad_pcb

# One footprint (e.g. before fanning it out)
python check_pads.py board.kicad_pcb --component U23

# Broader board-level short check across all footprints
python check_pads.py board.kicad_pcb --cross-footprint
```

### Output

```
FAILED: 2 overlapping different-net pad pair(s):
  U23.92 (SCL) <-> U23.93 (SDA)  overlap 0.480 mm  at (142.00,148.45)
  ...
Likely a pad-rotation modelling error - fix before running fanout.
```

```
OK: no overlapping different-net pads (tolerance 0.05 mm)
```

## Net Analyzer (`list_nets.py`)

Analyzes nets in a PCB file. Can list nets on a component, detect differential pairs, identify power/ground nets, and report the board's net-class design rules.

### Usage

```bash
python list_nets.py input.kicad_pcb [OPTIONS]

Options:
  --component, -c REF   Component reference (e.g., U1)
  --pads                Show pad-to-net assignments
  --diff-pairs, -d      Detect differential pairs (a name-based P/N pair is
                        suppressed when both nets land on a single
                        crystal/oscillator footprint, since a 2-terminal
                        resonator is not a differential signal — issue #145)
  --power, -p           Show power/ground nets with pad counts
  --design-rules, -r    Show net-class clearance/track/via/diff-pair rules
                        and the CLI flags to pass them to the routing tools
  --top N               Show top N most-connected nets (default: 10)
  --pattern GLOB        Filter nets by pattern
```

### Design rules

The routers **auto-read** the board's net classes from the sibling `.kicad_pro`
and honor KiCad's cross-class `max(classA, classB)` spacing. `--clearance` is a
**ceiling**: given, every class (Default included) is capped at `min(class,
--clearance)`; omitted, each net routes at its own net-class clearance (base = the
board's Default class), and `--hole-to-hole-clearance` / `--board-edge-clearance`
default to the board's own `min_hole_to_hole` / `min_copper_edge_clearance`
constraint (#439). They do **not** infer per-net track/via nominals on their own,
so `--design-rules` reads the real rules for you to pass explicitly:

```bash
python list_nets.py board.kicad_pcb --design-rules
```

It reads two tiers of rules and combines them with the JLCPCB fab floor:

- **Net-class values** from the sibling `.kicad_pro` (KiCad 8+,
  `net_settings.classes`) or `(net_class …)` blocks (KiCad 6/7): `clearance`,
  `track_width`, `via_diameter`, `via_drill`, `diff_pair_gap`, `diff_pair_width`.
  These are KiCad *drawing defaults*; only `clearance` is a DRC minimum.
- **Board Constraints** from `board.design_settings.rules`: `min_clearance`,
  `min_track_width`, `min_via_diameter`, `min_hole_to_hole`,
  `min_through_hole_diameter`. **These are what DRC actually enforces** for the
  geometric rules — so a board can use a via/track *smaller* than the net-class
  nominal and still pass DRC (issues #111/#115).

From these it prints a **manufacturing floor** (the Constraint or the JLC fab
minimum for the board's layer count, whichever is larger). The floor spells out
two distinct rules the router honours: **hole-to-hole** (drill-to-drill) is
net-INDEPENDENT and applies to via/via, via/pad-drill and pad-drill/pad-drill on
*all* nets including same-net; **copper clearance** applies to via/pad and
via/via copper between *different* nets, while same-net via-pad copper clearance
may be 0 (via-in-pad) where the fab allows it. The floor's **track** value is the
DRC-enforced one (the board's own `min_track_width`); separately it reports a
**fine-pitch signal track floor** — the fab's *physical* minimum trace width,
which is layer-dependent (JLC: 3.5 mil / 0.0889 mm on 4+ layer, 5 mil / 0.127 mm
on 2-layer) and can be *below* the board's `min_track_width`. On dense/congested
boards, route ordinary signals at that fab floor (`route.py --track-width`), below
the board's own rule, like the human originals — thinner completes more nets and
routes faster; keep power nets on `--power-nets` and impedance nets at net-class
width. It then emits ready-to-paste flags: the small **working**
`--via-size`/`--via-drill` from the floor (not the net-class `via_diameter`), and
`check_drc.py` is graded at the floor
(`--clearance <floor> --hole-to-hole-clearance <floor>`), not the inflated
net-class clearance. Nets assigned to a non-Default class are reported so they
can be routed separately with that class's values. When neither net classes nor
Constraints exist, it falls back to the JLC fab floor for the layer count.

### Examples

```bash
# Board summary with top connected nets
python list_nets.py board.kicad_pcb

# Detect differential pairs
python list_nets.py board.kicad_pcb --diff-pairs

# Show power/ground nets
python list_nets.py board.kicad_pcb --power

# Full board analysis
python list_nets.py board.kicad_pcb --diff-pairs --power

# List all nets on U2A
python list_nets.py board.kicad_pcb --component U2A

# Show pad-to-net assignments
python list_nets.py board.kicad_pcb --component U2A --pads

# List only DATA nets on a component
python list_nets.py board.kicad_pcb --component U2A --pattern "*DATA*"
```

### Output Examples

Board summary (no options):
```
Board: board.kicad_pcb
Total nets: 174
Total components: 25

Top 10 most-connected nets:
  GND: 42 pads
  VCC: 23 pads
  ...
```

Differential pairs (`--diff-pairs`):
```
Differential Pairs (5 found):
  LVDS_CLK_P  /  LVDS_CLK_N
  LVDS_DATA0_P  /  LVDS_DATA0_N
  ...
```

Power/ground nets (`--power`):
```
Ground Nets (2 found):
  GND: 42 pads
  AGND: 8 pads

Power Nets (3 found):
  VCC: 23 pads
  +3.3V: 15 pads
  +1.8V: 6 pads
```

Component nets (`--component U2A`):
```
Nets on U2A (42 total):
  Net-(U2A-DATA_0)
  Net-(U2A-DATA_1)
  ...
  GND
  VCC
```

With `--pads`:
```
Pads on U2A (64 pads):
  A1: Net-(U2A-DATA_0)
  A2: Net-(U2A-DATA_1)
  A3: GND
  ...
```

## BGA Fanout Generator (`bga_fanout.py`)

Generates escape routing for BGA packages with support for differential pairs.

### Usage

```bash
python bga_fanout.py input.kicad_pcb --component REFERENCE --output output.kicad_pcb [OPTIONS]

Options:
  --component, -c     Component reference (auto-detect BGA if not specified)
  --output, -o        Output PCB file (default: kicad_files/fanout_test.kicad_pcb)
  --layers, -l        Routing layers (default: F.Cu B.Cu)
  --track-width, -w   Track width in mm (default: 0.3)
  --clearance         Track clearance in mm (default: 0.25)
  --via-size          Via outer diameter in mm (default: 0.5)
  --via-drill         Via drill size in mm (default: 0.3)
  --nets, -n          Net patterns to include
  --diff-pairs, -d    Differential pair patterns (e.g., "*lvds*")
  --diff-pair-gap     Gap between P/N traces in mm (default: 0.1)
  --exit-margin       Distance past the BGA boundary the escape stubs extend to, in mm
                      (default: 0.5)
  --primary-escape, -p  Primary escape direction: horizontal or vertical (default: horizontal).
                      Pairs try this direction first, then switch if channels are full
  --force-escape-direction  Only use the primary escape direction; do not fall back to the
                      secondary direction
  --rebalance-escape  Rebalance escape directions after initial assignment: pairs near the
                      secondary edge but far from the primary edge are reassigned to the
                      secondary direction for more even distribution
  --check-for-previous  Skip pads that already have fanouts (also avoids occupied channels)
  --no-inner-top-layer  Prevent inner pads from using F.Cu
  --grid-step         Routing grid step in mm (default: 0.1). Escape stub ends
                      are snapped to this grid so the router gets on-grid
                      terminals; MATCH the --grid-step you pass to route.py
  --escape-method     auto (default), channel, underpad, or dogbone.
                      "channel" is the 45-stub + channel router; "underpad"
                      vias each ball in its pad and routes under the pad
                      field (dense arrays), working outside-in: outer rings
                      first escape as deep as possible on the pad layer, and
                      only the balls that cannot reach the edge via out;
                      "dogbone" places the classic offset via on the channel
                      diagonal beside each ball (opt-in, human-style; never
                      chosen by auto); "auto" runs channel and, if it drops
                      any ball, retries with underpad and keeps whichever
                      escapes more
  --layer-costs       One value per --layers entry, matching route.py semantics:
                      negative = forbidden (no escape copper there, e.g. a
                      soon-to-be-plane inner layer), otherwise a weight in
                      [1.0, 1000] - cheaper layers are filled first and costly
                      layers keep only overflow escapes
  --fab-tier {standard,advanced}  JLC fab capability floor (default: standard)
  --fab-overrides FILE  Fab-floor override file overlaying the selected --fab-tier
                      (see [Fab Tier Options](configuration.md#fab-tier-options))
```

Escape stub ends are snapped to the `--grid-step` grid, and the decorative
end-jog is validated against the full obstacle map (foreign pads + tracks +
vias): a jog that would extend into a foreign obstacle is dropped, leaving an
on-grid stub end the router can launch from (issue #149).

### Example

```bash
# Generate fanout for LVDS differential pairs on IC1
python bga_fanout.py board.kicad_pcb --component IC1 --output fanout.kicad_pcb \
    --nets "*lvds*" --diff-pairs "*lvds*" --primary-escape vertical

# Generate fanout for data nets on U3 with 4 layers
python bga_fanout.py board.kicad_pcb --component U3 --output fanout.kicad_pcb \
    --nets "*DATA*" --layers F.Cu In1.Cu In2.Cu B.Cu
```

### What It Does

Creates escape routing from BGA pads through channels between pad rows:

```
Before:        After:
  o o o          o─┐ o─┐ o─┐
  o o o    =>    o─┘ o─┘ o─┘
  o o o          o── o── o──
```

See [bga_fanout/README.md](../bga_fanout/README.md) for detailed documentation.

## QFN/QFP Fanout Generator (`qfn_fanout.py`)

Generates stub tracks for QFN/QFP package fanout.

### Usage

```bash
python qfn_fanout.py input.kicad_pcb --output output.kicad_pcb --component REFERENCE [OPTIONS]

Options:
  --component, -c     Component reference (auto-detected if not specified)
  --output, -o        Output PCB file
  --layer, -l         Routing layer (default: the layer the component is mounted on)
  --width, -w         Track width in mm
  --extension         Extension past pad edge before bend (mm)
  --clearance         Min clearance to other-net pads in mm; stubs that would
                      graze a foreign pad are shortened or dropped
  --nets, -n          Net patterns to include
  --grid-step         Routing grid step in mm (default: 0.1). Fanned stub ends
                      are snapped to this grid so the router gets on-grid
                      terminals; MATCH the --grid-step you pass to route.py
  --escape-method     stub (default) = surface 45-degree fan; underpad = drop a
                      through-via just past each pad and escape on an inner layer
                      (issue #164), for crowded fine-pitch edges. With --nets,
                      underpad also escapes interior (under-body) pads; unscoped
                      runs never via-drop the exposed/thermal pad (issue #410)
  --via-size          Underpad escape via outer diameter in mm (default: 0.45)
  --via-drill         Underpad escape via drill diameter in mm (default: 0.25)
  --board-edge-clearance  Min clearance from stub/via copper to the Edge.Cuts
                      outline in mm (default 0 = use --clearance)
  --allow-via-in-pad  Underpad escape: let the escape via overlap its OWN pad
  --fab-tier          JLC fab capability floor: standard (default) or advanced
  --fab-overrides FILE  Fab-floor override file overlaying the selected --fab-tier
```

Fanned stub ends are snapped to the `--grid-step` grid, and the end-jog is
validated against the full obstacle map (foreign pads + tracks + vias): if it
would extend into a foreign obstacle the fan is shortened instead (issue #149).

### Example

```bash
# Generate fanout stubs for all signal nets on U3
python qfn_fanout.py board.kicad_pcb fanout.kicad_pcb U3
```

## Build Script (`build_router.py`)

Builds the Rust router module.

### Usage

```bash
python build_router.py [OPTIONS]

Options:
  --clean        Remove all Rust build outputs and compiled library files
  --from-source  Skip the prebuilt download and build locally with cargo
  --tag TAG      Download from a specific release tag (e.g. v0.15.0) instead of
                 the latest release
```

### What It Does

By default it downloads a prebuilt binary for the current platform from the
GitHub Release and installs it, verifying the version. With `--from-source` it
skips the download and builds locally with `cargo` instead. Either way it copies
the resulting library to the correct location:
   - Windows: `grid_router.pyd`
   - Linux/Mac: `grid_router.so`

### Requirements

- Rust toolchain installed (`rustup`)
- PyO3 and maturin for Python bindings

## PCB Geometry Extractor (`extract_pcb_geometry.py`)

Extracts PCB geometry data into a simple JSON format for analysis or debugging.

### Usage

```bash
python extract_pcb_geometry.py input.kicad_pcb [output.json] [OPTIONS]

Options:
  --output FILE    Output JSON file
  --nets PATTERN   Filter nets by pattern
  --summary        Print summary statistics only
```

### Example

```bash
# Extract all geometry to JSON
python extract_pcb_geometry.py board.kicad_pcb geometry.json

# Extract specific nets
python extract_pcb_geometry.py board.kicad_pcb --nets "*lvds*" --output lvds.json

# Print summary without writing file
python extract_pcb_geometry.py board.kicad_pcb --summary
```

### Output Format

```json
{
  "nets": {
    "1": {"name": "GND", "segments": 45, "vias": 12},
    "2": {"name": "VCC", "segments": 23, "vias": 8},
    ...
  },
  "components": [
    {"reference": "U2A", "footprint": "BGA-256", "pads": 256},
    ...
  ],
  "layers": ["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"],
  "board_outline": {"min_x": 0, "min_y": 0, "max_x": 100, "max_y": 80}
}
```

## Test Scripts

### Full Fanout and Route Test (`test_fanout_and_route.py`)

Runs a complete integration test: fanout generation followed by routing and verification.

#### Usage

```bash
# Run all stages
python test_fanout_and_route.py --all

# Quick mode (reduced scope)
python test_fanout_and_route.py --all --quick

# Run specific stages only
python test_fanout_and_route.py --fanout --ftdi
python test_fanout_and_route.py --ram --planes

# Run only checks (no routing)
python test_fanout_and_route.py --onlychecks --ftdi --lvds --ram
```

#### Options

| Option | Description |
|--------|-------------|
| `--all` | Run all stages: fanout, ftdi, lvds, ram, planes, and checks |
| `--quick` | Run quick test with reduced routing (fewer nets per stage) |
| `--fanout` | Run fanout tests (QFN and BGA fanout) |
| `--ftdi` | Run FTDI single-ended routing tests |
| `--lvds` | Run LVDS differential pair routing tests |
| `--ram` | Run DDR RAM routing tests |
| `--planes` | Run power/ground plane routing tests |
| `--checks` | Run DRC and connectivity checks after routing |
| `--onlychecks` | Run only checks, skip all routing stages |
| `-u, --unbuffered` | Run python commands with unbuffered output |

Note: By default, no stages run. Use `--all` to run everything, or specify individual stages.

This script runs a sequence of fanout and routing operations on the test board, including:
1. QFN fanout for U2 (ADC interface)
2. BGA fanout for multiple components (FTDI, ADC, FPGA, DDR)
3. Single-ended routing (FTDI data lanes)
4. Differential pair routing (LVDS)
5. DDR routing (DQS/CK diff pairs + DQ data lanes)
6. DRC and connectivity verification

#### Cross-Platform

This Python script replaces the previous PowerShell script and works on any platform (Linux, macOS, Windows).

## Cycle Checker (`check_cycles.py`)

Robustly detects **redundant copper loops (cycles)** per net. A routed signal net
should be a **tree**; rip-reroute / failed-edge retry and overlapping fanout +
routed copper can close loops (the "crazy" wandering traces and via stacks seen
on dense boards). This reports the **cyclomatic number** (independent loops =
`E - V + components`) per net, plus same-net **overlapping via pads**.

The connectivity graph is geometry-correct (this is what makes it robust, and
what a naive shared-endpoint graph misses):

- tolerance endpoint matching (size-aware for vias/pads),
- **T-junction** detection — an endpoint landing on another segment's *interior*
  splits that segment, but only when the cluster actually has copper on that
  layer (no false cross-layer junctions),
- via / through-hole-pad **layer joins** (a via connects all copper at its spot).

Copper pour / zone nets (planes) are meshes by design and are hidden unless
`--all`. A non-zero loop count means redundant copper (a detour loop and/or
fanout/route overlapping on the same line). Overlapping via pads (centre distance
< sum of radii) are flagged as redundant via stacks.

### Usage

```bash
python3 check_cycles.py board.kicad_pcb [OPTIONS]

Options:
  --net NAME       Check one net by exact name (always shown, even if clean)
  --nets PATTERN   Glob of net names to check (e.g. 'RAM_*')
  --all            Also list zoned/plane nets (meshes, normally hidden)
  --verbose        Also list the overlapping via-pad coordinates / per-net detail
```

Exits non-zero if any signal net has loops or overlapping vias (usable in CI).

### Examples

```bash
# Whole board: list every signal net with loops or overlapping vias
python3 check_cycles.py routed.kicad_pcb

# Focus one net (the wandering loop you saw in the layout)
python3 check_cycles.py routed.kicad_pcb --net RAM_D8

# All RAM data lines, with via-overlap coordinates
python3 check_cycles.py routed.kicad_pcb --nets 'RAM_D*' --verbose
```

### Output

```
net                           segs  vias  loops  comp  ovlVias
--------------------------------------------------------------
RAM_LDM                         28     4     13     1        1  <== LOOPS  <== 1 OVERLAPPING VIA PAIR(S)
...
signal nets with loops: 116; total independent loops: 615; nets with overlapping vias: 11
```

`loops` is the cyclomatic number (0 = tree). `comp` is the number of connected
copper components for the net (should be 1 for a fully-routed net). The
`prune_redundant_cycles` finalization pass in `route.py` removes the simple
shared-endpoint loops; loops that remain here are typically **overlapping
collinear copper** (a fanout stub and the route on top of it) or **via stacks**,
which that pass does not yet handle.

## Copper Hygiene Checker (`check_weird.py`)

Read-only scan for **weird copper** a routed board should not have: dangling
trace ends and tails, near-open **soft joints** (same-net segments that overlap
by their end caps instead of meeting endpoint-to-endpoint), redundant copper
**loops/cycles**, **removable** segments (copper that can be deleted without
disconnecting the net), **stacked** duplicate copper, and **floating** vias
(vias touching no copper on any layer). It never modifies the board — use it as
a triage pass before or after the other checkers.

### Usage

```bash
python3 check_weird.py board.kicad_pcb [OPTIONS]

Options:
  --nets, -n PATTERN [PATTERN ...]  Net name patterns to check (fnmatch wildcards,
                                    e.g. "*lvds*"). Default: all nets
  --thorough            Run the removable-segment scan on nets with >500 segments too
                        (slow — off by default so big power/plane nets don't dominate)
  --tolerance FLOAT     Minimum finding size in mm (dangle/tail length, gap,
                        duplicated-copper length, via diameter); smaller findings are
                        dropped. Default: 0.1; use 0 to report everything
  --max-print INT       Max findings printed per category (<=0 prints all; default 20)
```

### Examples

```bash
# Whole board, default 0.1mm tolerance
python3 check_weird.py routed.kicad_pcb

# Only the LVDS nets, report every finding regardless of size
python3 check_weird.py routed.kicad_pcb --nets "*lvds*" --tolerance 0

# Include the slow removable-segment scan on large nets
python3 check_weird.py routed.kicad_pcb --thorough
```

## Non-Orthonormal Segment Checker (`check_orthonormal.py`)

Flags track segments that run at an angle other than a multiple of 45° **and**
are longer than about one grid cell. An on-grid router emits only 0/45/90°
segments; the one legitimate non-orthonormal segment is the **short** terminal
connector that joins an on-grid path to an off-grid pad/ball/fanout-stub end
(≤ ~1 grid cell). Any *longer* skewed segment is a routing defect — a terminal
connector or merged terminal that spanned many cells and can cut straight
across foreign copper (issues #157/#159). This finds them directly in the
output, independent of DRC.

### Usage

```bash
python3 check_orthonormal.py board.kicad_pcb [OPTIONS]

Options:
  --grid-step FLOAT     Router grid step in mm (default: 0.05)
  --max-len FLOAT       Flag non-orthonormal segments longer than this (mm).
                        Default: 5 × grid-step (0.25mm at 0.05 grid) — clears the
                        legitimate ≤~0.15mm bga_fanout stub-end jogs
  --angle-tol FLOAT     Degrees off a 45° multiple still considered orthonormal
                        (default: 1.0, absorbs float noise)
  --max-list INT        Max segments to list (default: 40)
  --include-stub-pads   Also flag skewed segments that terminate at a pad (by default
                        these bga/qfn fanout stubs are excluded)
  --pad-tol FLOAT       Distance in mm for an endpoint to count as on a pad (default: 0.06)
```

### Examples

```bash
# Default (0.05mm grid)
python3 check_orthonormal.py routed.kicad_pcb

# Match a 0.1mm routing grid
python3 check_orthonormal.py routed.kicad_pcb --grid-step 0.1
```

## Parser-Parity Validator (`validate_pcb_data.py`)

Headless twin of the GUI About-tab "Validate PCB Data" button: loads the board
with pcbnew, builds `PCBData` from the live board objects (the GUI plugin's
path), text-parses the same file with `parse_kicad_pcb` (the CLI path), and
diffs the two models with `compare_pcb_data`. Any difference means one of the
two parsers mis-models the board (pad geometry, arc tracks, zones, bounds,
...) and the CLI and GUI would route against different worlds.

```bash
python3 validate_pcb_data.py board.kicad_pcb [more.kicad_pcb ...]
```

Needs the pcbnew Python module; when run with a plain `python3` it re-execs
itself into KiCad's bundled interpreter automatically. Exit 0 = all boards
match, 1 = differences, 2 = error. The stress-test flows run it per board
(RUNBOOK rule 1b; `redo_stress_test.py` validates the final board after each
replay, `--skip-validate` to opt out).

## DRC Settings Fixer (`fix_kicad_drc_settings.py`)

Rewrites a board's **project file** (`.kicad_pro`) so that a manual DRC in the
KiCad GUI shows only the routing-relevant errors instead of stock-default noise,
making KiCad's enforced constraints consistent with the floors the board was
actually routed to (issue #160).

KiCad stores DRC *design rules* and *violation severities* in the `.kicad_pro`,
not in the `.kicad_pcb`. A freshly written board gets a project with KiCad's
defaults, which produce noise in two ways:

1. **Constraint floors stricter than the routed board** — stock `min_clearance`
   0.2 mm, `min_via_diameter` 0.45 mm, `min_track_width` 0.2 mm,
   `min_hole_clearance` 0.25 mm — fire on every track/via/drill the router placed
   below them (hundreds of markers). They are spurious at the real manufacturing
   floor; `check_drc.py` never reports them.
2. **Non-routing categories** — courtyard overlaps, solder-mask bridges and
   footprint/library markers (`annular_width`, `lib_footprint_*`) the router
   neither creates nor fixes — often dominate the report (e.g. ~200 annular +
   ~150 library markers on the orangecrab stress board).

The script sets the relevant **Constraints / Net Classes** to the per-object
minima the board uses — copper `min_clearance` (+ Default net-class clearance),
`min_hole_to_hole`, `min_hole_clearance`, `min_copper_edge_clearance`, and the
min track / via / drill / annular sizes — sets the courtyard / solder-mask /
footprint severities to `ignore`, and demotes `starved_thermal` (thermal-relief
spoke shortfall) from error to a **warning** (`--keep-thermal` keeps it an error).
For the **size** floors (track / via / drill) it uses the **smaller** of the
routing param you pass and the smallest such object actually on the board, so a
later coarse step (say a 0.3 mm repair pass) can't raise the floor above 0.127 mm
tracks that earlier steps left — the floor always sits at or below the smallest
object physically present. **Clearance / hole-to-hole / edge** can't be read back
from the board (a spacing isn't stored per object, and the *achieved* spacing may
dip below the intended value at real violations, which we must not mask), so they
come from the routing param; across a chain they accumulate the **minimum** used
in any step via the only-loosen rule as each step threads its `.kicad_pro` to the
next. Clearance defaults to the project's Default net-class clearance when not
passed. Pass the routing parameters (`--clearance`, `--hole-to-hole`,
`--edge-clearance`, `--track-width`, `--via-size`, `--via-drill` — the same values
you gave `route.py`) to pin them exactly.

If the project has **no** Default net class (a bare/stub project), the script
seeds a *complete* one before lowering its clearance — KiCad silently ignores a
sparse net class and falls back to the stock 0.2 mm clearance, so an incomplete
class would leave the board flagging 0.2 mm clearance everywhere.

**Only loosen, never tighten.** Every constraint is set to `min(current, target)`
— lowered toward the fab floor but never raised — so the fix can never introduce
a new violation or silently strengthen a rule you set looser. `clearance` shorts
and `unconnected_items` are left untouched (KiCad shows unconnected items in
their own tab).

**KiCad-version safe.** The script edits only the `.kicad_pro` (JSON); it never
touches the `.kicad_pcb`, so a KiCad-9 board (`version 20241229`) stays
KiCad-9 — unlike a `pcbnew`/GUI round-trip, which upgrades the board to the
running KiCad's format on save. (One caveat: copper **zones** are not refilled by
`kicad-cli pcb drc`, so a stale GND pour can still show "zone clearance" markers
on the command line; KiCad's *interactive* DRC refills zones first, so those
do not appear in the GUI.)

> **Close the board in KiCad before running.** KiCad keeps the project in memory
> and overwrites an externally-edited `.kicad_pro` on save/close, reverting the
> fix. Run the script with the board closed, then reopen.

### Usage

```bash
python3 fix_kicad_drc_settings.py board.kicad_pcb [OPTIONS]

Options:
  --clearance MM        Copper clearance floor (default: project Default net-class clearance)
  --hole-clearance MM   Hole/copper clearance floor (default: the copper clearance)
  --hole-to-hole MM     Hole-to-hole clearance floor (routing --hole-to-hole-clearance)
  --edge-clearance MM   Copper-to-edge clearance floor (routing --board-edge-clearance)
  --track-width MM      Min track width (default: smallest track on the board)
  --via-size MM         Min via diameter (default: smallest via on the board)
  --via-drill MM        Min hole/drill diameter (default: smallest drill on the board)
  --keep-courtyards     Do not ignore the courtyard categories
  --keep-mask           Do not ignore solder_mask_bridge
  --keep-footprint      Do not ignore footprint/library categories
                        (annular_width, lib_footprint_issues, lib_footprint_mismatch)
  --keep-thermal        Keep starved_thermal an error (default: demote to warning)
  --ignore CAT [CAT...] Additional severity categories to set to "ignore"
  --ignore-warnings     Set EVERY category currently at "warning" severity to
                        "ignore" (hides all warning markers; errors untouched)
  --dry-run             Print what would change without writing
```

The board must already have a sibling `.kicad_pro` (any board opened or saved in
KiCad has one). If it is missing, open the board in KiCad once to generate it,
then re-run. The script is idempotent and accepts either the `.kicad_pcb` or the
`.kicad_pro` path.

### Runs automatically after routing

You normally don't run this by hand: **`route.py`, `route_diff.py`,
`route_planes.py`, and `route_disconnected_planes.py` invoke it as their final
step** (issue #160), pinning the floors to the clearances/sizes they just routed
with, so the written project is DRC-consistent by default. If the output is a new
file with no project yet, they copy the input board's `.kicad_pro` (or seed a
complete one when the input has none). Pass `--no-fix-drc-settings` to skip it, or
`--keep-thermal` to leave `starved_thermal` at its original severity instead of
demoting it to a warning (all four routing CLIs accept both flags).

When routing used an explicit `--clearance` ceiling, the writeback also **clamps**
each NON-Default net class' clearance/track/via floor DOWN to the routed value
(#439), so KiCad grades the copper at what was actually routed rather than at the
(usually aspirational) stock class. When `--clearance` was omitted the classes are
preserved (each net routed at its own class). There is no separate flag — the
`--clearance` ceiling is the switch; in the GUI, checking the **Min Clearance**
override box is the equivalent (unchecked = honor classes, checked = clamp).

The **GUI plugin** does the equivalent on the live board via the pcbnew API
(`BOARD_DESIGN_SETTINGS` + the Default net class + severities) after routing, and
marks the board modified so your next save keeps it. A single **"Fix DRC settings
after routing"** checkbox on the **Basic tab** controls this for every routing
action in the dialog — single-ended routing, differential pairs, and plane
create/repair all read that one shared toggle (it is on by default); a **"Keep
thermal-relief DRC severity"** checkbox on the **Advanced tab** is the GUI
counterpart of `--keep-thermal` (off by default). Both front-ends share the same
target-computing logic (`compute_targets` / `severity_plan` in
`fix_kicad_drc_settings.py`) and differ only in how they apply it (`.kicad_pro`
file vs. pcbnew API).

### Examples

```bash
# Default: derive floors from the board's own minima + project clearance; ignore
# courtyard, solder-mask and footprint/library (annular_width, lib_footprint_*) noise
python3 fix_kicad_drc_settings.py routed.kicad_pcb

# Pin every floor to the routing parameters you gave route.py (recommended)
python3 fix_kicad_drc_settings.py routed.kicad_pcb \
    --clearance 0.15 --track-width 0.15 --via-size 0.4 --via-drill 0.3 \
    --hole-to-hole 0.2 --edge-clearance 0.0

# Preview without writing
python3 fix_kicad_drc_settings.py routed.kicad_pcb --dry-run

# Pin the hole-clearance floor explicitly
python3 fix_kicad_drc_settings.py routed.kicad_pcb --hole-clearance 0.1

# Also silence every warning-severity marker (track_dangling, silk_*, etc.)
python3 fix_kicad_drc_settings.py routed.kicad_pcb --ignore-warnings

# Keep courtyard checks, ignore only the mask bridges plus one extra category
python3 fix_kicad_drc_settings.py routed.kicad_pcb --keep-courtyards --ignore starved_thermal
```

### Output

Prints each change (`min_hole_clearance: 0.25 -> 0.0889 mm`,
`severity[courtyards_overlap]: error -> ignore`, …) and a reminder to reopen the
board. On a typical dense BGA board this turns a ~300-violation DRC into the few
dozen genuine routing errors (clearance + shorts + real sub-floor hole clearance).

## Fab Tier Floors (`fab_tiers.py`)

A shared library — not a stand-alone command — that defines the JLCPCB
manufacturing **floor ladder** every routing/DRC CLI shrinks tracks, vias and
clearances *down toward*. It is stdlib-only so the lightweight DRC-settings
script can import it without the PCB parser, and it exposes the two flags
(`--fab-tier` / `--fab-overrides`) that `route.py`, `route_diff.py`,
`route_planes.py`, `route_disconnected_planes.py`, `bga_fanout.py`,
`qfn_fanout.py`, `check_drc.py`, `fix_kicad_drc_settings.py` and `list_nets.py`
all add through its `add_fab_tier_args()` helper (so the flag is identical
everywhere).

- **`--fab-tier standard`** (default) — the cheap, no-extra-cost floor. Routing
  prefers it but **auto-escalates to `advanced` (printing a one-line warning)**
  when a fine-pitch fan-out genuinely cannot escape at the standard floor.
- **`--fab-tier advanced`** — JLC's tighter, "more costly" floor (0.25 via /
  0.15 drill, 0.09–0.10 mm track/clearance). A **hard** floor: no escalation.

`--fab-overrides FILE` overlays the selected tier with a plain, human-editable
`key = value` file — only the floor values listed change; the rest come from the
base tier. Supplying one **disables escalation** (the floor becomes exactly
*base tier + file*), since the file states your exact fab limits. Keys are
`track_width`, `clearance`, `via_diameter`, `via_drill`, `hole_to_hole`,
`pad_hole_to_hole`, `annular`; `#` starts a comment; values are in mm and must
be > 0 (unknown/invalid lines are warned and skipped). A fully-commented
template listing every key and the built-in tier values ships as
[`fab_overrides.example.txt`](../fab_overrides.example.txt) in the repo root.

```bash
# Route to the cheap floor (default); dense fan-outs warn when they escalate
python3 route.py in.kicad_pcb out.kicad_pcb --nets "Net*"

# Opt the whole board into the tighter, more-costly floor
python3 route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-tier advanced

# Declare your own fab capability (pins the floor, no escalation)
python3 route.py in.kicad_pcb out.kicad_pcb --nets "Net*" --fab-overrides my_fab.txt
```

See [Fab Tier Options](configuration.md#fab-tier-options) for the full floor
tables and how the CLIs enforce these floors (they **error** if a size/clearance
param is set below the active floor).

## Common Workflows

### Route and Verify

```bash
# Route the nets
python route.py input.kicad_pcb routed.kicad_pcb "Net-*" --ordering mps

# Check for DRC violations
python check_drc.py routed.kicad_pcb

# Verify connectivity (all routed nets)
python check_connected.py routed.kicad_pcb

# Or verify connectivity for a specific component
python check_connected.py routed.kicad_pcb --component U1
```

### BGA Differential Pair Flow

```bash
# 1. Generate fanout stubs
python bga_fanout.py board.kicad_pcb --component U2A --output fanout.kicad_pcb \
    --nets "*lvds*" --diff-pairs "*lvds*"

# 2. Route differential pairs
python route.py fanout.kicad_pcb routed.kicad_pcb --nets "*lvds*" \
    --diff-pairs "*lvds*" \
    --ordering inside_out

# 3. Verify results
python check_drc.py routed.kicad_pcb
python check_connected.py routed.kicad_pcb --nets "*lvds*"
```

### Debug Routing Issues

```bash
# List nets on problematic component
python list_nets.py board.kicad_pcb U2A --pads

# Route with debug lines enabled
python route.py input.kicad_pcb debug.kicad_pcb "Net-(*)" --debug-lines

# Open debug.kicad_pcb in KiCad, check User.3/4/5/6/8/9 layers
```

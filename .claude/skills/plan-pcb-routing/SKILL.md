---
name: plan-pcb-routing
description: Analyzes a KiCad PCB file and creates a comprehensive routing plan. Examines components for fanout needs (BGA/QFN/QFP/PGA), identifies differential pairs, categorizes power/ground nets, and presents a step-by-step routing workflow with explanations.
---

# Plan PCB Routing

When this skill is invoked with a KiCad PCB file, perform a comprehensive analysis and present a routing plan to the user.

## Step 1: Load and Analyze PCB Structure

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')

# Basic stats
print(f'Total nets: {len(pcb.nets)}')
print(f'Total footprints: {len(pcb.footprints)}')
print(f'Existing segments: {len(pcb.segments)}')
print(f'Existing vias: {len(pcb.vias)}')
```

Report to user:
- Number of nets, components, existing routing
- Whether this is a fresh board or partially routed

## Step 2: Identify Copper Layers

Check the KiCad file directly for layer definitions:

```bash
grep -E "^\s+\([0-9]+ \".*\.Cu\"" path/to/file.kicad_pcb
```

Report to user:
- Available copper layers (F.Cu, B.Cu, In1.Cu, In2.Cu, etc.)
- Whether it's a 2-layer, 4-layer, or multi-layer board

### Stackup Check (always run this early)

Inspect the stackup now, before planning, and report the verdict **at the top of the
plan report** so problems surface before any routing work:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
for layer in pcb.board_info.stackup:  # List[StackupLayer], ordered top to bottom
    print(layer.name, layer.layer_type, layer.thickness, layer.epsilon_r)
```

- No stackup section, or all dielectrics with identical thickness and ε_r ≈ 4.5, means
  KiCad's untouched default. If the board also has impedance-relevant signals (see the
  speed detection in Step 4), lead the report with a clear warning: impedance and
  time-matching calculations will not match the user's fab, and `/recommend-stackup`
  should be run before impedance-controlled routing. Take plane-layer assignments from
  its output when available.
- A 2-layer board with multiple differential pairs or planes-worth of power nets is
  itself worth flagging (no inner layers for reference planes).
- If the stackup looks deliberate, say so in one line and move on.

Report problems prominently but still produce the full plan - the user decides whether
to fix the stackup first.

## Step 3: Check for Components Needing Fanout

Identify BGA, QFN, QFP, PGA, and other array packages that benefit from escape routing:

```python
for ref, fp in pcb.footprints.items():
    name_upper = fp.footprint_name.upper()
    pad_count = len(fp.pads)

    # Check for array packages
    needs_fanout = any([
        'BGA' in name_upper,
        'PGA' in name_upper,
        'QFN' in name_upper,
        'QFP' in name_upper,
        'LQFP' in name_upper,
        'TQFP' in name_upper,
    ])

    # Also flag high pin-count components
    if pad_count > 40:
        needs_fanout = True

    if needs_fanout:
        # Analyze pad arrangement
        xs = sorted(set(round(p.local_x, 2) for p in fp.pads))
        ys = sorted(set(round(p.local_y, 2) for p in fp.pads))
        grid_cols, grid_rows = len(xs), len(ys)

        # Check SMD vs through-hole
        smd_count = sum(1 for p in fp.pads if p.drill == 0)
        th_count = sum(1 for p in fp.pads if p.drill > 0)
```

### Fanout Tool Selection

| Package Type | Tool | Notes |
|--------------|------|-------|
| BGA (SMD grid) | `bga_fanout.py` | Escape routing for ball grid arrays |
| PGA (through-hole grid) | `bga_fanout.py` | Same tool works for PGA |
| QFN/QFP (perimeter SMD) | `qfn_fanout.py` | Stub routing for quad flat packages |
| DIP/SOIC (through-hole/SMD rows) | None needed | Standard routing handles these |

### When to Use Fanout for BGA/PGA

**Rule: Use fanout for any BGA/PGA with more than 2 pins depth from outside to center.**

**Important:** Calculate ACTUAL depth by counting pads from the edge toward center, not grid size.
Many PGA/BGA packages (especially FPGAs/CPLDs) have hollow centers with only perimeter pins populated.

To calculate actual depth:
```python
# Check middle column from top edge toward center
mid_col = xs[len(xs)//2]
depth = 0
for y in ys:  # ys sorted from edge
    if (mid_col, y) in pad_positions:
        depth += 1
    else:
        break  # Stop at first empty position
```

Examples:
- 13×13 grid, fully populated → depth = 7 → **USE FANOUT**
- 13×13 grid, hollow center (3 rows populated) → depth = 3 → **USE FANOUT**
- 10×10 grid, hollow center (2 rows populated) → depth = 2 → fanout optional
- 4×4 grid, fully populated → depth = 2 → fanout optional

Inner pins beyond depth 2 cannot escape without fanout routing through channels between outer pins.

**Escape layers (multi-layer boards):** `bga_fanout.py` defaults to `--layers F.Cu B.Cu`
only. On a 4+ layer board, pass ALL the board's copper layers, e.g.
`--layers F.Cu In1.Cu In2.Cu B.Cu` — otherwise deep balls have nowhere to escape to
and those nets are dropped from the fanout. `qfn_fanout.py` is perimeter-only and
doesn't take escape layers.

**Always check the fanout escaped all requested balls.** `bga_fanout.py` ends with
`JSON_SUMMARY: {"component", "requested", "escaped", "failed", "unescaped_nets", ...}`.
A dropped ball is **removed from the output** and later fails signal routing as "no
rippable blockers", so it must be caught here. If `failed > 0`, retry the fanout with
more layers and/or a smaller `--clearance` (see "Escape clearance" below) before
moving on — do not start signal routing while balls are still dropped.

**If balls still drop on a dense, fully-populated array, switch to the under-pad
escape:** add `--escape-method underpad` with a small via/track for the pitch
(e.g. `--via-size 0.35 --track-width 0.12 --clearance 0.1` at 0.8 mm pitch). The
default `channel` engine confines every layer to the gaps *between* ball rows, so
a few channels over-subscribe and the deepest balls can't escape; `underpad`
routes each ball *under* the pad field on inner layers via a via-in-pad and
escapes arrays `channel` can't (e.g. a 22×22 BGA that drops ~20 balls → 0).
Caveats: it routes diff pairs as **single-ended**, and it **skips power/plane
nets** (they tap their plane), so create the planes first (or exclude power with
`--nets`). Rule of thumb: try `channel` first (keeps diff pairs); fall back to
`underpad` when `channel` can't escape a dense array.

Report to user:
- List of components that may need fanout
- Package type, pad count, and grid depth for each
- Recommended fanout tool

## Step 4: Check for Differential Pairs and Power Nets

Use `list_nets.py` to detect differential pairs and power/ground nets:

```bash
python3 list_nets.py path/to/file.kicad_pcb --diff-pairs --power
```

### Read the board's design rules and pass them to the CLI

The router does NOT read the board's design rules — it falls back to a generic
`--clearance 0.25` / `--track-width` default, which is often WIDER than the
board's own rule and can box pads in so nets fail with "no rippable blockers".
Read the board's real rules and pass them explicitly:

```bash
python3 list_nets.py path/to/file.kicad_pcb --design-rules
```

**KiCad has TWO tiers of rules, and DRC only enforces one of them — this matters
for fine-pitch boards (#111/#115):**

- **Net-class values** (`clearance`, `track_width`, `via_diameter`, `via_drill`):
  these are the size new objects are *drawn at*. Of these, only **clearance** is
  a DRC-enforced minimum. `track_width` and `via_diameter`/`drill` are **not** DRC
  floors — they are just defaults, so a board can (and the human originals do) use
  a **smaller** via/track than the net-class nominal and still pass DRC.
- **Board Constraints** (`min_clearance`, `min_track_width`, `min_via_diameter`,
  `min_hole_to_hole`, `min_through_hole_diameter`): **these are the actual DRC
  floors.** `--design-rules` reads them from `design_settings.rules` and combines
  them with the JLCPCB fab minimum (backstop when a Constraint is 0/unset — e.g.
  `min_clearance` is frequently 0) into a single **manufacturing floor**.

Use the printed flags as-is:

- **Routing** (`route.py`, `qfn_fanout.py`, `bga_fanout.py`, `route_planes.py`):
  `--clearance` from the **Default class**, but **`--via-size`/`--via-drill`
  from the working floor**, NOT the net-class `via_diameter`. Emitting the net-class
  via everywhere is #115 — it's a max-like default, far too big for fine-pitch
  escape (e.g. a 0.4 mm QFN/BGA needs the small working via the original used).
  For `--track-width`, the net-class value is only a starting point and is *not* a
  hard minimum: on dense/congested boards route ordinary signals at the **fab
  physical floor** instead (thinner is both more complete and faster — see "Route
  signals at the FAB floor by default" in Diagnose and Retry). Keep the net-class
  width only for current-carrying nets (`--power-nets`) and impedance-controlled
  nets.
- **Escape clearance — trigger on dropped balls, not pitch (issue #122):** the
  inter-ball channel is too narrow to fit a track at the net-class clearance on
  more BGAs than just "fine-pitch" ones. Even an **0.8 mm-pitch** BGA drops balls
  at `--clearance 0.2` (the ~0.45 mm gap between 0.35 mm balls can't fit a 0.2 mm
  track at 0.2 mm clearance) — the same board escapes **all** balls at the 0.1 mm
  floor. So don't gate on pitch: gate on whether balls actually dropped.
  `bga_fanout.py` ends with a `JSON_SUMMARY: {...}` line giving
  `requested`/`escaped`/`failed`/`unescaped_nets`. **After every fanout, parse it;
  if `failed > 0` (escaped < requested), re-run the fanout with `--clearance` at
  the manufacturing floor** (never below it — the floor is the rule the human
  board passes DRC against, so tightening board-wide is manufacturable and needs
  no rule-area settings). If still short, also try the smaller **fine-pitch escape
  via** (below) and/or a narrower `--track-width` toward the floor. Do not proceed
  to signal routing with `failed > 0` unexpected — those balls are dropped from the
  output and will fail later as "no rippable blockers".
- **Fine-pitch escape VIA (4+ layer):** the 0.45 mm standard via can't dog-bone /
  via-in-pad sub-~0.5 mm-pitch BGA/QFN balls. For *those parts only*, pass the
  smaller **fine-pitch escape via** that `--design-rules` prints (`fine-pitch
  escape via <d>/<drill>`, e.g. `0.30/0.15` — JLC "advanced", small extra cost)
  as `--via-size`/`--via-drill` to that part's `bga_fanout.py` / `qfn_fanout.py`,
  to `route_diff.py` when it launches from that part's escaped stubs, **and to
  `route_disconnected_planes.py`** (its per-pad repair connects the fine-pitch
  GND/power plane balls under such parts). Keep the **standard** working via for
  general `route.py` routing and the bulk `route_planes.py` pour — the advanced
  via is escape-only, not a board-wide default (issues #99/#122).
- **Non-Default classes:** route those nets separately with that class's
  `--clearance`/`--track-width` (clearance is the one per-class DRC value, so keep
  each class's nets at their own clearance rather than forcing one global value).
- **Diff pairs:** `--track-width`/`--diff-pair-gap` from the Default class for `route_diff.py`.

**Verification (DRC/connectivity) grades at the manufacturing floor**, not the
inflated net-class clearance — that is the same rule the human original passes, so
it's the honest delta. Use the printed `check_drc.py` flags
(`--clearance <floor> --hole-to-hole-clearance <floor>`); see Step 6.

Only fall back to tool defaults when neither net classes nor Constraints are found
(`--design-rules` then prints the JLCPCB fab floor for the board's layer count).

This will output:
- Differential pairs detected (P/N naming conventions)
- Ground nets with pad counts
- Power nets with pad counts

If differential pairs are found:
- List each P/N pair
- Note that `route_diff.py` should be used for these
- Explain that diff pairs maintain consistent spacing and length matching
- **If a pair's pads are on a BGA/PGA being fanned out, escape it with
  `bga_fanout.py` too** — pass `--diff-pairs "<patterns>" --diff-pair-gap <gap>`
  so P and N escape the array together on one layer. Don't just exclude the
  pair from fanout and hand it to `route_diff.py`: it can't launch from the
  deep balls ("no valid position at any setback"). `route_diff.py` then
  connects the escaped stubs — **but on a 4+ layer board you must pass those
  inner layers to `route_diff.py` via `--layers` too** (it defaults to F.Cu
  B.Cu, so an inner-layer escaped stub is otherwise unreachable and the pair is
  silently dropped — issue #116). Pairs not on an array package don't need fanout.

> **Tip:** Name-based detection misses pairs with unconventional names. For boards with
> high-speed ICs (PHYs, SerDes, USB, FPGA transceivers), or when detection finds suspiciously
> few pairs, run `/identify-diff-pairs` for datasheet-based detection by pin function and
> per-interface gap/impedance recommendations.

Also note: `route_diff.py` resolves P/N polarity mismatches automatically, which can swap
target pad net assignments. Swaps are reported in the output — when they happen, the
schematic sync step below applies (see "Schematic Synchronization After Swaps").

**Far-apart terminal pads → single-ended follow-up (issue #121).** A "diff pair"
sometimes has pads that aren't a coupled connection — e.g. a P and an N test point
several mm apart, or a logical pair daisy-chained through spread-out parts. If the
coupled chain can't be routed, `route_diff.py` peels those far-apart pads off the
chain (routing the genuinely-coupled terminals as a pair) and lists the affected
nets under `single_ended_followup_nets` in its `JSON_SUMMARY` (and a "route them
single-ended next" block on stdout). Those pads are **not** dropped — the **Signal
Routing** step (`route.py "*" "!GND" "!VCC"`) connects them P→P / N→N along with
every other unrouted net, since they remain unrouted after the diff-pair step. So:
**do not exclude the diff-pair nets from the signal-routing step's net selection** —
that step is what finishes the peeled pads. If you scope the signal step to specific
nets instead of `"*"`, add any `single_ended_followup_nets` to it explicitly.

### Check for DDR/High-Speed Memory Signals

Look for DDR signal patterns in the net list that may need length matching:
- Data signals: DQ0-DQ63
- Strobes: DQS, DQM, DM
- Clocks: CLK, CK

If DDR signals detected:
- Note that `--length-match-group auto` should be used
- DQ0-7 + DQS0 form byte lane 0, DQ8-15 + DQS1 form byte lane 1, etc.

Report to user:
- List of detected differential pairs (or "none found")
- Whether `route_diff.py` is needed
- Whether DDR/length-matching is needed

### High-Speed Signal Check (delegate to /find-high-speed-nets)

Whether the plan includes GND return vias - and the `--gnd-via-distance` to use -
is the `/find-high-speed-nets` skill's job: it classifies nets into speed tiers
(datasheet lookup, rise-time estimates) and maps tiers to recommended distances.
Follow that skill's methodology here (its quick net-name/footprint scan decides
whether the deeper datasheet pass is worth it) and put the recommended distance
into the plan's GND-via step. Remember its physical floor: never set
`--gnd-via-distance` below 3 x (via_size + clearance), ~2.5 mm for standard vias.

Report to user when presenting the plan:
- If high-speed nets found: "**GND Return Vias:** This board has [tier] signals ([examples]).
  GND return vias are included in Step N with `--gnd-via-distance [X]mm`. Let me know if
  you'd like to skip this step."
- If no high-speed nets found: "**GND Return Vias:** No high-speed signals detected (only
  low-frequency I2C/UART/GPIO). GND return vias are included in the plan but are optional
  for this board. Want me to remove the step?"

## Step 5: Review Power and Ground Net Strategy (delegate to /recommend-plane-mappings)

Which nets deserve planes and on which copper layers is the
`/recommend-plane-mappings` skill's job: it weighs pad counts and datasheet
current estimates, and assigns layers with SI rationale (GND adjacent to signal
layers for return paths, power planes paired against GND, split layers for
multiple rails). Follow its methodology here, seeded by the `list_nets.py --power`
output, and put the resulting net -> layer assignments into the plan's
`route_planes` steps. Nets it leaves to wide traces become `--power-nets` /
`--power-nets-widths` on the route step instead.

Report to user:
- Identified GND nets and pad counts
- Identified power nets and pad counts
- Recommended strategy (plane vs wide traces) with layer assignments

## Step 5b: Net-Coverage Reconciliation (mandatory — do not skip)

The stages partition every routable net by glob pattern, and the patterns are
**not** reconciled automatically. The failure mode this step prevents: a net is
*excluded* from one stage (`!X`) but never *claimed* by a later one, so it
silently gets zero copper and the run "completes" with it fully unrouted. This
is exactly how `GNDA` (an analog ground tied to `GND` through a single 0Ω/
ferrite) was dropped — excluded from the signal route as a "power net", yet never
added to the plane step's `--nets`, ending with 0/23 pads connected while the run
reported success.

**The invariant: every routable net (≥2 pads, not no-connect) must be claimed by
exactly one stage. A net excluded from any stage MUST be claimed by a later one.**

Before running any command, write the net-handling ledger and reconcile it
mechanically — do not eyeball it:

1. **Assign every routable net to one handler:**
   - `fanout + signal route` — ordinary signals (the `"*"` selection minus exclusions)
   - `diff-pair route` — detected pairs
   - `plane / pour` — every net you exclude from the signal route with `!X`
   - `wide trace` — power carried *inside* the route selection via `--power-nets` (NOT excluded)

2. **Diff the two pattern lists.** The set of signal-route exclusions (`!A !B …`)
   MUST be identical to the set of nets the plane step pours (`--nets A B …`).
   A net in the symmetric difference is a plan bug — it is excluded from routing
   but not poured (→ it will be unrouted), or poured but not excluded (→ routed as
   tracks, defeating the plane). Print both and assert the difference is empty:
   ```python
   route_exclusions = {"GND", "+3V3"}   # the !X you will pass route.py
   plane_nets       = {"GND", "+3V3"}   # the --nets you will pass route_planes.py
   orphans = route_exclusions ^ plane_nets       # symmetric difference
   assert not orphans, f"Net-coverage gap: {sorted(orphans)} handled by no stage"
   ```
   Do not proceed until `orphans` is empty.

3. **Secondary grounds / split rails** (`AGND`, `GNDA`, `DGND`, `VREF`, or any rail
   tied to its parent through a single 0Ω resistor or ferrite bead — find the tie
   with `list_nets.py`: the part with one pad on each net). These are real,
   separate nets. Pour each as **its own local region** (Voronoi-sharing an inner
   layer with the main ground is fine) and let the single tie component join it to
   the parent. **Never** merge it into the parent plane (that shorts the split and
   defeats its purpose — a green connectivity check then hides an electrical error)
   and **never** leave it out (that leaves it unrouted). Give each its own `--nets`
   entry in the plane step, so it appears in BOTH lists in step 2 above.

## Step 6: Generate Routing Plan

Based on the analysis, generate a step-by-step plan. The general order is:

### Routing Order Rationale

1. **Fanout** (if needed) - Escape routing first, while the board is empty. Exclude
   nets that planes will handle (`"*" "!GND" "!VCC"`).
2. **Differential Pairs** - The most constrained routes claim their channels before
   anything else can block them (if present). May peel far-apart "terminal" pads
   (e.g. spread-out test points) off the coupled chain and leave them for the
   signal-routing step (reported as `single_ended_followup_nets`, issue #121).
3. **Signal Routing** - All remaining nets, **excluding the plane nets**
   (`--nets "*" "!GND" "!VCC"`). Routing them as tracks now would defeat the
   planes step - the exclusions are mandatory whenever a later step gives those
   nets planes. This step also finishes any diff-pair pads peeled off in step 2,
   so keep the diff-pair nets in its selection (the `"*"` covers them).
4. **Power Planes** - Create GND and VCC planes together. Stitching vias adapt
   around the routed signals; the reverse is not true - a stitching via placed
   early can block the only clean channel for a diff pair (issue #56). If signal
   tracks boxed in a power pad, add `--rip-blocker-nets` so the blockers are
   ripped and rerouted.
5. **GND Return Vias** - Add return current vias near signal vias (when GND planes
   present); folds into the planes call with `--add-gnd-vias`.
6. **Plane Repair** - Reconnect any broken plane regions
7. **Verification** - DRC and connectivity checks

### Example Plan Output Format

Present the plan to the user as a numbered list with explanations:

```
## Routing Plan for board.kicad_pcb

### Board Summary
- 2-layer board (F.Cu, B.Cu)
- 174 nets, 25 components
- Unrouted (0 existing traces)

### Components Requiring Special Handling
- **U9 (PGA120)**: 120-pin grid array - use bga_fanout.py for signals only

### Differential Pairs
- None detected

### Power/Ground Nets
- **GND**: 42 pads - use plane on B.Cu
- **VCC**: 23 pads - use plane on F.Cu (or wide traces if planes not desired)

---

## Step-by-Step Routing Commands

### Step 1: Fanout U9 (PGA120) - All Non-Plane Nets
Generates escape routing for ALL nets on the component EXCEPT those that the
planes step will handle. This ensures every signal net gets fanned out,
avoiding `--no-bga-zone` workarounds during routing.

**Important:** Use `"*" "!GND" "!VCC"` to fan out all nets except the power
plane nets. Do NOT use `"/*"` alone, as it misses nets with non-hierarchical
names like `Net-(U9-Pad1)` which would then require `--no-bga-zone` to route.

On a 4+ layer board also pass every copper layer with `--layers` (default is
F.Cu B.Cu only) so inner balls can escape — drop `--layers` only for true
2-layer boards.

python3 -X utf8 bga_fanout.py board.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" "!VCC" \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --output board_step1.kicad_pcb \
    2>&1 | tee /tmp/step1_fanout.txt

**Then check the `JSON_SUMMARY` line: if `failed > 0`, balls were dropped — retry
before continuing.** First confirm all copper layers are passed; then re-run with
`--clearance` at the manufacturing floor (e.g. `--clearance 0.1`), which fixes the
common case (an 0.8 mm-pitch BGA can't fit a track between balls at 0.2 mm). If still
short, add the fine-pitch escape via and/or a smaller `--track-width`. Only proceed
to Step 2 once `failed == 0` (or the remaining `unescaped_nets` are understood and
accepted).

### Step 2: Route All Signal Nets (excluding plane nets)
Routes all remaining unrouted nets EXCEPT the nets that get planes in the
next step - the `"!GND" "!VCC"` exclusions are mandatory here, otherwise the
power nets get routed as ordinary tracks and the planes step has nothing to
do. Routing signals before planes means the plane stitching vias (placed
next) adapt around the signals instead of blocking them.

For boards with BGA/PGA components, use `--no-bga-zone` to allow the router
to find alternative paths through the dense pin area (even when fanout was
done, some paths may require this). Use `--max-ripup 10
--max-iterations 1000000` for difficult 2-layer boards.

python3 -X utf8 route.py board_step1.kicad_pcb board_step2.kicad_pcb \
    --nets "*" "!GND" "!VCC" \
    --no-bga-zone \
    --max-ripup 10 \
    --max-iterations 1000000 \
    2>&1 | tee /tmp/step2_routing.txt

### Step 3: Create Power Planes (GND and VCC) + GND Return Vias
Creates power planes in a single call, after signal routing so the stitching
vias find spots around the finished tracks. Each net is paired with its
corresponding layer (GND→B.Cu, VCC→F.Cu). Through-hole PGA/BGA pads
automatically connect to planes on their layer; SMD pads get vias routed to
the plane. `--add-gnd-vias` also places return-current vias near the signal
vias that now exist. If signal tracks boxed in a power pad, add
`--rip-blocker-nets` to rip and re-route the blockers.

> **Note to user:** GND return vias improve signal integrity for high-speed
> signals. Based on the speed analysis, this board has [speed_tier] signals,
> so `--gnd-via-distance` is set to [X] mm. If this is a purely low-frequency
> board (I2C/UART/GPIO only), drop `--add-gnd-vias`. Let me know if you'd
> like that.

python3 -X utf8 route_planes.py board_step2.kicad_pcb board_step4.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    --add-gnd-vias --gnd-via-distance 2.0 \
    2>&1 | tee /tmp/step3_planes.txt

Adjust `--gnd-via-distance` based on the board's highest signal speed:
- Ultra-high (>1 GHz): 2.0 mm
- High (100 MHz - 1 GHz): 3.0 mm
- Medium (10 - 100 MHz): 5.0 mm
- Minimum physical limit: 3 x (via_size + clearance)

### Step 5: Repair Disconnected Plane Regions
Signal traces and GND return vias may have cut through planes. This step
reconnects any isolated copper islands AND repairs pad-level plane connections.
Pass the **same signal parameters as Step 3** (clearance/via/track-width/grid,
plus `--power-nets`/`--power-nets-widths` and `--no-bga-zone` if Step 3 used it)
so that any nets ripped to clear a blocked pad re-route correctly. Enable
`--rip-blocker-nets --reroute-ripped-nets`: a plane-net pad that can't reach its
plane (e.g. a tiny connector GND pin blocked by a signal trace) is then connected
by tracing to an adjacent same-net pad, ripping the blocker and re-routing it
(restoring any net that can't re-route). These map to the plugin's Planes repair
tab "Rip up blocking nets" / "Auto-reroute ripped nets" checkboxes.

python3 -X utf8 route_disconnected_planes.py board_step4.kicad_pcb board_step5.kicad_pcb \
    --clearance <floor> --via-size <V> --via-drill <D> --track-width <signal_track> --grid-step <G> \
    --rip-blocker-nets --reroute-ripped-nets \
    --power-nets <PWR...> --power-nets-widths <W...> [--no-bga-zone] \
    2>&1 | tee /tmp/step5_plane_repair.txt

### Step 6: Verify Results
Invoke `/review-routed-board board_step5.kicad_pcb` for the full review (DRC,
connectivity, orphan stubs, length-match tolerances, GND return via coverage,
diff pair checks). If that skill is unavailable, run the raw checks — DRC at the
**manufacturing floor** from Step 4's `--design-rules` output (the
`check_drc.py` flags it printed), NOT a hardcoded 0.25, so legitimately-tight
fine-pitch escapes that are still fabbable don't read as violations (#111):

python3 -X utf8 check_drc.py board_step5.kicad_pcb --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee /tmp/step6_drc.txt
python3 -X utf8 check_connected.py board_step5.kicad_pcb 2>&1 | tee /tmp/step6_connectivity.txt
python3 -X utf8 check_orphan_stubs.py board_step5.kicad_pcb 2>&1 | tee /tmp/step6_orphans.txt
```

**Coverage gate (mandatory — close the loop on Step 5b).** `check_connected.py`
already lists every net with ≥2 pads but no copper and no covering zone as
"Unrouted net with N pads" (it accounts for plane zones and ignores genuine
single-pad / no-connect nets). After planes + repair, **this unrouted list must
be empty** except for entries you can individually justify in writing (true
single-pad nets, deliberate no-connects). A fully-unrouted multi-pad net is a
coverage defect, NOT a shortfall to report-and-accept: it means a net fell
through the stage partition (Step 5b). For each one, go back and handle it —
route it, or add it to the plane step (a secondary ground gets its own pour
region per Step 5b) — then re-verify. Do not declare the board done while the
list has unjustified entries.

### Alternative: VCC as Wide Traces (No Plane)

If you prefer not to use a VCC plane, route VCC with wide traces instead:

```
### Step 1 (Alternative): Fanout U9 Including VCC
python3 -X utf8 bga_fanout.py board.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" \
    --output board_step1.kicad_pcb

### Step 2 (Alternative): Route Signals + VCC as Wide Traces
python3 -X utf8 route.py board_step1.kicad_pcb board_step2.kicad_pcb \
    --nets "*" "!GND" \
    --power-nets VCC --power-nets-widths 0.5
```

Only GND keeps its exclusion (it still gets a plane in Step 3, now with
`--nets GND --plane-layers B.Cu` only). If VCC wasn't fanned out, add
`--no-bga-zone U9` to allow router access.

## Step 7: Check for High-Speed Signal Requirements

### Length Matching (DDR, high-speed buses)

For DDR memory or other length-matched buses, detect signals that need matching:

```python
# Common DDR signal patterns
ddr_patterns = ['DQ', 'DQS', 'DQM', 'DM', 'CLK', 'CK', 'CAS', 'RAS', 'WE', 'CS', 'ODT', 'CKE']
ddr_nets = [n.name for n in pcb.nets.values()
            if n.name and any(p in n.name.upper() for p in ddr_patterns)]
```

If DDR or length-matched signals detected, add to the plan:
- `--length-match-group auto` for automatic DDR byte lane grouping
- `--length-match-tolerance 0.1` for acceptable variance (mm)
- `--time-matching` if routes span different layers (accounts for dielectric)

### Impedance-Controlled Routing

For high-speed signals with impedance requirements:
- `--impedance 50` for 50Ω single-ended (calculates width per layer from stackup)
- `--impedance 100` with `route_diff.py` for 100Ω differential

### Bus Detection

For parallel data/address buses with clustered endpoints:
- `--bus` enables automatic bus detection and parallel routing
- Routes are attracted to neighbors, creating clean parallel traces

## Step 8: Handle Special Cases

### 2-Layer Board with Dense Components

On 2-layer boards, BGA/PGA fanout may fail for some inner pins due to
insufficient routing channels. Options:
- Accept partial fanout; router will complete remaining connections
- Skip fanout entirely; direct routing often works for through-hole PGA

**Important:** If you skip fanout for a BGA/PGA component but still need to connect its
internal pads, use `--no-bga-zone <component>` to disable the automatic exclusion zone
and allow the router to enter the dense pin area:

```bash
python3 route.py board.kicad_pcb \
    --nets "*" \
    --no-bga-zone U9 \
    --output board_routed.kicad_pcb
```

Without this flag, the router auto-detects BGA/PGA zones and avoids them, which would
leave internal pads unconnected if they weren't fanned out.

### Multi-Layer Boards (4+ layers)

- Use inner layers for planes (In1.Cu for GND, In2.Cu for VCC)
- More fanout options available
- Use `--layer-costs` to prefer certain layers:
  ```bash
  --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 5.0 5.0 1.0
  ```
  Higher cost = layer is avoided (used only when necessary)

### Differential Pairs Present

Insert diff pair routing after fanout but before single-ended signals:

```bash
python3 route_diff.py board.kicad_pcb \
    --nets "*LVDS*" "*USB*" \
    --diff-pair-gap 0.15 \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --output board_diff.kicad_pcb
```

**Escape layers (multi-layer boards):** like `bga_fanout.py`, `route_diff.py`
defaults to `--layers F.Cu B.Cu` only. On a 4+ layer board you MUST pass every
copper layer — when a pair was escaped by `bga_fanout.py` onto an INNER layer,
`route_diff.py` can only launch from those escaped stubs if that inner layer is
in `--layers`. Omitting it strands the inner-layer stubs and silently drops
those pairs (you'll see a low routed-pair count, e.g. 8/40 instead of 22/40 —
issue #116). Use the same copper-layer list you passed to `bga_fanout.py`; drop
`--layers` only for true 2-layer boards.

Key options:
- `--diff-pair-gap 0.1` - Gap between P and N traces (mm)
- `--no-gnd-vias` - Disable automatic GND via placement near signal vias
- `--diff-pair-intra-match` - Match P/N lengths within each pair
- `--swappable-nets "*rx*"` - Allow target swap optimization for memory lanes

### QFN/QFP Components (Perimeter Pads)

Use `qfn_fanout.py` instead of `bga_fanout.py`:

```bash
python3 qfn_fanout.py board.kicad_pcb \
    --component U1 \
    --output board_qfn.kicad_pcb
```

Creates two-segment stubs (straight + 45° fan) for each pad.

### Power Net Width Options

Instead of routing power separately, use `--power-nets` with signal routing:

```bash
python3 route.py board.kicad_pcb \
    --nets "*" \
    --power-nets "GND" "VCC" "+3.3V" \
    --power-nets-widths 0.5 0.4 0.4 \
    --output board_routed.kicad_pcb
```

First matching pattern determines width. Useful when not using planes.

### Target Swap Optimization (Memory Routing)

For swappable signals (e.g., memory data lanes where any DQ can connect to any):

```bash
python3 route.py board.kicad_pcb \
    --nets "*DQ*" \
    --swappable-nets "*DQ*" \
    --output board_routed.kicad_pcb
```

Uses Hungarian algorithm to find optimal assignments minimizing crossings.

### Schematic Synchronization After Swaps

When routing performs polarity swaps (P↔N) or target swaps, the schematic can get
out of sync with the PCB. Use `--schematic-dir` to automatically update:

```bash
python3 route_diff.py board.kicad_pcb \
    --nets "*LVDS*" \
    --swappable-nets "*LVDS*" \
    --schematic-dir /path/to/kicad/project \
    --output board_routed.kicad_pcb
```

This updates the `.kicad_sch` files with any pad swaps made during routing.

**Important:** After routing with swaps, ask the user:
> "The router performed X polarity swaps and Y target swaps. Would you like to
> update the schematic to match? If so, provide the path to your KiCad project
> directory and I'll re-run with `--schematic-dir`."

Schematic sync is **disabled by default** to avoid unexpected changes. Only enable
when the user confirms they want schematic updates.

### Guide Corridors (user-drawn preferred routes)

When specific nets keep taking bad paths (or the user wants control over where a bundle
runs), the user can draw a polyline on `User.1` in KiCad and re-route those nets with:

```bash
python3 route.py board.kicad_pcb --nets "SPI*" --guide-corridor --output board_routed.kicad_pcb
```

The route follows the line as waypoints, strictly best-effort — a guide never makes a route
fail or adds vias. See `docs/configuration.md` "Guide Corridor Options" for details.

**Scope rule: do NOT draw guide corridor geometry yourself.** Suggest *in words* where a
corridor would help ("a line on User.1 south of J3, between the mounting hole and C14") and
let the user draw it; then incorporate `--guide-corridor` into the plan.

### Keepout Zones (RF / analog exclusions)

Check the board for components that warrant routing exclusions: antennas (footprint/value
keywords ANT, ANTENNA, chip antenna parts), RF modules, and sensitive analog front-ends. If
found, recommend the user draw closed polygon(s) on `User.2` around those regions and add
`--keepout` to every routing step (`route.py`, `route_diff.py`) so tracks and vias stay out
on all copper layers. Same scope rule as guide corridors: describe where the keepout should
go; the user draws it.

### MPS Layer Swap (crossing conflicts)

When MPS ordering reports crossing conflicts (nets in Round 2+), or failures show pairs of
nets repeatedly ripping each other up, add `--mps-layer-swap` to attempt layer swaps that
eliminate same-layer crossings before routing begins.

### Vertical Track Alignment

On 4+ layer boards where through-hole components need via space, `--vertical-attraction-radius`
/ `--vertical-attraction-cost` attract tracks on different layers to stack vertically,
consolidating routing corridors.

### Plane Via Placement Options (route_planes.py)

- Multiple nets can share one plane layer (Voronoi partitioning): `--nets GND VCC --plane-layers In2.Cu In2.Cu`
- `--same-net-pad-clearance <mm>` forces plane vias outside same-net pads with that edge-to-edge clearance (default places at pad center when possible)
- `--rip-blocker-nets` rips up interfering routed nets to maximize via placement, then re-routes them

### Net Ordering Strategies

| Strategy | Flag | Best For |
|----------|------|----------|
| MPS (default) | `--ordering mps` | General routing, minimizes crossings |
| Inside-Out | `--ordering inside_out` | BGA escape routing |
| Original | `--ordering original` | Manual control |

### Useful Utility Scripts

| Script | Purpose |
|--------|---------|
| `list_nets.py U1` | List all nets connected to a component |
| `list_nets.py U1 --pads` | Show pad-to-net assignments |
| `check_orphan_stubs.py` | Find traces ending without connection |

### Debug and Visualization Options

When routing fails or behaves unexpectedly:

```bash
# Verbose output with diagnostic info
python3 route.py board.kicad_pcb --nets "*" --verbose --output board_debug.kicad_pcb

# Debug geometry on User layers (visible in KiCad)
python3 route.py board.kicad_pcb --nets "*" --debug-lines --output board_debug.kicad_pcb

# Real-time visualization (requires pygame-ce)
python3 route.py board.kicad_pcb --nets "*" --visualize --output board_debug.kicad_pcb

# A* search statistics
python3 route.py board.kicad_pcb --nets "*" --stats --output board_debug.kicad_pcb
```

### Post-Routing Enhancements

```bash
# Add teardrop settings to all pads (improves manufacturability)
python3 route.py board.kicad_pcb --nets "*" --add-teardrops --output board_routed.kicad_pcb
```

### Advanced Routing Parameters

For difficult boards, consider tuning these parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--max-ripup 3` | 3 | Max blocking nets to rip up and retry |
| `--max-iterations 200000` | 200000 | A* iteration limit per route |
| `--heuristic-weight 1.9` | 1.9 | >1 = faster but may miss tight routes, 1.0 = optimal |
| `--via-cost 50` | 50 | Higher = fewer vias, longer paths; lower (10-25) for BGA escape |
| `--grid-step 0.1` | 0.1 | Smaller = finer routing but slower; 0.05 for fine-pitch |

Manufacturing constraints (set to match your fab's requirements):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--clearance 0.25` | 0.25 | Track-to-track clearance (mm) |
| `--board-edge-clearance 0.5` | 0 | Min distance from board edge (mm) |
| `--hole-to-hole-clearance 0.2` | 0.2 | Min drill-to-drill spacing (mm) |

### Proximity Penalties

For dense boards, use proximity penalties to spread out routes:

```bash
python3 route.py board.kicad_pcb --nets "*" \
    --stub-proximity-radius 2.0 --stub-proximity-cost 0.2 \
    --bga-proximity-radius 7.0 --bga-proximity-cost 0.2 \
    --track-proximity-distance 2.0 --track-proximity-cost 0.1 \
    --output board_routed.kicad_pcb
```

## Important Notes

0. **Net-coverage invariant (Step 5b)** - Every routable net must be claimed by exactly one stage; a net excluded from one stage (`!X`) MUST appear in a later stage's selection. Reconcile the route-exclusion set against the plane `--nets` set before routing (symmetric difference empty), and confirm `check_connected.py`'s unrouted list is empty at the end. This is the guard against a net (e.g. a secondary ground like GNDA) being silently dropped by every stage.
1. **Always check for GND connections** - If a component has GND pads but GND isn't being fanned out, the plane vias will handle it
2. **Fanout ALL non-plane nets** - Use `--nets "*" "!GND" "!VCC"` to fan out all nets except those handled by planes. Do NOT use `"/*"` alone as it misses nets with non-hierarchical names like `Net-(U9-Pad1)`. Unconnected nets are automatically filtered out.
3. **Order matters** - Fanout, then diff pairs, then signals (always excluding plane nets with `"!GND" "!VCC"` exclusions), then planes + GND return vias, then repair. Signals route first because stitching vias can relocate around tracks, but a diff pair cannot relocate around a badly placed via
4. **Verify at the end** - Always run DRC, connectivity, and orphan stub checks
5. **Consider the analyze-power-nets skill** - For complex boards where power net identification isn't obvious, use that skill first to analyze component datasheets
6. **Consider the find-high-speed-nets skill** - For accurate GND return via distance recommendations based on actual component datasheet speeds and rise times, run `/find-high-speed-nets` before planning. The lightweight inline analysis (Step 4) uses net name patterns only.
7. **Stub layer switching is on by default** - The router automatically moves stubs to eliminate vias when beneficial; disable with `--no-stub-layer-swap`
8. **Default layer costs** - 2-layer boards default to F.Cu=1.0, B.Cu=3.0 to prefer top layer; 4+ layer boards use 1.0 for all
9. **Schematic sync is disabled by default** - After routing with swaps, offer to re-run with `--schematic-dir` if the user wants to update their schematic
10. **Rip-up and reroute is automatic** - When a route fails, the router automatically rips up blocking nets and retries (up to `--max-ripup` blockers)
11. **Component shortcut** - Use `--component U1` to route all signal nets on a component (auto-excludes GND/VCC/unconnected)
12. **Use --no-bga-zone for difficult boards** - Even when fanout is complete, use `--no-bga-zone` during routing to allow the router to find alternative paths through the dense pin area. This is especially important for 2-layer boards where routing channels are limited.
13. **Windows UTF-8 encoding** - On Windows, use `python3 -X utf8` to avoid Unicode encoding errors when scripts print special characters (like Ω for resistance). Example: `python3 -X utf8 route_planes.py ...`
14. **BGA/PGA power pins and planes** - When using power planes, BGA/PGA power pins (GND, VCC) connect most efficiently via direct vias to the plane rather than fanout routing. Create planes first, then fanout only signal nets. Through-hole PGA pads automatically connect to planes on that layer; SMD BGA pads need vias placed by `route_planes.py`. This approach:
    - Reduces routing congestion (power pins don't consume escape channels)
    - Provides lower impedance power connections
15. **Aggressive parameters for 2-layer BGA/PGA boards** - Use `--max-ripup 10 --max-iterations 1000000` from the start for boards with dense components. These parameters help resolve routing conflicts that would otherwise fail.
16. **Guide corridors and keepouts are user-drawn** - Never draw `User.1` guide polylines or `User.2` keepout polygons yourself; suggest in words where they should go and let the user draw them, then add `--guide-corridor` / `--keepout` to the plan.
17. **Companion skills** - Defer to `/identify-diff-pairs` (datasheet-based pair detection), `/recommend-stackup` (before impedance/time-matching work), `/diagnose-routing-failures` (after failures), and `/review-routed-board` (final verification) rather than duplicating their logic inline.

## Presenting the Plan

After generating the plan:
1. Show the board summary
2. Explain any special components found
3. List differential pairs if detected
4. Highlight any length-matching or impedance requirements
5. Present each step with the command AND a brief explanation of why
6. Ask the user if they want to proceed or modify the plan
7. Offer to run the commands if approved

## After Routing Completes

### Capture Logs for Analysis

Always capture command output to `/tmp` files for later analysis:

```bash
python3 -X utf8 route.py input.kicad_pcb output.kicad_pcb --nets "*" 2>&1 | tee /tmp/route_output.txt
python3 -X utf8 route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu 2>&1 | tee /tmp/planes_output.txt
python3 -X utf8 check_connected.py output.kicad_pcb 2>&1 | tee /tmp/connectivity.txt
python3 -X utf8 check_drc.py output.kicad_pcb --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee /tmp/drc.txt
```

(`<floor>` = the manufacturing floor from `list_nets.py --design-rules`, not the
0.2 default — grade DRC at the rule the board's own Constraints + fab capability
define, per #111/#115.)

### Parse Logs for Failure Analysis

After routing, parse the log files to understand failures:

```bash
# Check routing summary (last 20 lines usually have the summary)
tail -20 /tmp/route_output.txt

# Look for failed nets
grep -i "failed\|FAILED" /tmp/route_output.txt

# Check JSON summary for detailed failure info
grep "JSON_SUMMARY" /tmp/route_output.txt | sed 's/JSON_SUMMARY: //' | python -m json.tool

# Find specific failure reasons
grep -A5 "FAILED NET HISTORIES" /tmp/route_output.txt
```

The JSON_SUMMARY line contains structured data including:
- `failed_single`: List of failed single-ended net names
- `failed_multipoint`: List of nets with unconnected pads (includes pad coordinates)
- `multipoint_pads_connected` vs `multipoint_pads_total`: Connection success rate

### Diagnose and Retry

After running routing commands:
1. Report how many nets were routed successfully
2. **If routes failed**, invoke `/diagnose-routing-failures <board> <log files>` — it parses
   the JSON summary, failed-net histories, and blocking reports, correlates failures
   spatially, and outputs a targeted retry command. Apply its recommendation. If that skill
   is unavailable, fall back to this table:

| Failure Pattern | Likely Cause | Solution |
|-----------------|--------------|----------|
| "no rippable blockers found" | Route blocked by non-rippable obstacle | Use `--no-bga-zone`; if pads are "boxed in by static obstacles", shrink geometry / finer grid (see "Congestion escalation" below) |
| "Re-route FAILED: no path found" | Ripped net couldn't find new path | Increase `--max-iterations` |
| Many multipoint pads failed on same component | Congested area | Use `--max-ripup 10` or higher; shrink geometry toward the fab floor (see below) |
| Many failures cluster in one channel/region | Tracks too fat for the channel | **Congestion escalation**: re-route the failed nets at smaller track/via/clearance down to the fab floor (see below) |
| Routes near BGA boundary failing | BGA exclusion zone too aggressive | Use `--no-bga-zone` |

```bash
python3 -X utf8 route.py board_prev.kicad_pcb board_routed.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 10 \
    --max-iterations 1000000 \
    2>&1 | tee /tmp/route_retry.txt
```

   Key parameters for difficult boards (especially 2-layer with BGA/PGA):
   - `--no-bga-zone` - **Critical**: Allows router to enter BGA area for alternative paths
   - `--max-ripup 10` (default 3) - More rip-up attempts to resolve conflicts
   - `--max-iterations 1000000` (default 200000) - 5x more search iterations
   - `--stub-proximity-radius 10 --stub-proximity-cost 3.0` - Spread out fanout stubs (optional, for aesthetics)

#### Route signals at the FAB floor by default (thin is faster AND more complete)

**`track_width` and `via_diameter` are NOT DRC floors** (Step 4), and — this is
the subtlety — **the fab floor is NOT the board's `min_track_width` constraint
either.** Three different numbers get confused here; keep them straight:

- **Board `min_track_width`** (from `.kicad_pro`, e.g. ottercast = 0.2 mm) — the
  author's self-imposed DRC rule. Often conservative. Note `list_nets
  --design-rules` reports its "manufacturing floor" track as `max(this, JLC min)`,
  so it currently **clamps the track floor to this constraint** (0.2) and does NOT
  surface the finer fab capability — do not treat that printed track number as the
  real floor (it's right for clearance/via, just not for track).
- **Fab physical track minimum** (JLC ≈ **0.0889 mm / 3.5 mil** standard; **0.127
  mm / 5 mil** is the safe no-extra-cost width) — the actual floor. **This is the
  target.** It can be *below* the board's `min_track_width`: the human ottercast
  board routes most signals at 0.127 mm, under its own 0.2 mm constraint, which is
  exactly why it fits channels our 0.2 mm net-class tracks can't.

For ordinary signals there is **no benefit to routing fat** and a real cost.
Measured on ottercast_audio (signal pass, same clearance/grid, width only):

| Signal track width | Multipoint nets routed | Pads connected | Time |
|--------------------|------------------------|----------------|------|
| **0.127 (5 mil)**  | **122**                | **360/376**    | **2.69 s** |
| 0.15               | 118                    | 354/376        | 2.93 s |
| 0.20 (net-class)   | 103                    | 323/376        | 6.52 s |

Thinner is **monotonically better on both axes** — more nets complete *and* it
finishes faster (fat tracks cause ripup churn). So don't route fat and escalate;
**route the signal step at the fab floor from the start, and if still congested
go DOWN toward the fab physical minimum** (0.2 → 0.127 → 0.0889), not toward the
board's conservative `min_track_width`. There is no "knee" above the fab floor to
hunt for.

1. **Take the fab floor**, not the board constraint: the fab's physical track
   minimum (JLC 0.0889 mm / 3.5 mil; use 0.127 mm / 5 mil for a zero-cost,
   high-yield default). Going below the board's `min_track_width` is intended here
   — it's what the human did. (Keep DRC honest separately: grade at the clearance
   floor from `--design-rules`; a thinner track only *increases* clearance to
   neighbours, so it never creates a clearance violation.)
2. **Route the whole signal step at that width** (re-route everything, not just the
   failed nets — a victim is blocked by the *successful* wide tracks already in its
   channel, so thinning only the failures leaves the channel full):
   ```bash
   python3 -X utf8 route.py board_fanout.kicad_pcb board_signal.kicad_pcb \
       --nets "*" "!GND" "!VCC" \
       --track-width <fab floor, e.g. 0.127 or 0.0889> --clearance <floor, e.g. 0.1> \
       --via-size <floor via, e.g. 0.30> --via-drill <floor drill, e.g. 0.15> \
       --no-bga-zone --max-ripup 10 --max-iterations 1000000 \
       2>&1 | tee /tmp/route_signal.txt
   ```
   A finer `--grid-step` (0.05, or 0.025 for sub-0.4 mm pitch) is the complementary
   lever — a corridor that exists geometrically still needs a grid line on it to be
   found; pair it with the thin width at fine-pitch escapes ("boxed in by static
   obstacles"). If still congested, step the width down further toward the fab
   physical minimum and re-route.
3. **Keep only the nets that NEED width wide — by rule, not by sweep.**
   Power/high-current nets stay wide via `--power-nets`/`--power-nets-widths`, and
   impedance-controlled nets keep their calculated width (`--impedance`, or
   `route_diff.py` for pairs). Everything else routes at the fab floor. You do
   **not** need to find which signals are "genuinely congested": there's no reason
   to widen an ordinary signal at all, so the question never arises (and a net that
   passes wide can itself be the blocker of another, so a per-net width guess is
   unsound regardless).

3. **If swaps occurred** (polarity or target swaps):
   - Tell the user how many swaps were made
   - Ask if they want to sync the schematic
   - If yes, ask for the KiCad project directory path
   - Re-run the routing command with `--schematic-dir` added
4. Run verification: invoke `/review-routed-board` (falls back to the raw DRC and connectivity checks)
4b. **Apply the coverage gate (Step 6):** if `check_connected.py` lists any
   fully-unrouted multi-pad net, the board is NOT done — handle each (route or
   pour it) and re-verify before summarizing. Do not present an unrouted net as
   an accepted shortfall.
5. Summarize the final state of the board
6. **Offer to clean up intermediate files**:
   - List the intermediate `.kicad_pcb` files created (e.g., `board_step1.kicad_pcb`, `board_step2.kicad_pcb`, etc.)
   - Ask if the user wants to delete them, keeping only the final output
   - If yes, delete the intermediate files

Example cleanup prompt:
> "Routing complete. The following intermediate files were created:
> - board_step1.kicad_pcb (after GND/VCC planes)
> - board_step2.kicad_pcb (after fanout)
> - board_step3.kicad_pcb (after signal routing)
> - board_step4.kicad_pcb (after GND return vias)
>
> The final routed board is: board_step5.kicad_pcb
>
> Would you like me to delete the intermediate files?"

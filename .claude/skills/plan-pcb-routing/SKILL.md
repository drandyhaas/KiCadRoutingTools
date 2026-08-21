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

Identify BGA, QFN, QFP, PGA, LGA, and other array packages that benefit from escape routing:

```python
for ref, fp in pcb.footprints.items():
    name_upper = fp.footprint_name.upper()
    pad_count = len(fp.pads)

    # Check for array / fine-pitch land/no-lead packages by name. Note 'QFP'
    # already matches LQFP/TQFP/VQFP, 'QFN' matches VQFN/WQFN/HVQFN, and 'BGA'
    # matches FBGA/UFBGA/TFBGA, so only distinct families need listing.
    needs_fanout = any(k in name_upper for k in (
        'BGA',          # ball grid array
        'PGA',          # pin grid array (through-hole)
        'LGA',          # land grid array (interior lands, e.g. LGA-12) - issue #144
        'CSP', 'WLCSP', 'WLP',  # wafer-level / chip-scale = micro-BGA, sub-0.5mm
        'CGA',          # column grid array
        'QFN', 'DFN',   # quad / dual no-lead (exposed-pad)
        'QFP',          # quad flat pack
    ))

    # SMD vs through-hole FIRST -- it gates everything below (#513 item 16).
    smd_count = sum(1 for p in fp.pads if p.drill == 0)
    th_count = sum(1 for p in fp.pads if p.drill > 0)
    mostly_tht = th_count > smd_count

    # A THT part's pins are reachable on EVERY copper layer -- there is no
    # "escape" problem to solve, so fanout buys nothing regardless of pad
    # count. PLCC/DIP/ZIF SOCKETS are the trap: a PLCC-44 THT socket's
    # staggered double-ring reads as a sparse uniform grid and used to be
    # misdetected as a BGA (rc2014_82c55_ide U1 -- nets near it burned >1M
    # A* iterations each behind a phantom exclusion zone, #513 item 16).
    # Wide-pitch (>=2mm) PGAs route fine without fanout too; only reach for
    # bga_fanout on a PGA when its channels are genuinely contested.
    if mostly_tht and 'PGA' not in name_upper:
        needs_fanout = False

    # Fine-pitch arrays strand even at low pad count: trigger by PITCH + interior
    # pads, not just pad_count > 40 (issue #144: LGA-12 at 0.5mm has only 12 pads
    # but its center lands box in). Compute the min pad-to-pad spacing and whether
    # any pad is interior (not on the bounding-box edge).
    if not needs_fanout and not mostly_tht and pad_count >= 6:
        xs = sorted({round(p.local_x, 3) for p in fp.pads})
        ys = sorted({round(p.local_y, 3) for p in fp.pads})
        def _min_step(v):
            return min((b - a for a, b in zip(v, v[1:])), default=999)
        pitch = min(_min_step(xs), _min_step(ys))
        minx, maxx, miny, maxy = xs[0], xs[-1], ys[0], ys[-1]
        has_interior = any(minx < round(p.local_x, 3) < maxx and
                           miny < round(p.local_y, 3) < maxy for p in fp.pads)
        # Fine pitch (<=0.6mm) with interior pads, OR a large multi-row part
        # AT FINE PITCH. Raw pad_count > 40 alone is NOT a fanout signal: a
        # 44-pin THT socket, a 2x20 header, or a 1.27mm connector trips it
        # while gaining nothing from escape routing.
        if (pitch <= 0.6 and has_interior) or (pad_count > 40 and pitch <= 0.8):
            needs_fanout = True

    if needs_fanout:
        # Analyze pad arrangement
        xs = sorted(set(round(p.local_x, 2) for p in fp.pads))
        ys = sorted(set(round(p.local_y, 2) for p in fp.pads))
        grid_cols, grid_rows = len(xs), len(ys)
```

### Does this part actually BENEFIT from fanout? (check before planning it)

A name/pad-count match is a candidate, not a decision. Fanout (escape routing)
exists to solve ONE problem: pads that cannot be reached by ordinary routing
because neighboring pads at fine pitch box them in. Before adding a fanout
step, confirm the geometry actually has that problem:

1. **Through-hole part (most pads drilled)?** → **No fanout.** Every pin is
   reachable on every layer; there is nothing to escape. This includes
   PLCC/DIP/ZIF **sockets** (a PLCC-44 THT socket's staggered pin field looks
   like a sparse grid but is just a socket, #513 item 16), headers, and DIN /
   backplane connectors. Wide-pitch (>=2mm) PGAs also normally route fine
   without fanout.
2. **Wide-pitch SMD (>=1.27mm) perimeter part?** → No fanout; plain routing
   handles it.
3. **Interior pads at fine pitch (<=0.6mm), or a perimeter at <=0.65mm with
   many pads?** → Yes, fanout genuinely helps (this is the boxed-in case).
   Dense 2-row mezzanine/card-edge connectors at 0.4mm (CM4/CM5, 200+ pads)
   DO benefit -- use `qfn_fanout.py --escape-method underpad --allow-via-in-pad`.
4. **Unsure?** The fanout tools now refuse or warn on wrong shapes
   (staggered arrays, non-arrays). Trust a refusal: if the tool says the part
   is not an array and the geometry checks above say the pins are reachable,
   plan ordinary routing instead of forcing a workaround.

### Fanout Tool Selection

| Package Type | Tool | Notes |
|--------------|------|-------|
| BGA (SMD grid) | `bga_fanout.py` | Escape routing for ball grid arrays |
| PGA (through-hole grid) | `bga_fanout.py` | Same tool works for PGA |
| LGA / WLCSP / CGA (land/chip-scale grid) | `bga_fanout.py` | Grid escape; interior lands strand without it (issue #144) |
| QFN/QFP/DFN (perimeter SMD) | `qfn_fanout.py` | Stub routing for quad/dual no-lead and flat packages |
| **AQFN / staggered multi-row no-lead** | `qfn_fanout.py` **`--escape-method underpad --allow-via-in-pad`** | Inner rows the surface fan cannot reach - see below. **Never `bga_fanout.py`** |
| DIP/SOIC (through-hole/SMD rows) | None needed | Standard routing handles these |
| PLCC (SMD J-lead or THT socket) | None needed | Perimeter part; the THT socket's pins reach every layer. Never a BGA (#513 item 16) |
| Sockets / headers / backplane connectors (THT) | None needed | All-layer reachable; pad count alone is not a fanout signal |

### When to Use Fanout for BGA/PGA/LGA

**Rule: Use fanout for any grid array (BGA/PGA/LGA/WLCSP/CGA) with more than 2 pins
depth from outside to center, OR any fine-pitch (<=0.5mm) array with interior pads
regardless of pin count** — a small LGA-12/WLCSP at 0.5mm pitch boxes its center
lands in even though it has well under 40 pads (issue #144).

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

**Staggered multi-row no-lead packages (AQFN) - use via-in-pad (#500).** An
AQFN (e.g. `Nordic_AQFN-73-1EP_7x7mm_P0.5mm`, on osprey_kb / hex_gateway /
mikoto_nrf52840) puts its pads in TWO OR MORE staggered rows per side. The
surface 45-degree stub fan reaches only the outermost row, so the default
silently drops the rest. Measured on osprey_kb U1 (78 pads, 39 nets):

| command | escaped | time |
|---|---|---|
| `qfn_fanout.py` (default stub) | 26/40 | 2.4s |
| `qfn_fanout.py --escape-method underpad` | 35/40 | 2.6s |
| **`qfn_fanout.py --escape-method underpad --allow-via-in-pad`** | **39/39, DRC-clean** | **2.4s** |
| `bga_fanout.py` | 39/39 | **2967s** |

So: **for any AQFN or staggered multi-row no-lead part, plan
`qfn_fanout.py --escape-method underpad --allow-via-in-pad`.** Via-in-pad is
what reaches the innermost row; without it 5 pads drop.

Do NOT send these to `bga_fanout.py`. It models a ball grid, and a staggered
package's two offset rows project onto each axis at HALF the real pad spacing -
so its detected pitch is half the truth, its escape budget evaluates to a
NEGATIVE via size, and it grinds for ~50 minutes to reach the same answer.
`bga_fanout.py` now refuses these outright with the qfn_fanout command to use
(override: `KICAD_ALLOW_STAGGERED_BGA=1`).

Spotting one: the footprint name contains `AQFN`, or the part has far more pads
than a single peripheral ring of its size would hold (73-90 pads on a 7x7mm
body), or `bga_fanout.py` reports a pitch that is half the name's `P<pitch>mm`.

**Crowded fine-pitch QFN edge (surface fan has no room):** if a `qfn_fanout`
stub (especially a diff pair) is boxed in by a neighbour pair and a foreign
track and the surface 45° fan drops it, use `qfn_fanout.py --escape-method
underpad --via-size 0.45 --via-drill 0.25` (#164). It drops a through-via just
past each pad and escapes on an inner/back layer — straight out past the lateral
congestion instead of fanning into it (adjacent vias are staggered to clear).
Match `--via-size`/`--via-drill` to the board's fine-pitch via rule. If the
underpad run still **drops** a leg ("N dropped") because the via has no clear
room *outward* (a neighbour pad/track exactly one pitch away), add
`--allow-via-in-pad` (#161): the escape via may then sit on its own pad and
stagger *inward toward the chip*, away from the neighbour, instead of being
dropped. It still clears every other-net pad/via/track — it only gains
permission to overlap its own pad — so reach for it specifically when underpad
reports drops on a boxed-in fine-pitch pair.

**Size the escape via/track to the pitch BEFORE running fanout (issue #158).**
`bga_fanout.py` escapes one track down the channel between adjacent via columns —
at the **half-pitch**. So the via, track, and clearance must fit that half-pitch
or *every* escape grazes the neighbouring column's via by a few µm, and the fanout
still reports `failed: 0` (its success metric ignores sub-clearance grazes). The
budget, per array (measure each component's own pitch — they differ):

```
via_size + track_width + 2·clearance + margin ≤ pitch     (one escape track per channel)
via_size ≥ via_drill + 2·min_annular_ring,  track_width ≥ fab min track   (fab floors)
```

Don't just shrink the via against a *fixed* track — **solve for via AND track
together**, taking each down toward the fab floor as the pitch demands, and leave
a little margin so the result clears DRC instead of merely touching it. Read each
array's own ball pitch `P` (the min ball spacing — arrays on one board differ) and
the requested clearance `C` (Default net-class clearance from
`list_nets.py --design-rules`), plus the board's fab floors (`min_track_width`,
`min_via_diameter`/`min_via_drill`, annular ring), then:

```python
margin = 0.05                                  # slack: clear DRC, don't graze it
budget = P - 2*C - margin                       # room for one via + one track
track  = max(min(nominal_track, 0.15), min_track_width)   # keep a routable track
via    = min(nominal_via, budget - track)       # largest via that still fits
if via < via_floor:                # via fell below the floor -> thin the track to free room
    via   = via_floor
    track = max(min_track_width, budget - via)
infeasible = track < min_track_width or via < via_floor   # even fab floors won't fit
via_drill  = max(min_via_drill, via - 2*min_annular_ring)  # hold the annular ring at floor
# via_floor = max(min_via_diameter, min_via_drill + 2*min_annular_ring)
```

Pass the computed `--via-size via --via-drill via_drill --track-width track
--clearance C` to the fanout step. If `infeasible`, the pitch can't take a channel
escape even at the fab floor → for a POPULATED array prefer `--escape-method dogbone` (it never escapes fewer balls than underpad and matches the human idiom; this supersedes older underpad advice), else `--escape-method underpad`, and/or add
escape layers; don't ship the graze.

**Plan params can set ANY GUI option:** in the GUI's RESULT schema, each
step's `params` may include any option shown on that step's tab or the shared
options panel, keyed by its snake_case field name (`max_iterations`,
`max_ripup`, `grid_step`, `board_edge_clearance`, `hole_to_hole_clearance`,
`via_cost`, `heuristic_weight`, `turn_cost`, `ordering_strategy`, ...).
Unknown names are ignored with a note in the plan log. Use this to carry the
same values the equivalent CLI chain would pass (e.g. `--max-ripup 5
--grid-step 0.05`), so a GUI plan run matches a stress run step for step.
(Leave `max_iterations` at its default — the engine self-budgets, #529.)

**Why this heuristic matters for the GUI:** the plugin runs `/plan-pcb-routing` in
*plan-only* mode — it never executes the fanout and never runs the DRC↔smaller-via
retry loop, so it cannot discover a too-big via after the fact and shrink it. The
plan must therefore carry via/track that are **already** DRC-safe for the pitch.
Computing them here — both dimensions, with margin, clamped to the fab floor — is
what lets the single fanout the GUI runs come out clean the first time.

Worked example (a 256-ball 0.8 mm-pitch BGA, clearance 0.1, fab floor track 0.1 / via 0.45):
`budget = 0.8 − 0.2 − 0.05 = 0.55`; track 0.127 → via = min(working, 0.55−0.127) =
**0.42** (≥ floor) → DRC-clean, vs the Ø0.5 the net-class default would have used
(163 grazes). At 0.4 mm pitch the budget forces both to the floor (track 0.10, via
~0.30/0.20 advanced); if even those don't fit, go `--escape-method dogbone`
(populated array; `underpad` only when no inter-ball gap site exists at all).
`bga_fanout.py` also warns `WARNING: escape via ... busts the half-pitch budget`
when handed infeasible params, but choose feasible ones here so it never fires.

**Always check the fanout escaped all requested balls.** `bga_fanout.py` ends with
`JSON_SUMMARY: {"component", "requested", "escaped", "failed", "unescaped_nets", ...}`.
A dropped ball is **removed from the output** and later fails signal routing as "no
rippable blockers", so it must be caught here. If `failed > 0`, retry the fanout with
more layers and/or a smaller `--clearance` (see "Escape clearance" below) before
moving on — do not start signal routing while balls are still dropped.

**If balls still drop on a dense, fully-populated array, switch to the dog-bone
escape:** add `--escape-method dogbone` with a small via/track for the pitch
(e.g. `--via-size 0.35 --track-width 0.12 --clearance 0.1` at 0.8 mm pitch). The
`channel` engine confines every layer to the gaps *between* ball rows, so a few
channels over-subscribe and the deepest balls can't escape; `dogbone` stubs each
ball to a via in the diagonal inter-ball gap (falling back per-ball to
via-in-pad), so it never escapes fewer balls than `underpad` at roughly half the
via-in-pad / IPC-4761 fab burden (#669: orangecrab U3 108/108 vs underpad's
104/108, with the 3 stranded balls unrecoverable by ANY later routing).
`underpad` (every via in its pad) is for arrays with **no legal inter-ball gap
at all** — WLCSP-class pitches where even a floor via busts the half-pitch lane
budget. Caveats (both grid escapes): diff pairs route **single-ended**, and
power/plane nets are skipped as escapes — but every skipped plane ball still
gets a **plane-drop via** (below), so nothing is left stranded. `auto` (the
default) retries channel's drops with `underpad` only — a dogbone-first retry
was measured and REJECTED as the default (#669 sets1-5 corpus A/B: +10
incomplete nets, +59 kicad DRC — dogbone gap vias claim inter-ball streets
that chains not authored for dogbone then collide with). So on a populated
array, `dogbone` must be passed EXPLICITLY, with via/track/clearance chosen
for it — which is exactly what this plan does.

**How humans escape big BGAs — and which of OUR tool options that maps to**
(survey of 54 human corpus boards with a real ≥100-ball array; the fanout
places vias itself, so this is about choosing its options, not via positions):

- **Dog-bone is the dominant human method at every pitch** (median 30–43% of
  balls; via-in-pad is ~0% on most boards, appearing only on a handful of very
  dense 6/8-layer designs). Roughly HALF of all balls get no via at all — the
  outer rings escape on the surface, rails connect into pours. Mapping: for a
  populated array prefer **`--escape-method dogbone`** — each ball vias in a
  free inter-ball gap and falls back per-ball to via-in-pad, so it never
  escapes fewer balls than `underpad` while keeping the inner-layer streets
  open. `channel` (`auto`'s first pass) already leaves the outer rings
  via-free; keep it for sparse/perimeter-heavy arrays and diff pairs. Note
  `auto`'s retry for channel's drops is `underpad`, NOT dogbone (#669 measured
  a dogbone-first retry worse as a default) — so a populated array gets
  dogbone only by passing `--escape-method dogbone` explicitly, with params
  chosen for it.
- **Rail balls under a pour need NO via when the pour is on their own layer**
  — the plane-drop pass (#424) detects this automatically when the pours
  already exist (the Step 1 pour runs before fanout): it prints `N pour-covered (no via
  needed)` and skips those vias (measured: 104 of 127 GND balls on a 285-ball
  BGA, ~100 via barrels kept out of the escape field;
  `KICAD_FANOUT_POUR_DIRECT=0` reverts). This is why Step 1 pours before fanout;
  put rail pours on the layers that carry the rail balls (the outer layer for
  a surface flood). This generalizes beyond BGAs: **choose each outer-layer
  flood net by same-layer SMD pad count** — every SMD pad of the flood net on
  that layer connects by fill contact with no via at all. `list_nets.py
  --power` prints per-net `(F.Cu n SMD, B.Cu n SMD, TH n)` for this choice;
  ignore the TH counts (barrels connect on every layer regardless).
- **Escape via, by pitch:** at 0.8–1.0 mm the median minimum via in the
  courtyard is 0.45/0.20; at ≤0.5 mm humans go to 0.28/0.15 and even
  0.25/0.10. Escape-track minimum: median 0.125 mm at coarse pitch, 0.089–0.10
  (the fab floor) at fine pitch. The computed budget-per-pitch above lands in
  the same range — trust it, and treat 0.25/0.15 as the floor for ≤0.5 mm.
- **Deep balls leave through the inners, not the surface.** Inner-layer share
  of courtyard copper: ~0–15% on 4-layer boards, 30–67% on 6/8-layer. Mapping:
  give the fanout the FULL `--layers` list, and keep the escape-depth inner
  layers ROUTABLE — on a 6-layer board that means at most ONE solid inner
  plane next to each outer (fine-pitch-BGA humans keep a median of ONE solid
  plane total; **pouring 2–3 solid inner planes on a 6-layer BGA board is the
  classic self-inflicted failure** — it leaves signals a 2-layer board).
  Rails beyond GND go as SPLIT region pours or late route+pour, not extra
  solid planes and not wide tracks.
- **Buses concentrate.** A RAM/DDR bus runs on ONE inner "highway" layer with
  a solid GND plane adjacent (plus the outers). Mapping: pick the highway
  layer at plan time, keep its `--layer-costs` at 1.0–1.5, and put the solid
  GND plane on the layer NEXT to it.

**Plane-net balls are dropped to vias automatically (#424).** With any escape
method, after the signal escape each SMD ball on a plane net — a net excluded
from the fanout with ≥ 6 balls on the part, or an excluded net that already
owns a copper zone — gets a via immediately: a dog-bone via in a free
inter-ball gap, else a via-in-pad tap. The Step 1 pour — run BEFORE fanout and
before any routing — picks these vias up at fill while the pour is
still intact, which kills the tap-behind-the-ball-wall failure class (#360)
and, with the default-on plane-fragility field, keeps the plane whole through
signal routing (measured: pour-first + fragility served 63/70 balls by fill
alone; pour-last served 0/70 — every ball needed repair welds). Consequences
for the plan:
- Keep excluding plane nets from the FANOUT's `--nets` — the exclusion is
  exactly what marks them for drops. (The ROUTE step later includes them,
  #562 — its pour-launch anchors and in-run finalize complete them.)
- Pour the planes in Step 1, BEFORE fanout (see the Routing Order Rationale),
  so the drop pass sees real fill.
- The route step's plane finalize rarely needs to do more than verify under
  a dropped BGA (its oracle exits at round 0 on a healthy board).
- `--plane-drop off` disables the pass; `KICAD_FANOUT_PLANE_DROP=0/1`
  overrides either way (the recorded-manifest A/B switch). The per-net drop
  counts are in `JSON_SUMMARY.plane_drop`.

**After every BGA/PGA fanout, run the decoupling-cap placement optimizer
(#130).** A fanout drops vias near the ball field; where a foreign-net via
lands under a decoupling cap placed at a ball, the via copper overlaps the
cap pad → a real `PAD-VIA` DRC violation at the clearance floor. The fix is
placement, so run `place_fanout_clearance.py` on the **fanned** board to
nudge those caps clear (and pull each pad toward its nearest same-net ball so
a power/GND via dropped there later shares the via). See "Step 1b" below for
the command. It's cheap, only touches caps near a BGA, and is a no-op when
nothing collides — so run it ONCE after ALL fanouts are done, before signal
routing (see Step 1c for why once, not per-BGA).

Report to user:
- List of components that may need fanout
- Package type, pad count, and grid depth for each
- Recommended fanout tool

## Step 4: Check for Differential Pairs and Power Nets

Use `list_nets.py` to detect differential pairs and power/ground nets:

```bash
python3 py_router/list_nets.py path/to/file.kicad_pcb --diff-pairs --power
```

### Read the board's design rules and pass them to the CLI

The router does NOT read the board's design rules — it falls back to a generic
`--clearance 0.25` / `--track-width` default, which is often WIDER than the
board's own rule and can box pads in so nets fail with "no rippable blockers".
Read the board's real rules and pass them explicitly:

```bash
python3 py_router/list_nets.py path/to/file.kicad_pcb --design-rules
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
  width only for **current-carrying nets** (`--power-nets`).
  **Do NOT keep the net-class gap/width for impedance-controlled (diff-pair) nets** —
  the stock net class is usually wide (`diff_pair_gap` 0.25 / width 0.2 mm), and a
  fat pair is a wider bundle that gets dropped on congested boards (measured:
  a 4-layer FPGA corpus board routes all 13 of its pairs at `--diff-pair-gap 0.1` but loses 2 at
  0.25). Per `/find-high-speed-nets`, route those at the **fab floor for gap and
  clearance (~0.1 mm)** while keeping `--impedance` for the width (the router
  computes it from the stackup and clamps it to the floor). `route_diff.py` then
  auto-updates the Default net class to those tight values (only-loosen, via
  `fix_kicad_drc_settings.py`), so the `.kicad_pro` stops advertising the wide gap.
- **Diff-pair sizing default + shrink-to-succeed.** Default `route_diff.py` to
  **`--track-width 0.1` and `--diff-pair-gap 0.1`** (the fab floor) — a thin, tight
  bundle routes on congested boards where a fat pair is dropped. If the interface
  is impedance-controlled, ALSO pass `--impedance <ohms>`: the router derives the
  width from the stackup and clamps it to the floor, so the target impedance is
  **maintained** while the geometry stays as small as it can. **When a pair fails
  or falls back** — `route_diff.py`'s `JSON_SUMMARY` lists it in `failed_diff_pairs`
  or `single_ended_diff_pairs`, or DRC shows an intra-pair / via-via graze — re-run
  the failing pairs with **smaller track width, smaller gap, AND smaller vias**
  (`--via-size`/`--via-drill` toward the fab via floor). A tighter track+gap fits a
  narrower channel, and smaller vias fit a tight pad pitch (measured: lumenpnp
  USB_D's two 0.5 mm vias collide by 0.1 mm at the connector pitch — a smaller via
  clears it). Step **toward, never below**, the fab floors, and keep `--impedance`
  so the ohms target is held as the geometry shrinks.
- **Escape clearance — trigger on dropped balls, not pitch (issue #122):** the
  inter-ball channel is too narrow to fit a track at the net-class clearance on
  more BGAs than just "fine-pitch" ones. Even an **0.8 mm-pitch** BGA drops balls
  at `--clearance 0.2` (the ~0.45 mm gap between 0.35 mm balls can't fit a 0.2 mm
  track at 0.2 mm clearance) — the same board escapes **all** balls at the 0.1 mm
  floor. So don't gate on pitch: gate on whether balls actually dropped.
  `bga_fanout.py` and `qfn_fanout.py` both end with a `JSON_SUMMARY: {...}` line
  giving `requested`/`escaped`/`failed`/`unescaped_nets`. **After every fanout, parse it;
  if `failed > 0` (escaped < requested), re-run the fanout with `--clearance` at
  the manufacturing floor** (never below it — the floor is the rule the human
  board passes DRC against, so tightening board-wide is manufacturable and needs
  no rule-area settings). If still short, also try the smaller **fine-pitch escape
  via** (below) and/or a narrower `--track-width` toward the floor. Do not proceed
  to signal routing with `failed > 0` unexpected — those balls are dropped from the
  output and will fail later as "no rippable blockers".
- **Also check `drc_grazes` (even when `failed == 0`).** The summary's
  `drc_grazes` (graded at the fanout `--clearance`) reports sub-clearance grazes the
  escape left in the output: `via_segment` / `pad_via` are the #130 classes (an
  escape via too close to a foreign track or pad), `segment_segment` is the #179
  class (two escape **stubs** grazing — typically the 45° fans of two adjacent pads
  of a tight-pitch diff pair, e.g. 0.4 mm-pitch QFN, clipping at the wrist),
  `total` is all DRC violations. A *successful* fanout (every ball/pad escaped) can
  still leave many of these — they're not caught by `failed`. **If any
  `drc_grazes` class > 0 and there is headroom above the fab floor, re-run the
  fanout stepping toward — never below — the floor:**
  - `via_segment` / `pad_via` (#130): smaller **`--via-size` / `--via-drill`**
    (and/or a thinner `--track-width`).
  - `segment_segment` (#179): thinner **`--width`** — the escape stubs carry the
    track width, so narrowing them widens the gap between the two converging
    diagonals. Step down toward the fab-floor track (e.g. 0.15 → 0.13 → 0.10 mm)
    until `segment_segment == 0`; all pads still escape (`failed` stays 0).
    (Measured on a dense QFN corpus board: 3 grazes at `--width 0.15`/`0.13`, 0 at `0.10`.)

  These grazes are typically a uniform ~1-grid-cell shortfall, so even one size step
  down usually clears them all; shrinking the via also relieves escape congestion.
  (For *via-over-pad* grazes where a decoupling cap/resistor sits on a via,
  `place_fanout_clearance.py` (Step 1b) is the better fix — it moves the part;
  smaller vias/thinner tracks help the via-over-track and stub-over-stub classes.)
- **Fine-pitch escape VIA (4+ layer):** the 0.45 mm standard via can't dog-bone /
  via-in-pad sub-~0.5 mm-pitch BGA/QFN balls. For *those parts only*, pass the
  smaller **fine-pitch escape via** that `--design-rules` prints (`fine-pitch
  escape via <d>/<drill>`, e.g. `0.30/0.15` — JLC "advanced", small extra cost)
  as `--via-size`/`--via-drill` to that part's `bga_fanout.py` / `qfn_fanout.py`
  and to `route_diff.py` when it launches from that part's escaped stubs. (The
  in-run plane finalize taps at the ROUTE step's via — fine-pitch plane balls
  under such parts should already carry fanout-time plane-drop vias, #424; if
  the finalize still reports them unconnected, re-run the route step with this
  smaller via.) Keep the **standard** working via for
  general `route.py` routing and the bulk `route_planes.py` pour — the advanced
  via is escape-only, not a board-wide default (issues #99/#122).
- **Non-Default classes:** route those nets separately with that class's
  `--clearance`/`--track-width` (clearance is the one per-class DRC value, so keep
  each class's nets at their own clearance rather than forcing one global value).
- **Diff pairs:** default `--track-width 0.1 --diff-pair-gap 0.1` for `route_diff.py`
  (NOT the wide net-class values), plus `--impedance` when the interface is
  impedance-controlled; shrink track/gap/via further toward the fab floor for any
  pair that fails or grazes (see "Diff-pair sizing default + shrink-to-succeed").
  **Never set `--diff-pair-gap` below the same command's `--clearance`** — KiCad
  grades the pair's P↔N coupling as a plain clearance violation, so `route_diff`
  floors the gap up to clearance (#441). Set the two equal (both at the fab floor).

**Verification (DRC/connectivity) grades at the manufacturing floor**, not the
inflated net-class clearance — that is the same rule the human original passes, so
it's the honest delta. The routing/plane/fanout steps now **record the smallest
clearance any step actually used** (route_planes, route.py's plane finalize, and the
single-ended multipoint taps auto-step the fine-pitch tap clearance DOWN toward the
fab floor as the geometry demands) into the output `.kicad_pro` DRC floor and into
`JSON_SUMMARY` (`min_clearance_used`). `check_drc.py` **auto-grades at that
`.kicad_pro` clearance when `-c` is omitted**, so a bare `check_drc.py board.kicad_pcb`
already grades at the true routed floor. Passing `--clearance <floor>` still works
to TIGHTEN the grade — it is a FLOOR, `max(-c, classA, classB)`, not an
override, so a value at or below the board's netclasses changes nothing.
See Step 6.

**`check_drc.py -c` is NOT `route.py --clearance`.** On route.py the flag is a
**ceiling over every class** (`--clearance` caps each net at `min(its class,
--clearance)`). On `check_drc` it is only the **global fallback**, and a netclass
override still wins — the tool prints `Required clearance: 0.1600mm
(local/netclass override; global 0.1500mm)` and grades at 0.16 no matter what
`-c` says. Measured on one board: 7 violations at `-c 0.16`, the same 7 at
`-c 0.15`, the same 7 at `-c 0.149`. If you expected a looser `-c` to clear
class-driven violations, it will not; change the class, or use
`--clearance-margin` (default 0.05) to filter grid-quantisation noise — and when
you use it, quote the unfiltered count beside the filtered one.

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

**Polarity-swap policy (#279).** `route_diff.py` can resolve a P/N polarity mismatch by
swapping the target pads' net assignments — but a swap physically cross-connects one
device's P pin to the other's N pin, and is only harmless when an endpoint can compensate.
Swaps are **denied by default**; grant them per pair with `--polarity-swap-nets <patterns>`.
Before emitting the route_diff command, classify each pair's electrical endpoints (walk
through series AC caps/resistors to the real device):

- **Allow** pairs with an FPGA/CPLD generic-I/O endpoint (pin functions are reassigned in
  gateware — look for paired `IO_LxxP/N`-style pinfunctions on Xilinx/Lattice/Altera/Gowin
  parts), and protocol-tolerant links (PCIe lanes, SerDes with polarity-invert, 1000BASE-T).
- **Deny** USB `D+/D-`, MIPI, TMDS/HDMI/DP, CAN, RS-485/422, DDR `CK/DQS`, clock/analog
  inputs to fixed-function parts, anything reaching a connector or unknown part, and any
  pair whose nets carry an asymmetric attachment (e.g. a single-sided pull-up) — it stays
  on its net and would land on the wrong physical wire. MCUs/SoCs do NOT count as
  programmable (their diff functions are fixed silicon). **When in doubt, deny** — a
  skipped pair beats a dead interface. `/identify-diff-pairs` reports a per-pair
  `polarity_swappable` verdict from datasheet pin functions for the ambiguous cases.

Pass the resulting allowlist, e.g. `--polarity-swap-nets '/fpga/IO_*'` (use `'*'` only when
every pair classifies swappable). Applied swaps are listed in `polarity_swapped_pairs` —
when they happen, the schematic sync step below applies (see "Schematic Synchronization
After Swaps"). Pairs that *wanted* a swap but were denied are listed in
`polarity_swap_denied_pairs` — surface these to the user (they either routed via the
opposite-side flip or failed honestly and may need a manual pin swap in the schematic).
Side-flipped pairs the HYBRID routed with a wrap-around leg because no swap was
permitted are listed in `polarity_flip_unswapped_pairs` (#266) — surface these too:
adding the pair to `--polarity-swap-nets` on a re-run may shorten it.

**Far-apart terminal pads → single-ended follow-up (issue #121).** A "diff pair"
sometimes has pads that aren't a coupled connection — e.g. a P and an N test point
several mm apart, or a logical pair daisy-chained through spread-out parts. If the
coupled chain can't be routed, `route_diff.py` peels those far-apart pads off the
chain (routing the genuinely-coupled terminals as a pair) and lists the affected
nets under `single_ended_followup_nets` in its `JSON_SUMMARY` (and a "route them
single-ended next" block on stdout). Those pads are **not** dropped — the **Signal
Routing** step (`route.py --nets "*"`) connects them P→P / N→N along with
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
- If no high-speed nets found: "**GND Return Vias:** The high-speed scan found
  no nets that need them (only low-frequency I2C/UART/GPIO). The step is
  included; it is cheap and harmless here. Want me to remove it?"
  Say what the SCAN found, not that the vias are "optional" -- optional invites
  dropping them on a board where the scan simply was not run, and a missing
  return path is not visible in any DRC.

`/find-high-speed-nets` ALSO reports **controlled-impedance nets** (its Step 4.5):
RF/antenna feeds (radio/PA/LNA -> SMA/U.FL/chip-antenna = **50 ohm single-ended**,
or 100 ohm if balanced), DDR SSTL, and the impedance-controlled diff interfaces.
Thread these into the plan:

- **Differential** impedance nets stay in the diff-pair step (Step 2) — just add
  `route_diff.py --impedance <ohms>`.
- **Single-ended** impedance nets (RF 50, DDR SSTL 40) get a **dedicated
  `route.py --impedance` pass placed AFTER diff pairs and BEFORE the general
  signal route** (Step 2b below). They must then be **excluded from the general
  signal route** (`"*" "!RF"` — the plane nets stay IN that route, #562) and
  counted in the Step 5b ledger as
  claimed by the impedance step — otherwise a later rip-up re-routes them at the
  wrong width.
- Impedance width is computed from the **stackup**: if the board has only KiCad's
  default stackup, lead the report with that warning and run `/recommend-stackup`
  first (an RF feed routed at a wrong width is electrically useless).
- For an RF/antenna feed also recommend (in words) a `User.2` keepout around the
  antenna region and `--keepout`, and route it short/direct on an outer layer.

If no controlled-impedance nets are found, omit Step 2b.

### Step 2b-i: Coplanar (CPW-over-ground) — decide this WITH the plane step (#486)

An impedance trace on an **outer layer that will also carry a GND pour** is not a
microstrip: the side ground pulls Z0 down hard, so hitting the target needs a
**narrower** trace (e.g. 0.277 mm instead of 0.376 mm for 50 Ω on 0.2 mm FR4).
Routing the microstrip width through a pour lands the trace well below target.

The router cannot detect this — the trace width comes from your declaration,
not from sensing copper. So this is **your decision to make in the plan**, and
it must be coordinated across two steps. (With the pour-first order, an
outer-layer GND pour normally comes from Step 1c — give THAT call the matching
`--zone-clearance G`; a pour-first pour makes the declaration safer, since the
copper the trace is sized against actually exists when it routes.)

**Declare coplanar when ALL of these hold:**
1. The impedance net routes on an **outer layer** (`F.Cu` / `B.Cu`). Inner layers
   are stripline; the flag is ignored there.
2. A `route_planes` step in this plan pours **GND on that same layer** — or the
   board already has an outer-layer GND pour that will survive.
3. You can name the gap: it is the pour's zone clearance.

**If you are not pouring on the signal's own layer, do NOT pass `--coplanar-gap`.**
A coplanar declaration whose pour never arrives leaves the trace too narrow, i.e.
impedance too HIGH — the opposite error, equally wrong.

**Coordination — one number, three places:**

```bash
# choose ONE gap G (the pour's clearance; near the fab floor, e.g. 0.2)
# 1. route the impedance nets, declaring G
python3 py_router/route.py in.kicad_pcb s2b.kicad_pcb --nets "RF*" \
    --impedance 50 --coplanar-gap 0.2 --clearance 0.2

# 2. pour GND on the SAME layer with a MATCHING zone clearance
python3 py_router/route_planes.py s2b.kicad_pcb s5.kicad_pcb \
    --nets GND GND --plane-layers F.Cu B.Cu --zone-clearance 0.2

# 3. verify the declaration actually held
python3 py_tools/check_impedance.py s5.kicad_pcb --coplanar-gap 0.2 --nets "RF*"
```

- `--coplanar-nets "<patterns>"` narrows the declaration to some nets in a call;
  omit it and every net in that call is treated as coplanar. Since Step 2b is
  already a dedicated impedance pass over exactly those nets, omitting it is
  usually right.
- `route_diff.py` takes `--coplanar-gap` but has **no** `--coplanar-nets` (the
  diff engine bakes one width per layer). Split interfaces into separate calls.
- The gap must be **achievable**: it is a pour clearance, so it cannot be below
  the fab floor, and near via antipads / pads the real gap will be wider. The
  Step-3 audit reports how much of each net actually achieved it.

**Report to the user** which nets you declared coplanar, the gap, and the plane
step it is tied to — this is a coupled choice they may want to override. If the
board has no outer-layer pour planned, say so explicitly and note that the
impedance nets are being routed as plain microstrip.

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

### Step 5a-tuned: Plane-map derivation rules (measured-optimal; refine the delegate's output with these)

**THE DENSITY GATE COMES FIRST — plane-map aggression must scale with the
board (15-board wave + controlled A/B, 2026-08-17).** The aggressive map
below (outer floods, rail co-pours, many-rail splits) is what wins on dense
BGA boards (orangecrab 18 KiCad-unconnected, the best from-scratch result of
five arms; daisho 8-layer: 1 open, 0 new DRC). The SAME map applied to
small boards was the wave's dominant failure source AND its dominant time
sink: outer floods got carved into pad-anchored islands and hairline
(<60 µm) gaps, and every route pass re-oracled the big outer fills.
Controlled A/B on the four regressed boards, changing ONLY the plane map
(floods+fragility=0 → inner-only) with every other step identical:

| board | aggressive map | inner-only map |
|---|---|---|
| a 4-layer board | 7 open, 60s | **0 open, 41s** |
| upduino | 5 open + 2 DRC, 286s | **0 open, 1 DRC, 61s** |
| eis | 3 open, 504s | **0 open, 2 DRC, 115s** |
| a small 2-layer board | 1 open, 428s | **0 open, 0 DRC, 88s** |

And the reverse control on the dense board cuts the other way just as
hard — the same skill chain with the conservative (recorded-style) map
on orangecrab: **26 open at 7687 s vs 18 open at 2028 s** with the
aggressive map (plus ~2000 self-crossing weld-debris warnings on the
conservative arm). The aggressive map on a dense board is BOTH more
complete and ~4× faster; the conservative map on a small board is both
more complete and 3–5× faster. Neither map is "the safe one" — the GATE
is the safety.

**Compute the tier, don't vibe it. DENSE** = the board has **6+ copper
layers AND** (a populated fine-pitch grid array of ≥100 balls at ≤0.8 mm
pitch, **or** >150 nets). Everything else is STANDARD. Layer count is the
load-bearing half of the gate: on ≤4 layers the outer layers ARE the
routing surface, so floods there lose even next to a big BGA (measured:
eis, 4-layer with a fully-populated BGA-121 @0.8 mm, went 3 opens/504 s
with the aggressive map → 0 opens/115 s inner-only; orangecrab and daisho,
6/8-layer, are where the aggressive map wins).

**STANDARD boards (the measured-optimal default — most boards):**

- **Inner-only pours**: GND solid on the first inner layer (price 6.0);
  the ONE dominant rail (most pads) solid or split on the second inner
  (price 2.5). **NO outer-layer floods** — on a small board the outer
  layers ARE the routing surface, and a flood there becomes island debt,
  sub-60 µm gap debt, and board-edge DRC (all three measured).
- **Every other rail rides `--power-nets` as a wide trace.** Do not
  Voronoi many rails onto one layer: **never more than 2–3 rails share a
  split layer** (measured: six rails Voronoi'd onto one 4-layer board's In2
  fragmented +3V3 into 8 pad-anchored islands → 7 opens; the 2-rail map
  → 0).
- 2-layer boards: GND flood(s) per Step 8's 2-layer flow (pour LAST on
  dense 2-layer); rails as traces.
- No per-zone fragility overrides — the default fragility field protects
  inner pours correctly.

**DENSE boards (the aggressive map — every rule keys on a MEASURED board
property; derive, don't copy):**

1. **Outer-layer GND floods (pour-direct service).** Count GND SMD pads
   per outer layer. An outer layer with a substantial GND SMD population
   (≳20 pads, or a fine-pitch BGA's GND balls on it) gets a GND flood:
   pads and balls are then served by FILL CONTACT with zero vias
   (measured: 124 balls pour-served, the first 100% fanout escapes).
   Floods must be carve-free — per-zone fragility `GND@<layer>=0` — or
   later routing fragments them into weld debt. (This knob is DENSE-only:
   it is exactly what turned small-board floods into island debt.)
2. **One solid inner GND plane, adjacent to the highway.** The unsplit
   reference layer. Price it 6.0 in `--layer-costs`.
3. **Bus-highway layer: derive it, keep it EMPTY and FREE.** Find the
   board's widest bus (largest same-endpoint-footprint-pair net group, or
   the dominant netclass family — DDR data/address, ≥8 nets). The highway
   is the inner layer adjacent to the GND reference spanning the bus's
   endpoints. NO pours on it, cost 1.0 — pricing or pouring the highway
   cost completions every time it was measured.
4. **A rail whose pads live overwhelmingly on one outer layer shares
   that layer's flood.** ≥~80% of the rail's pads SMD on outer layer L
   (termination arrays, e.g. VTT on B.Cu) → co-pour on L by Voronoi/
   grammar partition; the pads connect by fill contact and the rail needs
   no inner-layer real estate at all.
5. **Remaining rails: split across the remaining inner layers, grouped
   by pad geography — capped at 2–3 rails per layer.** Cluster rails
   geographically (the grammar-pour clustering) and assign clusters per
   layer so each partition stays compact (#662 shape targets: sheet
   compactness ≥0.6, islands ≥0.5). Price rail layers 2.5. Rails that
   don't fit under the cap (or whose region would be a sliver) ride
   `--power-nets` as wide traces instead — a fragmented pour is worse
   than no pour.
6. **Carry the SAME `--layer-costs` vector into `route_diff`** — the
   diff step is otherwise plane-blind and its pairs squat the priced
   layers.

### Step 5a-tuned-ii: Escape completeness sweep (all interior-pad parts)

COMPLETE FANOUT IS THE INVARIANT — never trade an escaped ball for a
routing score. Beyond the big BGAs, enumerate EVERY component with
interior pads the surface router cannot reach (WLCSP/CSP at ≤0.5mm pitch,
staggered no-lead arrays — the issue-#144 class) and give each its own
fanout step (`underpad` + via-in-pad at the pitch-derived fab-floor via)
BEFORE the route step. On the benchmark, three 0.4mm WLCSP regulators
nobody had ever fanned were part of the winning plan. A board-wide sweep:
for every footprint, compute min pad pitch and whether any pad is
enclosed by other pads on all four sides; plan an escape for every hit.

### Step 5a-tuned-iii: Length matching from the board's own classes

If netclass names carry length-match hints (`*_LM<tol>`, `*length*`,
DDR-class groupings), wire them into the route step:
`--length-match-group auto --length-match-tolerance <tol>` and
`--time-matching` when the bus spans layers — but `--time-matching` ONLY
on a board with a real stackup (it converts length to delay through the
dielectrics; on KiCad's default stackup it computes garbage — the Step 10
rule-1 no-stackup precedence applies to it exactly as to `--impedance`).
The board's classes are the author's spec — honor them even when the
recorded chains never did.

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
   - `route step` — ordinary signals AND the plane nets (#562: the route step
     takes `"*"`; plane pads weld into their pour via pour-launch and the
     in-run finalize taps whatever fill can't reach)
   - `diff-pair route` — detected pairs
   - `impedance SE route (Step 2b)` — single-ended controlled-impedance nets (RF/antenna
     50 ohm, DDR SSTL 40 ohm); the ONLY nets excluded from the route step
   - `pour` — nets the plane step pours; they are ALSO in the route step (see above)
   - `wide trace` — power carried via `--power-nets` widths (never excluded)

2. **Diff the pattern lists (#562 rules).** Two checks, and note that the old
   "every poured net must be excluded" rule is now exactly backwards — a
   poured net that is missing from the route step is the bug, because nothing
   then welds its pads to the pour:
   - the route step's exclusions MUST equal the Step-2b impedance set;
   - every poured net MUST appear in the route step's `--power-nets` (that is
     where the finalize's taps and welds get their width).
   ```python
   route_exclusions = {"RF"}                    # the !X you will pass route.py
   plane_nets       = {"GND", "+3V3"}           # the --nets you pass route_planes.py
   impedance_se     = {"RF"}                    # nets routed in Step 2b (route.py --impedance)
   power_nets       = {"GND", "+3V3"}           # the --power-nets on the route step
   orphans = route_exclusions ^ impedance_se
   assert not orphans, f"Net-coverage gap: {sorted(orphans)} handled by no stage"
   unsized = plane_nets - power_nets
   assert not unsized, f"Poured but no route-step width: {sorted(unsized)}"
   ```
   Do not proceed until both are empty.

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

1. **Pour the planes FIRST** — before fanout, before any routing. A bare
   `route_planes`
   call: nets + layers only. NO `--add-gnd-vias`, NO `--stitch-vias` — those
   adapt to signals that don't exist yet (the old #56 hazard) and belong in
   Step 3. (The pour cannot rip at all any more: `--rip-blocker-nets` and the
   other tap knobs were REMOVED from `route_planes` with the tap machinery.) Why the pour comes first (#424, measured):
   the fanout's plane-drop vias connect to a still-intact pour immediately, and
   the **plane-fragility field** (default on: `KICAD_PLANE_FRAGILITY_COST`,
   2.0 mm-equiv, `=0` reverts) then makes every later routing step pay to cut
   the real fill where it is narrow — so signals cross planes mid-pour, not at
   necks. Measured on a 4-layer corpus board, this order + the field: power nets fully connected,
   +3V3 pour ONE intact island, GND weld copper cut to a third, connectivity
   net-better, DRC clean. With planes poured signals-first style instead, the
   pour under a BGA arrives pre-shredded and every drop via needs repair welds.
1b. **Fanout** (if needed) - Escape routing on the poured board. Exclude the
   plane nets (`"*" "!GND" "!VCC"`) — that exclusion marks them for automatic
   **plane-drop vias** (#424), and because the pour already exists the drop
   pass can skip a via entirely where the fill already covers the ball
   (pour-direct) and land the rest on intact copper.
1c. **After ALL fanouts are done — once, not per-BGA — run
   `place_fanout_clearance.py`** to clear decoupling-cap / fanout-via
   collisions (#130) before routing. The pass is board-global (it reads every
   via and every BGA), so one late run sees everything; running it per-BGA
   compounds cap displacement and changes what later fanouts route around.
   See Step 1c.
2. **Differential Pairs** - The most constrained routes claim their channels before
   anything else can block them (if present). Add `--impedance <ohms>` for the
   controlled ones (USB/Ethernet/LVDS/balanced-RF; from `/find-high-speed-nets`).
   May peel far-apart "terminal" pads (e.g. spread-out test points) off the coupled
   chain and leave them for the signal-routing step (reported as
   `single_ended_followup_nets`, issue #121). (The pour is not an obstacle to
   them — pours never block the router; the fragility field only prices
   plane-severing paths.)
2b. **Impedance-controlled single-ended nets** (only if `/find-high-speed-nets`
   found any - RF/antenna feeds = 50 ohm, DDR SSTL = 40 ohm). A dedicated
   `route.py --impedance <ohms>` pass, routed here - after diff pairs, before the
   bulk signal route - because they need a stackup-derived width and a short,
   direct path over a clean ground reference, so (like diff pairs) they must claim
   their channel before the bulk signals fill the area. Route an RF feed on an
   outer layer (`--layers F.Cu`); requires a real stackup (see Step 2 stackup
   check). These nets are then EXCLUDED from step 3.
3. **Route ALL nets (#562)** - `--nets "*"`, plane nets INCLUDED: their pads
   weld into the pours via pour-launch anchors (default on) instead of routing
   as track webs, and the run FINISHES with the in-run plane finalize (taps +
   region joins + cleanup + KiCad-oracle verify, stubborn links joining the
   final reconciliation). Pass the plane nets in `--power-nets` with widths.
   Exclude only the single-ended impedance nets from step 2b (`"!RF"`) -
   re-routing those would drop their controlled width. This step also finishes
   any diff-pair pads peeled off in step 2, so keep the diff-pair nets in its
   selection (the `"*"` covers them). The fragility field steers it away from
   severing the Step 1 pours.
4. **Finalize planes (only when GND return vias / stitching are wanted)** -
   Re-run `route_planes` with the same nets/layers plus `--add-gnd-vias` (and
   any `--stitch-*` flags): an existing same-net zone is REPLACED in place, and
   the return/stitching vias now adapt around the finished signals — the #56
   ordering concern lives here, not at the pour. This step cannot rip either --
   the pour never taps, so there is no blocker to clear.
   **Stitching is normal human practice, not an exotic add-on**: 58% of ~400
   human corpus boards carry a free-standing GND stitch lattice. **But gate
   it on the SPEED TIER, not on pour count**: recommend `--stitch-vias` only
   when `/find-high-speed-nets` puts the board at high tier or above (RF,
   TMDS/SerDes, DDR) — "GND pours on 2+ layers" alone is NOT sufficient.
   Measured cost of stitching low-speed boards (15-board wave): the
   re-pour + mandatory trailing route.py roughly double a small board's
   wall time, and the stitch pass is the source of the recurring
   8–21 µm via-segment micro-graze DRC class and occasional orphan stitch
   vias — pure cost when nothing on the board needs the lattice. Leave
   the PITCH at the tool default (20 mm); only `/find-high-speed-nets` output
   tightens it (via `--stitch-max-freq`, which derives λ/20 and overrides the
   pitch). Do not hand-pick a pitch from corpus statistics.
   **Do not recommend `--add-teardrops`** (7% of human boards use teardrops)
   and **do not set `--thermal-relief`** — leave the tool's default connection
   style alone.
5. **Verification** - DRC and connectivity checks (plane repair is inside
   step 3's finalize — there is no separate repair step, #562)

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

### Step 1: Pour the Power Planes (FIRST — before fanout and routing, #424/#562)
A bare pour: nets and layers ONLY. No `--add-gnd-vias`, no `--stitch-*` —
those adapt to signals that don't exist yet and run in Step 3 instead. **The pour runs FIRST, before fanout**: the fanout's
plane-drop pass then sees real fill, so a ball the pour already covers needs
no via at all (pour-direct) and the ones that do get a via land on intact
copper. The pour step itself does no routing at all (#562: it places no taps
— the route step's pour-launch and in-run finalize own every plane pad).
The default-on plane-fragility field
(`KICAD_PLANE_FRAGILITY_COST`, 2.0 mm-equiv; `=0` reverts) then charges every
later routing step for cutting the fill where it is narrow — signals cross the
planes mid-pour instead of severing them at necks.

python3 -X utf8 py_router/route_planes.py board.kicad_pcb board_step1.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    2>&1 | tee /tmp/step1_pour.txt

**Zone clearance is a MINIMUM-ALLOWED, not a target — never pass a
`--zone-clearance` larger than the routed clearance.** The default already
follows `--clearance` and auto-steps down to the fab floor when the pour
can't thread the densest BGA via lattice; a larger value only stops pours
from penetrating between balls/vias (human boards pour at ~0.1 for exactly
this reason). Watch the pour output for
`pour cannot thread the densest BGA lattice even at the fab floor`: when it
fires, no clearance setting can get an INNER-layer pour through that field —
the fix is a pour on the balls' OWN (outer) layer, which connects the pads by
direct contact (the plane-drop pass then skips those vias: `N pour-covered`).
`--min-thickness` (default 0.1) matches human under-BGA pours (0.089–0.1);
leave it unless a fab demands wider minimum copper.

Expect `check_connected` to show the plane nets fully connected from here on
(the drops + pour serve every BGA plane ball with no tap search).

### Step 1b: Fanout U9 (PGA120) - All Non-Plane Nets
Generates escape routing for ALL nets on the component EXCEPT those that the
planes step will handle. This ensures every signal net gets fanned out,
avoiding `--no-bga-zone` workarounds during routing.

**Important:** Use `"*" "!GND" "!VCC"` to fan out all nets except the power
plane nets. Do NOT use `"/*"` alone, as it misses nets with non-hierarchical
names like `Net-(U9-Pad1)` which would then require `--no-bga-zone` to route.

On a 4+ layer board also pass every copper layer with `--layers` (default is
F.Cu B.Cu only) so inner balls can escape — drop `--layers` only for true
2-layer boards.

python3 -X utf8 py_router/bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" "!VCC" \
    --layers F.Cu In1.Cu In2.Cu B.Cu \
    --output board_step1b.kicad_pcb \
    2>&1 | tee /tmp/step1_fanout.txt

**Then check the `JSON_SUMMARY` line: if `failed > 0`, balls were dropped — retry
before continuing.** First confirm all copper layers are passed; then re-run with
`--clearance` at the manufacturing floor (e.g. `--clearance 0.1`), which fixes the
common case (an 0.8 mm-pitch BGA can't fit a track between balls at 0.2 mm). If still
short, add the fine-pitch escape via and/or a smaller `--track-width`. Only proceed
to Step 2 once `failed == 0` (or the remaining `unescaped_nets` are understood and
accepted).

### Step 1c: Optimize Decoupling-Cap Placement (run ONCE after ALL fanouts — issue #130)
Nudges decoupling caps near the BGA off the foreign-net fanout vias (the
`PAD-VIA` violations #130) and pulls each pad toward its nearest same-net
ball. Run it on the fully-fanned board — after the LAST fanout, **before**
signal routing. Use the
**same `--clearance`** you gave the fanout / your DRC floor — that's the only
setting that matters (it reads each via's real size from the board).

python3 py_placer/place_fanout_clearance.py board_step1b.kicad_pcb board_step1c.kicad_pcb \
    --clearance 0.1

It prints `Moved N cap(s); resolved R/M ... K unresolved`. Any **unresolved**
caps had no clear spot within the displacement budget — note them for a manual
nudge; they are not auto-fixed. By default (`--cap-prefix C,R`) it moves 2-pad
**caps and resistors** near a BGA (RN-style arrays auto-excluded since only
2-copper-pad parts move); it never overlaps parts, and is a no-op when nothing
collides. Feed `board_step1c.kicad_pcb`
into the next step. **With multiple BGAs, run it ONCE after the LAST fanout,
not after each.** The pass is board-global — it reads every via on the board
(`for v in pcb_data.vias`) and every BGA footprint for the same-net ball
attraction — so a single late run already sees every constraint at once. Per-BGA
runs are not equivalent, in two ways:
- **Displacement compounds.** Each cap's seed is wherever it sits on the board
  it is handed (`seed_x, seed_y = fp.x, fp.y`), and the budget
  (`--max-displacement` 2.0, growing ×1.5 to `--max-displacement-cap` 3.0) is
  measured from THAT seed. A second run re-seeds at the already-moved position,
  so a cap can drift ~2× the cap budget from where it started, and "move as
  little as possible" becomes minimal-from-the-moved-spot rather than from its
  real seed.
- **Moving caps changes later fanouts.** Cap pads are in the escape router's
  obstacle map (foreign pads + existing copper + vias), so tidying after BGA1
  hands BGA2's fanout a different obstacle field — different escapes, different
  vias, and then different cap decisions.
The two orders usually converge anyway (decoupling caps cluster around their own
BGA, so BGA1's caps are rarely in BGA2's escape field), which is why per-BGA was
long treated as interchangeable. Once-after-all is the default because it cannot
compound and costs one step instead of N.
Verify with `check_drc.py board_step1c.kicad_pcb -c 0.1` (PAD-VIA count drops).

### Step 2a: Differential Pairs (only if any were detected)
The most constrained routes claim their channels first. Add `--impedance <ohms>`
for controlled interfaces (USB/Ethernet/LVDS/balanced RF, from
`/find-high-speed-nets` or `/identify-diff-pairs`). The pours from Step 1 do not
block these — pours are never obstacles; the fragility field only prices paths
that would sever them. Pairs may peel far-apart terminal pads off the coupled
chain and report them in `single_ended_followup_nets`; the Step 2 route finishes
those, so do NOT exclude the pair nets there.

python3 -X utf8 py_router/route_diff.py board_step1c.kicad_pcb board_diff.kicad_pcb \
    --nets <pair globs, e.g. '/usb/*'> \
    --track-width 0.1 --diff-pair-gap 0.1 --clearance <floor> \
    [--impedance 90] \
    2>&1 | tee /tmp/step2a_diffpairs.txt

(No diff pairs on the board? Skip this step and feed `board_step1c.kicad_pcb`
straight into Step 2b / Step 2.)

### Step 2b: Impedance-Controlled Single-Ended Nets (only if any were found; runs before the Step 2 signal route)
ONLY when `/find-high-speed-nets` reported single-ended controlled-impedance nets
(RF/antenna feed = 50 ohm, DDR SSTL = 40 ohm). Route them in their own
`--impedance` pass, after diff pairs and BEFORE the general signal route, so they
claim a clean, short, direct channel at the stackup-derived width. Requires a real
stackup (run `/recommend-stackup` first if the board has KiCad's default). Route an
RF feed on an outer layer over the GND plane; recommend a `User.2` keepout +
`--keepout` around any antenna region (user draws it).

python3 -X utf8 py_router/route.py board_diff.kicad_pcb board_step2b.kicad_pcb \
    --nets RF --impedance 50 --layers F.Cu \
    --clearance <floor> --no-bga-zones \
    2>&1 | tee /tmp/step2b_impedance.txt

### Step 2: Route ALL Nets — plane nets included (#562)
Routes every unrouted net, **including the plane nets poured in Step 1** —
`--nets "*"` with no plane exclusions. Plane-net pads connect by welding
into the pour (pour-launch anchors, on by default), not by re-routing the
net as a track web, and the run **finishes with the plane finalize**: the
plane-repair engine (pad taps + region joins), the plane-copper cleanup,
and the KiCad-oracle exact-fill verify/reconnect all run IN this step, with
any stubborn oracle links joining the run's own final reconciliation. There
is **no separate plane-repair step anymore** — `repair_planes.py`
remains only for repairing a board outside this chain. Exclude only the
single-ended impedance nets already routed in Step 2b (`"!RF"`), so the
bulk pass cannot re-route them off their controlled width. The pours don't
block the router, and the fragility field makes plane-severing paths
expensive, which is what keeps them intact through this step.

**Pass the plane nets in `--power-nets` with widths** (e.g. `GND 0.3`): the
finalize's taps and welds size their copper from the power-width channel.

For boards with BGA/PGA components, use `--no-bga-zone` to allow the router
to find alternative paths through the dense pin area (even when fanout was
done, some paths may require this). Use `--max-ripup 5` for difficult
2-layer boards.

**If the finalize reports `Pads still unconnected` on fine-pitch (BGA/QFN
≤0.5 mm-pitch) pads, re-run this step in this order — cheapest first:**
1. **Smaller via** — drop `--via-size`/`--via-drill` toward the fab's
   fine-pitch escape via (e.g. `0.30/0.15`), never below the fab via floor.
   A boxed ball usually fails because the tap via can't fit beside it.
2. **Then finer grid** — drop `--grid-step` (e.g. `0.05 → 0.025`), not
   below the board's minimum feature: a 0.65 mm-pitch escape can be a
   grid-resolution limit, not a width one.
(BGA plane balls under a dropped part should already carry fanout-time
plane-drop vias (#424), so this retry is rare.)

> **Do NOT pass `--max-iterations` (#529 dynamic iterations, default on).**
> The router self-budgets: full searches automatically earn +1×base
> extensions while the search's heuristic keeps approaching the target, up
> to a 1e7-iteration ceiling — a genuinely hard net gets far MORE than the
> old `--max-iterations 1000000` advice ever gave it, while hopeless
> searches stop early. A net that still fails after an
> `"dynamic iterations (#529): search extended to N"` log line is a
> capacity problem (rip-up, clearance, layers), not a budget problem.
> (`KICAD_DYNAMIC_ITERATIONS=0` restores the legacy static caps for A/B.)

python3 -X utf8 py_router/route.py board_step1c.kicad_pcb board_step2.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 5 \
    --power-nets GND VCC <other PWR...> --power-nets-widths 0.3 0.4 <W...> \
    --layers <ALL copper layers> --layer-costs <1.0 signals / 3.0 solid planes / 1.5 split-or-highway> \
    2>&1 | tee /tmp/step2_routing.txt

The `--layer-costs` line is NOT optional when Step 1 poured any solid plane:
without it signals cross the pours at cost 1.0 and shred them (measured: split
power pours at 0–2% connected under a BGA on a chain that omitted it). Order
matches `--layers`; 3.0 on solid-plane layers, 1.0–1.5 on split/route+pour and
highway layers, 1.0 on F/B. On dense boards use the measured-optimal pricing
from Step 2c instead (GND plane 6.0, rail pours 2.5, bus highway FREE).

(When Step 2b ran, exclude its impedance nets, e.g. `--nets "*" "!RF"`, and
route from `board_step2b.kicad_pcb`.)

This produces the **canonical final board** — the finalize's `JSON_ORACLE`
line reports the KiCad-verified plane-completion verdict for the run.

### Step 2c: Tuned route parameters (the measured-optimal set)

A 15-board screen (2026-08-17) measured the following parameter set as
STRICTLY DOMINANT over each board's naive parameters — total KiCad
post-refill unconnected 62 → 23 across the corpus at **equal total wall
time** (better first-pass arrangement repays the extra search in saved
rip/retry churn). Apply it whenever the board is dense enough that any
fanout or escape is contested; on trivially-open boards the defaults are
fine.

1. **Strict small features** (the single biggest lever on packed boards —
   lane pitch is quantized by track+clearance, and a 0.1 grid cannot
   express a 0.28 pitch at fat features):
   `--track-width 0.0762 --clearance 0.0889 --via-size 0.25
   --via-drill 0.15`
   The fab-floor clamps pin track/clearance/via UP automatically on
   boards whose layer count or fab tier can't take them — passing those
   four is always safe. Grading stays honest via the `.kicad_pro` floor
   writeback.
   **EXCEPTION — `--hole-to-hole-clearance` does NOT clamp**: route.py
   board-derives h2h only when the flag is OMITTED and honors an explicit
   value verbatim, while `check_drc` pins its grade UP to the board's
   `min_hole_to_hole` — so an explicit 0.2 on a 0.25-constraint board
   routes real, graded drill-pair violations (verified in code by two
   independent plan audits). Pass the BOARD's own `min_hole_to_hole`
   (from `--design-rules`), or omit the flag and let route.py derive it.
2. **Direction preference**: this is now the DEFAULT (5), so passing
   `--direction-preference-cost 5` is optional — keep it if you want the
   manifest self-documenting. #663's corpus screen took the old 250 default
   to 5 on the strength of sets 1-5, 75 boards per arm at one commit: −22
   incomplete nets (−19.6%), W15/L6, real DRC flat. A weak nudge organizes
   layers; 250 priced every off-axis move above 3 vias and forced detours,
   while 0 loses the organization entirely.
3. **Layer pricing** (order matches `--layers`): GND solid-plane layer
   **6.0**; rail/split pour layers **2.5**; F/B and free routing layers
   **1.0**; and leave the board's **bus-highway layer at 1.0 even if it
   carries pours** — the inner layer adjacent to the largest BGA that its
   widest bus needs (orangecrab: In2, the RAM highway; pricing it cost
   completions every time it was tried).
4. **The plan/attraction environment** (route step only, as env-var
   prefixes on the command line so the manifest replays them):
   `KICAD_GLOBAL_PLAN=1 KICAD_GLOBAL_PLAN_SEQ=1
   KICAD_GLOBAL_PLAN_SEQ_COST=1.5 KICAD_GLOBAL_PLAN_VIA_COST=20
   KICAD_GLOBAL_PLAN_ITERS=50000 KICAD_GLOBAL_PLAN_ATTRACT=1
   KICAD_ATTRACT_POTENTIAL=65 KICAD_GLOBAL_PLAN_RIVER=1
   KICAD_FINALIZE_REAUDIT=1 KICAD_PACK_INLINE=1`
   (SEQ-negotiated global plan + potential attraction + river packing +
   finalize re-audit. These are env knobs today, so they ride the
   redo-manifest form of the plan but NOT the GUI plan JSON — see the
   promotion note in Step 9.)
   **On DENSE-tier boards (Step 5a-tuned gate), ALSO prepend**
   `KICAD_GLOBAL_PLAN_LAYER_MODE=clique KICAD_GLOBAL_PLAN_LAYER=pref` —
   clique-negotiated layer assignment with preference-directed layers.
   These were the knobs that unlocked the orangecrab hand-ladder's 22→15
   descent; on the skill's own plan they measured 17 vs 18 unconnected at
   equal wall time (within single-board wobble by itself, but
   directionally consistent and free). Leave them OFF for STANDARD
   boards — unmeasured there, and standard boards already hit 0.

### Step 2d: Guided iteration + endgame (dense boards)

When Step 2 leaves failures on a dense board, do NOT hand-tune — iterate:

```bash
# each pass re-attempts only the failed/open tail (connected nets
# gate-skip), with rip authority against the settled board; the plan
# guidance persists through rips, which is what makes iteration
# CONVERGE instead of plateauing
python3 -X utf8 py_router/route.py board_step2.kicad_pcb board_iter1.kicad_pcb --nets "*" <same flags+env>
python3 -X utf8 py_router/route.py board_iter1.kicad_pcb board_iter2.kicad_pcb --nets "*" <same flags+env>
```

Two iterations are near-free (measured: the tuned corpus run with 2
iterations baked in cost +1.5% total time) and historically descend
frontier boards 25→15 over 3–5 passes — so bake two passes into every
plan, dense or not. The near-free property DEPENDS on the Step 5a-tuned
density gate: connected nets gate-skip, but each pass still re-oracles
every pour, so big carve-free outer floods on a small board turn "free"
iterations into 100 s+ passes (one measured 428 s chain; 88 s with
the same iterations after the flood was removed). Then two endgame
signatures:

- **A net walled by its own protected diff partner** (`no rippable
  blockers` naming the partner): the #521 override — name BOTH members
  EXACTLY with `--nets P N --force-reroute`, then one more all-nets
  iteration (override passes may pay off one pass late).
- **Bare-ball / island signatures** are now handled automatically
  in-engine (bare-ball fanout rescue incl. the cap-move, per-island
  stitcher) — if a net still ships bare, check the rescue log lines
  before reaching for manual surgery.

#### Octolinear smoothing is ON by default -- leave it alone

`route.py` collapses grid-A* staircase micro-jogs into octolinear shortcuts
(#536) at the end of every route step, by default. Do not disable it, and do not
try to schedule it onto one step.

It was briefly defaulted OFF, on the theory that smoothing mid-chain removes the
staircase slack a later step's rip-up/rescue needs. Two boards supported that
(cubesat_backplane 10 -> 2 incomplete nets, spartan6_6layer 12 -> 7). A corpus
A/B refuted it -- one commit, 147 boards, only the knob differing:

| smoothing | incomplete nets | per board |
|---|---|---|
| ON (default) | 129 | 0.878 |
| OFF | 149 | 1.014 |

Turning it off costs ~20 nets: 34 boards worse, 10 better. Even spartan6, one of
the two boards that motivated the change, measures ON=6 OFF=9 there. So
smoothing is not starving the later passes -- on balance it helps them,
presumably by freeing corridor space.

`--no-smoothing` exists for A/B only (`KICAD_SMOOTH_ROUTE=0/1` overrides either
way). The lesson worth carrying: a two-board result is not a default change.

### Step 3: Finalize Planes — GND Return Vias + Stitching (only if wanted)
Skip this step entirely on low-speed boards. When the speed analysis calls
for GND return vias or area stitching, re-run `route_planes` with the SAME
nets/layers as Step 1 plus the via flags: an existing same-net zone on the
target layer is REPLACED in place (CLI default), and `--add-gnd-vias` places
return-current vias that adapt around the now-finished signals — the old
"stitching vias placed early block a diff pair's only channel" concern (#56)
is why these vias run HERE and not at the Step 1 pour. BGA plane balls
already carry their fanout-time plane-drop vias (#424), so this step needs
no tapping under a dropped BGA. It cannot rip anything either: the pour
places no taps, so `--rip-blocker-nets` is gone from `route_planes`. (For
the record, the measured failure mode of ripping here was routed signals
lost for tap pads the drops already serve. Reconnect anything a chain does
leave open with a follow-up `route.py` pass naming them,
using the same parameters as Step 2.)

> **Note to user:** GND return vias improve signal integrity for high-speed
> signals. Based on the speed analysis, this board has [speed_tier] signals,
> so `--gnd-via-distance` is set to [X] mm. If this is a purely low-frequency
> board (I2C/UART/GPIO only), drop `--add-gnd-vias`. Let me know if you'd
> like that.

python3 -X utf8 py_router/route_planes.py board_step2.kicad_pcb board_step4.kicad_pcb \
    --nets GND VCC \
    --plane-layers B.Cu F.Cu \
    --add-gnd-vias --gnd-via-distance 2.0 \
    2>&1 | tee /tmp/step3_planes.txt

Adjust `--gnd-via-distance` based on the board's highest signal speed:
- Ultra-high (>1 GHz): 2.0 mm
- High (100 MHz - 1 GHz): 3.0 mm
- Medium (10 - 100 MHz): 5.0 mm
- Minimum physical limit: 3 x (via_size + clearance)

### (No separate repair step — absorbed into Step 2, #562)
The old Step 5 (`repair_planes.py`) and its Step 5c reconnect
are **gone from the chain**: `route.py` finishes every run with the same
plane-repair engine (pad taps + region joins), the plane-copper cleanup,
and the KiCad-oracle exact-fill verify — and any oracle links its own
router can't route join the run's final reconciliation WITH rip authority,
so the old rip-then-reconnect two-step happens inside one invocation.
(Step 3 cannot rip — see its own note — so there is nothing to reconnect
after it.) `repair_planes.py` still exists
for repairing a board OUTSIDE this chain (e.g. a hand-edited board).

> **Never `cp` a board without its `.kicad_pro`.** A bare `cp a.kicad_pcb
> b.kicad_pcb` copies only the board and strands the sibling `.kicad_pro`, which
> holds the DRC floor (the Default-netclass clearance/track/via the chain routed to).
> The next routing step then reads no project, resolves its floor from the STOCK
> (looser) netclass, and its writeback stamps that looser floor over tighter copper —
> so KiCad grades correct sub-floor copper as phantom clearance violations (measured:
> a dropped 0.09 floor became 0.10 → 160 phantom grazes on one corpus board). Use
> **`python3 py_router/copy_board.py src.kicad_pcb dst.kicad_pcb`** — it copies the board plus every
> sibling (`.kicad_pro`/`.kicad_prl`) and self-records into the redo manifest — or, if you
> must use `cp`, copy the `.kicad_pro` too. The routing scripts also WARN when an input
> board has no sibling `.kicad_pro` (#441).

### Step 6: Verify Results
The final board is `board_step2.kicad_pcb` (or `board_step4.kicad_pcb` when
the optional Step 3 GND-via pass ran) — call it `board_final` below.
Invoke `/review-routed-board board_final.kicad_pcb` for the full review (DRC,
connectivity, orphan stubs, length-match tolerances, GND return via coverage,
diff pair checks). If that skill is unavailable, run the raw checks — `check_drc.py`
**auto-grades at the `.kicad_pro` clearance the routing steps wrote** (the smallest
clearance any step used, including auto-stepped fine-pitch taps), NOT a hardcoded
0.25, so legitimately-tight fine-pitch escapes that are still fabbable don't read as
violations (#111/#226). A bare invocation is correct; pass `--clearance <floor>`
(from Step 4's `--design-rules` output) only to override:

python3 -X utf8 py_router/check_drc.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_drc.txt
python3 -X utf8 py_router/check_connected.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_connectivity.txt
python3 -X utf8 py_tools/check_orphan_stubs.py board_final.kicad_pcb 2>&1 | tee /tmp/step6_orphans.txt
```

**Coverage gate (mandatory — close the loop on Step 5b).** `check_connected.py`
already lists every net with ≥2 pads but no copper and no covering zone as
"Unrouted net with N pads" (it accounts for plane zones and ignores genuine
single-pad / no-connect nets). After Step 2's in-run plane finalize, **this unrouted list must
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
### Step 1 (Alternative): Pour GND only
python3 -X utf8 py_router/route_planes.py board.kicad_pcb board_step1.kicad_pcb \
    --nets GND --plane-layers B.Cu

### Step 1b (Alternative): Fanout U9 Including VCC
python3 -X utf8 py_router/bga_fanout.py board_step1.kicad_pcb \
    --component U9 \
    --nets "*" "!GND" \
    --output board_step1b.kicad_pcb

### Step 2 (Alternative): Route ALL Nets, VCC as Wide Traces
python3 -X utf8 py_router/route.py board_step1b.kicad_pcb board_step2.kicad_pcb \
    --nets "*" \
    --power-nets GND VCC --power-nets-widths 0.3 0.5
```

VCC simply stays out of the pour assignments and rides the route step at
its wide power width; GND still pours in Step 1c and completes through the
route step's finalize like the main flow. If VCC wasn't fanned out, add
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

**Dense 2-layer boards: treat B.Cu as a real routing layer, not a plane.**
Reserving B.Cu for a GND plane (and/or pricing it 3×) turns a congested
2-layer board into single-layer routing — a dense 2-layer corpus board's human
original carries 47% of its routed length on B.Cu and pours GND *around* the
routes on both sides afterwards; a plane-first chain on the same board left 25
nets open. On a dense 2-layer board: route signals on BOTH layers at cost 1.0
(long-haul nets cross on the back), then pour GND last (`route_planes.py`
after the signal steps — the pour flows around existing copper; 80% of human
2-layer boards pour BOTH sides this way). Power rails as pours are a minority
practice on 2-layer (≈38% of human boards) — pour them too when there's room,
but GND-both-sides is the priority. Only plane-first on 2-layer boards with
light signal content.

**Important:** If you skip fanout for a BGA/PGA component but still need to connect its
internal pads, use `--no-bga-zone <component>` to disable the automatic exclusion zone
and allow the router to enter the dense pin area:

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*" \
    --no-bga-zone U9 \
    --output board_routed.kicad_pcb
```

Without this flag, the router auto-detects BGA/PGA zones and avoids them, which would
leave internal pads unconnected if they weren't fanned out.

### Multi-Layer Boards (4+ layers)

**Precedence: the Step 5a-tuned DENSITY GATE outranks this section.** The
survey below describes HUMAN practice, and our engine measurably matches it
only on DENSE boards. On STANDARD boards (no ≥100-ball fine-pitch array,
≤150 nets) the measured-optimal map is inner-only pours + rails as wide
traces (see the Step 5a-tuned A/B: four small boards, floods → 16 opens
total; inner-only → 0). Read this section as the Tier-DENSE playbook and
as background on why pours matter at all.

**Pour philosophy (from a survey of the human-routed corpus): pour EVERY GND
and power net that has more than a few pads, and treat pours as cheap.** Human
boards deliver power as copper pours, not tracks — a rail's ball/pad drops a
via straight into a pour instead of consuming a routed track through the
congested escape field. A dense 6-layer BGA board in the corpus that a
plane-light plan (GND-only, rails as wide tracks) left ~26% incomplete spends
~20% of its track copper on rails the human never routes at all. Concretely:

- **Which nets:** GND always, plus every power rail with more than a few pads
  — and pouring scales with layer count. Across ~400 human corpus boards,
  86% of 4-layer, 97% of 6-layer, and 100% of 8-layer boards pour power rails
  (2-layer boards: 38% — GND-only flood is the 2-layer norm). Humans pour
  even small rails (corpus median poured net ≈ 3 pads); a board with many
  rails gets many pours — human 4-layer boards commonly pour 5–15 distinct
  nets, dense 6-layer boards 10–20.
- **Which layers — any, including routing layers.** Pours are not confined to
  dedicated plane layers: 80–100% of human boards at every layer count pour
  copper ON their outer routing layers, flooding GND/rails around the
  finished tracks. The layer typology that recurs:
  - **4-layer:** signals+pours on F/B; one inner = solid GND; the second
    inner is a minority-solid choice (corpus: ~23% solid power, the rest
    SPLIT multi-rail (`/recommend-plane-mappings` Step 3b), route+pour, or
    plain routing when the board is dense and signals need to cross inner).
  - **6/8-layer:** solid GND planes nearest the outer signal layers (In1 and
    the last inner — 2/3 of human 6/8-layer boards have a solid plane
    adjacent to an outer layer), split power planes and/or route+pour in the
    middle, signals concentrated on F/B plus one inner "highway" layer.
- **High-speed nets need an UNSPLIT reference plane on the adjacent layer.**
  Whatever else moves, keep one solid (not split, not track-fragmented) GND
  plane directly under each layer that carries high-speed routes (DDR/RAM
  buses, USB HS, SerDes, RF — from `/find-high-speed-nets`). Split planes and
  route+pour layers are fine anywhere that isn't a high-speed reference.
- **Dense boards (BGA ≥ ~100 balls, DDR/SDRAM buses): keep escape-depth layers
  ROUTABLE, but still poured.** Don't let plane assignments turn the region
  around a big BGA into 2-layer routing — long-haul nets need to cross
  *through* inner layers (1–2 vias each). The resolution is order, not
  abstinence: solid planes pour FIRST (Step 1c); a layer signals must cross
  keeps its cost low (≤1.5) and gets its rail pours LATE (after the signal
  steps, like the 2-layer flow below — the pour flows around existing copper).
  Never leave a many-pad rail as pure tracks because its natural layer is
  shared with routing.
- **Check where the BGA fanout escapes landed before finalizing the plane
  layers** — a plane on a layer full of escape stubs leaves the route step
  threading its plane taps through a crowded field. Pick solid-plane layers
  the escapes avoid.

**Adapt the pour plan to the BOARD TYPE (measured across ~400 human corpus
boards, grouped by dominant component/function):**

| board type | human pour strategy |
|---|---|
| **Fine-pitch big BGA** (≥100 balls, ≤0.5 mm) | The most pour-heavy class: median poured nets 5 (4L) → 17 (6L) → 21 (8L). But median only ONE solid plane per board — a third have NONE. Keep nearly every layer routable (split + route+pour), pour many rails around the routes, and deliver every rail near the BGA by pour+via, never by tracks through the escape field. |
| **Big BGA, coarser pitch** (≥100 balls, >0.5 mm) | Same direction, milder: rails poured on 84% of boards, median ~6 nets; solid GND + split power inners are common. |
| **RF / radio** | GND-dominated: typically ONE GND net poured as many islands on every layer (coplanar grounding around RF traces); few rail pours. Keep GND pours tight to the RF path; rails as tracks are fine. |
| **Power / motor** | Heavy-current rails (V+, GNDPWR, phase outputs) are ALWAYS pours — 40–90-pad rails delivered as multi-layer copper regions, never wide tracks. Pour every supply rail on every layer it visits. |
| **Keyboard / LED matrix** | Humans pour nearly every net (rows, columns, LED chains) as small local zones on both sides of a 2-layer board. Our chain approximates this with: route both sides thin, then GND+rail pours; don't fight for a dedicated plane side. |
| **MCU / QFN, light 2-layer** | Modest: GND flood both sides (80%), rails poured on ~40–60%; a couple of pours is normal, don't force more. |

- More fanout options available.

**MANDATORY whenever any layer carries a solid plane: derive `--layer-costs`
from the plane plan and pass it to EVERY signal-routing step** (`route.py`,
the finalize's reconciliation, and retries). A measured failure mode: a 6-layer BGA
chain poured three solid inner planes and then passed NO `--layer-costs`
anywhere — signals crossed all three pours at cost 1.0, shredded them into
islands, and the board graded worse than a plane-light plan. Pour-first order
means the plane layers are already known when the signal steps run; there is
no excuse to omit this.

**Derive `--layer-costs` from the plane plan — penalize the plane-reserved
layers (issue #185).** The 4-layer default is **all 1.0**, so the router has no
idea which inner layers are about to become planes and freely routes signals
across them. Once you've decided the plane→layer map (via
`/recommend-plane-mappings` or the `route_planes` call you're about to make),
pass `--layer-costs` to the **signal** `route.py` step (and the later reconnect
passes) that makes each plane-reserved layer expensive, so signals prefer the
signal layers and leave the inner layers clean for the pour:
```bash
# GND plane on In1.Cu, power plane on In2.Cu -> penalize In1/In2 for signals:
py_router/route.py ... --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 3.0 3.0 1.0
```
- **~3× is the sweet spot on boards where F/B alone can carry the signals.**
  Any value ≥2× keeps signals off the planes and doesn't hurt completion; ≥5×
  just adds vias/copper for negligible further gain. Order matches `--layers`;
  keep the real signal layers (F.Cu/B.Cu) at 1.0. **On dense boards (BGA ≥
  ~100 balls / DDR buses) where an inner layer was deliberately left
  signal-routable (see the dense-board exception above), keep that layer at
  1.0–1.5** — 3× on the only spare layer starves the long-haul nets that need
  it (measured: a 4-layer FPGA corpus board failed 72 nets at 3×; its retry at
  1.5 was the correct call).
- **Why it matters — it's a cascade, not just tidiness.** Signals crossing a
  plane layer fragment the pour into islands; the route step's plane finalize then
  carpets the layer with island-stitching tracks. Keep signals off the plane
  layers and the planes stay whole, so the repair has almost nothing to stitch.
- **Measured on a 4-layer corpus board** (In1=GND, In2=+3.3V/+3.3VA), full chain,
  default `1.0 1.0 1.0 1.0` vs smart `1.0 3.0 3.0 1.0`, both fully connected and
  DRC-clean:

  | | default | smart 3× |
  |---|---|---|
  | total segments | 4857 | **2966 (−39%)** |
  | signal copper on plane layers | 307 mm | **44 mm (−86%)** |
  | vias | 309 | 318 (+9) |

  The 39% segment drop is the carpet disappearing because the planes stayed whole.

This is the 4-layer analogue of the 2-layer rebalance in best-practice #8 / #178:
in both cases derive the costs from how the layers will actually be used, rather
than taking the blunt default.

### Differential Pairs Present

Insert diff pair routing after fanout but before single-ended signals:

```bash
python3 py_router/route_diff.py board.kicad_pcb \
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
python3 py_router/qfn_fanout.py board.kicad_pcb \
    --component U1 \
    --output board_qfn.kicad_pcb
```

Creates two-segment stubs (straight + 45° fan) for each pad. On a crowded
fine-pitch edge where the surface fan has no room, add `--escape-method underpad`
(drop a through-via past each pad) and, if a boxed-in leg still drops,
`--allow-via-in-pad` so the via can sit on its own pad and stagger inward — see
"Crowded fine-pitch QFN edge" above.

Like `bga_fanout.py`, `qfn_fanout.py` ends with a `JSON_SUMMARY` carrying
`drc_grazes` (graded at `--clearance`). **Parse it after the fanout:** if
`drc_grazes.segment_segment > 0` the 45° escape stubs of two adjacent tight-pitch
pads (often a diff pair) are grazing at the wrist — re-run with a thinner
`--width` toward the fab floor until it's 0 (issue #179; see the `drc_grazes`
bullet under Step 1). All pads keep escaping (`failed` stays 0).

### Power Net Width Options

Instead of routing power separately, use `--power-nets` with signal routing:

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*" \
    --power-nets "GND" "VCC" "+3.3V" \
    --power-nets-widths 0.5 0.4 0.4 \
    --output board_routed.kicad_pcb
```

First matching pattern determines width. Useful when not using planes.

**Size power widths for the destination pitch, not just the current.** A
0.3–0.5 mm trunk physically cannot reach interior balls of a ≤0.8 mm-pitch
BGA (at 0.5 mm pitch only one ~0.09 mm track fits between balls; at 0.8 mm a
0.25 mm trace + 0.09 clearance is a knife-edge). The power step's automatic
tap neck-down helps at the pad, but if a rail feeds MANY interior balls
(core rails like +1V1/P1.35V/VCC_1V8), a fat-track tree through the ball
field fails outright — the human originals feed such rails with zones on
every layer plus 0.09–0.2 mm necks. For those rails prefer a plane/region
(`/recommend-plane-mappings`), or set the rail's width to what the ball
field admits (e.g. 0.15–0.2) rather than the open-field ideal.

### Target Swap Optimization (Memory Routing)

For swappable signals (e.g., memory data lanes where any DQ can connect to any):

```bash
python3 py_router/route.py board.kicad_pcb \
    --nets "*DQ*" \
    --swappable-nets "*DQ*" \
    --output board_routed.kicad_pcb
```

Uses Hungarian algorithm to find optimal assignments minimizing crossings.

### Schematic Synchronization After Swaps

When routing performs polarity swaps (P↔N) or target swaps, the schematic can get
out of sync with the PCB. Use `--schematic-dir` to automatically update:

```bash
python3 py_router/route_diff.py board.kicad_pcb \
    --nets "*LVDS*" \
    --swappable-nets "*LVDS*" \
    --schematic-dir /path/to/kicad/project \
    --output board_routed.kicad_pcb
```

This updates the `.kicad_sch` files with any pad swaps made during routing.

**Shared symbols are refused, not rewritten (#489 §3).** Pin numbers live in the
file's `lib_symbols` definition, which every instance of that `lib_id` shares. When
a second component uses the same symbol — the common case for connectors, identical
channels, and multi-channel analog — the swap is **refused** for that file with a
message naming the sharers, because applying it would silently re-pin those other
components too. The units of one multi-unit part (U2A/U2B) share the definition
legitimately and are still updated. A refused swap means board and schematic
disagree on those pins: report it and tell the user to fix it by hand (or give the
component its own uniquely-named symbol) before fabricating.

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
python3 py_router/route.py board.kicad_pcb --nets "SPI*" --guide-corridor --output board_routed.kicad_pcb
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
- The pour places NO tap vias and draws NO traces (#562), so it has no via-search or blocker-rip knobs: `--max-search-radius`, `--max-via-reuse-radius`, `--close-via-radius`, `--rip-blocker-nets`, `--max-rip-nets` and `--reroute-ripped-nets` are REMOVED. Do not emit them. Plane pads are welded by the route step's pour-launch and its in-run plane finalize.

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
python3 py_router/route.py board.kicad_pcb --nets "*" --verbose --output board_debug.kicad_pcb

# Debug geometry on User layers (visible in KiCad)
python3 py_router/route.py board.kicad_pcb --nets "*" --debug-lines --output board_debug.kicad_pcb


# A* search statistics
python3 py_router/route.py board.kicad_pcb --nets "*" --stats --output board_debug.kicad_pcb
```

### Post-Routing Enhancements

```bash
# Add teardrop settings to all pads (improves manufacturability)
python3 py_router/route.py board.kicad_pcb --nets "*" --add-teardrops --output board_routed.kicad_pcb
```

### Advanced Routing Parameters

For difficult boards, consider tuning these parameters:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--max-ripup 3` | 3 | Max blocking nets to rip up and retry |
| `--no-smoothing` | (smoothing is ON) | Disables #536 octolinear smoothing. A/B only — OFF measured ~20 nets worse across 147 boards |
| `--max-iterations 200000` | 200000 | A* base budget per route (self-extends to 1e7 while progressing — #529; don't tune) |
| `--heuristic-weight 2.3` | 2.3 | >1 = faster but may miss tight routes, 1.0 = optimal. 2.3 = the corpus dose-response peak (#586: 1.7 and 3.0 both worse; do not "tune it down for quality" -- measured, not intuitive) |
| `--via-cost 75` | 75 | Higher = fewer vias, longer paths; lower (25) for BGA escape. 75 = corpus-measured default (#586); 25 measured WORSE overall |
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
python3 py_router/route.py board.kicad_pcb --nets "*" \
    --stub-proximity-radius 2.0 --stub-proximity-cost 0.2 \
    --bga-proximity-radius 7.0 --bga-proximity-cost 0.2 \
    --track-proximity-distance 2.0 --track-proximity-cost 0.1 \
    --output board_routed.kicad_pcb
```

## Important Notes

0. **Net-coverage invariant (Step 5b)** - Every routable net must be claimed by a stage. Since #562 the route step takes `"*"` INCLUDING the plane nets, so the only legitimate exclusions are the Step-2b impedance nets; reconcile the exclusion set against that set (symmetric difference empty), check every poured net also appears in the route step's `--power-nets`, and confirm `check_connected.py`'s unrouted list is empty at the end. This is the guard against a net (e.g. a secondary ground like GNDA) being silently dropped by every stage.
1. **Always check for GND connections** - If a component has GND pads but GND isn't being fanned out, the plane vias will handle it
2. **Fanout ALL non-plane nets** - Use `--nets "*" "!GND" "!VCC"` to fan out all nets except those handled by planes. Do NOT use `"/*"` alone as it misses nets with non-hierarchical names like `Net-(U9-Pad1)`. Unconnected nets are automatically filtered out.
3. **Order matters** - Fanout (with plane-ball drops) comes AFTER the Step 1 bare pour (#424: planes FIRST, so the fill picks up the drop vias while intact and the fragility field steers every later route), then diff pairs, then the all-nets route with the plane nets INCLUDED (#562 — the run ends with the in-run plane finalize, so there is no separate repair step), then optional GND return vias/stitching. Signals route before stitching because stitching vias can relocate around tracks, but a diff pair cannot relocate around a badly placed via
4. **Verify at the end** - Always run DRC, connectivity, and orphan stub checks
5. **Consider the analyze-power-nets skill** - For complex boards where power net identification isn't obvious, use that skill first to analyze component datasheets
6. **Consider the find-high-speed-nets skill** - For accurate GND return via distance recommendations based on actual component datasheet speeds and rise times, run `/find-high-speed-nets` before planning. The lightweight inline analysis (Step 4) uses net name patterns only.
7. **Stub layer switching is on by default** - The router automatically moves stubs to eliminate vias when beneficial; disable with `--no-stub-layer-swap`
8. **Default layer costs** - 2-layer boards default to F.Cu=1.0, B.Cu=3.0 to prefer top layer; 4+ layer boards use 1.0 for all. On **dense** 2-layer boards this 3× back-side penalty can over-bias routing onto F.Cu (top channel exhausted, B.Cu empty, excess vias, stranded pads); if completion is low or the layer balance is badly skewed, **retry with more balanced `--layer-costs` (e.g. `1.0 1.5`, down toward `1.0 1.0`)** — see "Dense 2-layer boards: rebalance layer costs" under Diagnose and Retry (issue #178). On **4+ layer** boards the all-1.0 default is plane-blind: **derive `--layer-costs` from the plane→layer map and penalize the plane-reserved inner layers (~3×)** so signals stay on F.Cu/B.Cu and the planes stay whole — see "Multi-Layer Boards (4+ layers)" (issue #185).
9. **Schematic sync is disabled by default** - After routing with swaps, offer to re-run with `--schematic-dir` if the user wants to update their schematic
10. **Rip-up and reroute is automatic** - When a route fails, the router automatically rips up blocking nets and retries (up to `--max-ripup` blockers)
11. **Component shortcut** - Use `--component U1` to route all signal nets on a component (auto-excludes GND/VCC/unconnected)
12. **Use --no-bga-zone for difficult boards** - Even when fanout is complete, use `--no-bga-zone` during routing to allow the router to find alternative paths through the dense pin area. This is especially important for 2-layer boards where routing channels are limited.
13. **Windows UTF-8 encoding** - On Windows, use `python3 -X utf8` to avoid Unicode encoding errors when scripts print special characters (like Ω for resistance). Example: `python3 -X utf8 py_router/route_planes.py ...`
14. **BGA/PGA power pins and planes** - When using power planes, BGA/PGA power pins (GND, VCC) connect most efficiently via direct vias to the plane rather than fanout routing. Create planes first, then fanout only signal nets (this is the Step 1 -> 1b order). Through-hole PGA pads automatically connect to planes on that layer; SMD BGA pads need vias placed by `route_planes.py`. This approach:
    - Reduces routing congestion (power pins don't consume escape channels)
    - Provides lower impedance power connections
15. **Rip-up depth: MORE IS NOT BETTER (measured).** On a 6-board chain A/B, `--max-ripup 5` beat 10 (+0.78 pts completion, 13 fewer connectivity items, 3 boards better / 0 worse) and 20 was worse than 10 — each extra rip level risks a permanent casualty (a ripped victim whose corridor gets taken cannot be restored), and the gains from deep ripping don't materialize because victims can almost always reroute anyway. The optimum sits in 3-5 and wobbles by board (measured: one board monotone-better all the way down to 3, another best at 5) -- the SHIPPED default is 3 (the sets-11-15 holdout showed 5 hurting ordinary boards while helping knob-sensitive ones -- #586); try 5 as a free retry variant on difficult boards (deterministic: keep whichever grades better), and escalate above 5 only as a last resort on a specific failing net, never as the opening move. Do NOT add `--max-iterations` — the router self-budgets (#529 dynamic iterations, default on, up to a 1e7 ceiling while a search progresses); see the note in the routing-step section.
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
python3 -X utf8 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "*" 2>&1 | tee /tmp/route_output.txt
python3 -X utf8 py_router/route_planes.py input.kicad_pcb output.kicad_pcb --nets GND --plane-layers B.Cu 2>&1 | tee /tmp/planes_output.txt
python3 -X utf8 py_router/check_connected.py output.kicad_pcb 2>&1 | tee /tmp/connectivity.txt
python3 -X utf8 py_router/check_drc.py output.kicad_pcb --clearance <floor> --hole-to-hole-clearance <floor> 2>&1 | tee /tmp/drc.txt
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
- `failed_single`: List of single-ended nets with NO result at all
- `open_single`: Nets that KEPT a result whose copper still leaves pads disconnected (non-multipoint only). A run is clean only when `failed_single` AND `open_single` are both empty — a board can ship open copper with `failed_single: []`
- `terminal_restores`: `{net: outcome}` for rip victims restored at terminal failure — `full` is the only success; `full_open`/`stub` ship broken
- `stacked_copper`: Same-net duplicate copper KiCad's DRC never flags (disclosure only, not a routing failure)
- `failed_multipoint`: List of nets with unconnected pads (includes pad coordinates)
- `blockers`: Per still-failed net, which routed nets wall it off (`blocked_by` with cell counts; #409)
- `pad_pairs_connected`/`pad_pairs_total` + `pad_pairs_open`: Pad-pair routability tallies (PRR = connected/total) and per-open-net outcome — route-time failures are opens; shorts are DRC's domain (#409 follow-up)
- `multipoint_pads_connected` vs `multipoint_pads_total`: Connection success rate

### Tune mode (issue #153) — opt-in per-board feedback loop

When the user asks for **tune** (e.g. "plan routing with tune", "tune mode"),
don't just run the standard pipeline once with defaults: close the loop.
After EACH step, read the step's own diagnostics and adjust that board's
options before moving on. Off unless requested — the standard plan stays
deterministic and fast.

Rules of the loop:
- **Bounded, guided adjustment — not a grid sweep.** At most 2–3 targeted
  re-runs per step, each driven by a diagnosed failure mode (the symptom→knob
  table below and the failure-pattern table in Diagnose and Retry). Never
  loosen below the fab/board-constraint floor.
- **Signals to read after each step:** the `JSON_SUMMARY` line (failed nets,
  `rescue` block, `single_ended_diff_pairs`/`failed_diff_pairs`,
  `drc_grazes`), the FAILED NET HISTORIES block (`preexisting_blockers`
  hints, `no rippable blockers`, iteration exhaustion), fanout escape
  tallies (unescaped balls), and plane-step tap/`ripped`/`STILL FLOATING`
  reports. `/diagnose-routing-failures` automates most of this.
- **Symptom → knob map** (beyond the Diagnose and Retry table):
  - Fanout drops balls in one quadrant → re-run that fanout with
    `--escape-method dogbone` (`underpad` if no gap sites exist), a smaller
    via from the fab ladder (0.30/0.15 → 0.25/0.15), or different
    `--primary-escape` direction.
  - Signal step fails a cluster of long cross-board nets while an inner
    layer is plane-reserved → revisit the plane→layer map (dense-board
    exception above): free one inner layer, drop its `--layer-costs` entry
    to 1.0–1.5, re-run the failed nets.
  - `preexisting_blockers` hints repeat for the same nets → re-run those
    nets with the hinted `--rip-existing-nets` set (the engine now
    self-escalates once in reconciliation; a manual retry may widen the set).
  - Power multipoint pads fail inside a BGA courtyard → shrink that rail's
    `--power-nets-widths` entry toward the ball-field width (0.15–0.2) or
    promote the rail to a plane/region and re-run.
  - Diff pairs deferred single-ended → re-run the pairs with smaller
    `--diff-pair-gap`/width/vias toward the fab floor (keep `--impedance`).
  - The route step's plane finalize ships tap failures with fill nearby →
    re-run the route step at the advanced fab tier so smaller tap vias fit
    (or, outside the chain, `repair_planes` with a larger
    `--max-search-radius`).
  - A handful of nets fail on a NOT-saturated board (few failed nets, short
    detours available, failures share a corridor with early-routed nets) →
    try a **failed-first split**: re-run the step as two invocations, first
    `--nets <the failed nets>` on the clean input, then everything else to a
    fresh output. This is the one retry where naming nets is the POINT (you are
    changing the order), so accept it knowingly: a manifest that names nets
    cannot be A/B'd afterwards, because a replay hands the baseline a rescue
    fitted to its own failures and penalises every engine change that fails a
    different net. An ordinary retry should use `--nets '*'` instead -- route.py
    skips already-connected nets, so a wildcard retries exactly what is still
    broken. Ordering is the cheapest knob but rarely decisive:
    measured on four corpus boards of varying density, an automatic
    failed-first restart NEVER beat the normal order (twice it graded
    worse), so an in-engine restart was tried and removed — only reach for
    this manually when the failure histories actually show corridor
    competition, and expect it to matter on few boards.
- **Explainability:** keep a short tuning log per board — which knob changed,
  the before/after metric (completion / DRC / coupled pairs), and whether it
  helped. Revert a change that didn't help before trying the next.
- **Honest gates:** grade every accepted retry with `check_connected` AND
  `check_drc` at the routed clearance (plus the kicad oracle for final
  boards) — never accept a retry that trades new DRC for completion.

### End every chain on route.py

**A chain's LAST board is what ships, and only `route.py` finalizes planes.**
Since #562 a pour alone connects nothing and `route_diff.py` runs no plane
finalize: a chain that ends on a bare `route_planes.py` re-pour or on a diff
step writes a final board no weld/oracle pass ever verified. That is a PLAN
ERROR, not a tuning choice (set5-0805 evidence: dilemma and ghoul ended on a
bare re-pour and both shipped disconnected; core1106 simply stopped after a
failed retry). If you re-pour or re-run diffs late, ALWAYS follow with a
`route.py` step whose `--nets` covers the plane nets (even `--nets GND`
suffices — its in-run finalize welds and verifies against KiCad's fill).

### Diagnose and Retry

**Soft-cost retry levers (measured on 12-board challenging-chain A/B; these
are RETRY settings — the defaults stay mild on purpose):**

- **First-choice retry on any struggling board:** re-run the failing signal
  step (or chain) with `--ripped-route-avoidance-cost 3
  --track-proximity-cost 2` and **KEEP WHICHEVER RESULT GRADES BETTER** —
  routing is deterministic and the comparison is cheap. Across 12 hard
  boards this improved 8 (top gains +6.1 and +4.1 pts, connectivity down on
  nearly every win), regressed 2, and timed out 2 (expect up to 2× runtime).
  Board-type prediction is IMPERFECT — a 6-layer RAM board regressed −5.0 —
  so never blind-apply: always retry-and-compare.
- **Thrash-class variant:** on boards whose logs show heavy rip-up churn,
  ALSO try `--via-proximity-cost 100` on top (rescued one thrash board
  +2.4 pts where the base combo timed out) — but it fails more often than
  it helps elsewhere (3 wins / 5 losses / 3 timeouts); strictly a
  second-attempt lever, same keep-better rule.
- **Do not stack these with `--bga-proximity-cost` or a lower `--max-ripup`**
  — both combinations measured WORSE than either alone (they remove exactly
  the freedom the corridor pricing needs).
- **Boards routing fine at defaults:** leave everything alone.
- `--via-proximity-cost 0` now simply means "no extra via cost from
  proximity" (Rust 0.20.1 removed the old 0 = hard-via-ban mode, which was a
  measured ~200x CPU explosion) — safe, but rarely useful: the default 10 is
  what keeps vias out of escape fields. Leave
  `--ripped-route-avoidance-radius` at its default (widening it measured
  worse).

After running routing commands:
1. Report how many nets were routed successfully
2. **If routes failed**, invoke `/diagnose-routing-failures <board> <log files>` — it parses
   the JSON summary, failed-net histories, and blocking reports, correlates failures
   spatially, and outputs a targeted retry command. Apply its recommendation. If that skill
   is unavailable, fall back to this table:

| Failure Pattern | Likely Cause | Solution |
|-----------------|--------------|----------|
| "no rippable blockers found" | Route blocked by non-rippable obstacle | Use `--no-bga-zone`; if pads are "boxed in by static obstacles", shrink geometry / finer grid (see "Congestion escalation" below) |
| "Re-route FAILED: no path found" | Ripped net couldn't find new path | Capacity problem (`--max-iterations` self-extends, #529): `--max-ripup`, clearance, or layers |
| Many multipoint pads failed on same component | Congested area | Shrink geometry toward the fab floor (see below); keep `--max-ripup` at ~5 (deeper measured worse) |
| Many failures cluster in one channel/region | Tracks too fat for the channel | **Congestion escalation**: re-route the failed nets at smaller track/via/clearance down to the fab floor (see below) |
| 2-layer board: low completion, via count far above a hand layout, or copper badly skewed to F.Cu while B.Cu sits empty | Default B.Cu cost (3.0×) over-penalizes the back layer | Retry with balanced `--layer-costs 1.0 1.5` (down toward `1.0 1.0`) — see "Dense 2-layer boards: rebalance layer costs" below |
| Routes near BGA boundary failing | BGA exclusion zone too aggressive | Use `--no-bga-zone` |

```bash
python3 -X utf8 py_router/route.py board_prev.kicad_pcb board_routed.kicad_pcb \
    --nets "*" \
    --no-bga-zone \
    --max-ripup 5 \
    2>&1 | tee /tmp/route_retry.txt
```

   Key parameters for difficult boards (especially 2-layer with BGA/PGA):
   - `--no-bga-zone` - **Critical**: Allows router to enter BGA area for alternative paths
   - `--max-ripup 5` (default 3) - More rip-up attempts to resolve conflicts (measured optimum 3-5; deeper loses, see note 15)
   - Do NOT pass `--max-iterations` — self-budgeting (#529) extends hard searches to a 1e7 ceiling automatically; a post-extension failure is a capacity problem, not a budget one
   - `--stub-proximity-radius 10 --stub-proximity-cost 3.0` - Spread out fanout stubs (optional, for aesthetics)

#### Dense 2-layer boards: rebalance layer costs (issue #178)

On 2-layer boards the router defaults to per-layer costs **F.Cu=1.0, B.Cu=3.0**
(best practice #8) to keep most signal copper on top. But with a GND/power plane
already filling B.Cu, that 3× back-side penalty can over-bias routing onto F.Cu:
the top channel fills up while B.Cu sits nearly empty, the router takes long F.Cu
detours that then need a via to reach a B.Cu pad, and on congested boards the
exhausted F.Cu channel strands pads that B.Cu could have carried. This is the
dominant route-quality gap on tight 2-layer keyboard/peripheral boards.

**When to suspect it** (check the route `JSON_SUMMARY` / `comparison` block, or
measure per-layer copper length and via count against a reference):
- Strong F.Cu skew — e.g. >80% of signal copper on F.Cu while B.Cu is sparse.
- Via count far above a hand layout (the F.Cu-detour-then-via pattern).
- Low completion with failed pads clustered where F.Cu is full but B.Cu is free.

**Retry with more balanced layer costs** so the router crosses to B.Cu for short
diagonal runs instead of detouring on F.Cu (order matches `--layers`: F.Cu first,
B.Cu second):
```bash
python3 -X utf8 py_router/route.py board_fanout.kicad_pcb board_signal.kicad_pcb \
    --nets "*" \
    --track-width 0.127 --clearance 0.1 \
    --layer-costs 1.0 1.5 \
    --no-bga-zone --max-ripup 5 \
    2>&1 | tee /tmp/route_balanced.txt
```
(Mirror your Step 2 net selection and `--power-nets`/widths exactly in these
retries — the plane nets ride the route step in the pours-first chain, #562.)
Start around **`1.0 1.5`** (down from the `1.0 3.0` default); if F.Cu is still
saturated, step to **`1.0 1.2`** or fully balanced **`1.0 1.0`** (fine when a
plane fills B.Cu — signals carve the pour and it reflows around them). This is
**complementary to**, not a replacement for, routing at the fab floor (below): a
balanced layer that's still too fat won't fit the channel either, so keep
`--track-width` thin. Re-route the **whole** signal step, not just the failures (a
victim is blocked by the successful F.Cu tracks already in its channel). Then
compare completion, via count, and F.Cu:B.Cu balance, and keep whichever connects
more pads with fewer vias.

Measured at `--track-width 0.127` (B/F = B.Cu:F.Cu copper-length ratio; both
boards stay 100% connected at every setting — the win is via count and balance):

| board | default `1.0 3.0` | `1.0 1.5` |
|-------|-------------------|-----------|
| 2-layer corpus board A | B/F 0.17, 177 vias | **B/F 1.01, 98 vias** |
| 2-layer corpus board B | B/F 0.19, 102 vias | **B/F 1.85, 59 vias** |

`1.0 1.5` roughly **halves the via count** and pulls the layer balance from a
~6:1 F.Cu skew to near parity (board A's human layout sits around B/F 0.89).
`1.0 1.0` lands in the same neighbourhood — pick the one with fewer vias.

#### Route signals at the FAB floor by default (thin is faster AND more complete)

**`track_width` and `via_diameter` are NOT DRC floors** (Step 4), and — this is
the subtlety — **the fab floor is NOT the board's `min_track_width` constraint
either.** Three different numbers get confused here; keep them straight:

- **Board `min_track_width`** (from `.kicad_pro`, often 0.2 mm) — the
  author's self-imposed DRC rule. Often conservative. Note `list_nets
  --design-rules` reports its "manufacturing floor" track as `max(this, JLC min)`,
  so it currently **clamps the track floor to this constraint** (0.2) and does NOT
  surface the finer fab capability — do not treat that printed track number as the
  real floor (it's right for clearance/via, just not for track).
- **Fab physical track minimum** (JLC ≈ **0.0889 mm / 3.5 mil** standard; **0.127
  mm / 5 mil** is the safe no-extra-cost width) — the actual floor. **This is the
  target.** It can be *below* the board's `min_track_width`: human corpus
  boards routinely route most signals at 0.127 mm, under their own 0.2 mm constraint,
  which is exactly why they fit channels our 0.2 mm net-class tracks can't.

For ordinary signals there is **no benefit to routing fat** and a real cost.
Measured on a 4-layer corpus board (signal pass, same clearance/grid, width only):

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
   python3 -X utf8 py_router/route.py board_fanout.kicad_pcb board_signal.kicad_pcb \
       --nets "*" \
       --track-width <fab floor, e.g. 0.127 or 0.0889> --clearance <floor, e.g. 0.1> \
       --via-size <floor via, e.g. 0.30> --via-drill <floor drill, e.g. 0.15> \
       --no-bga-zone --max-ripup 5 \
       2>&1 | tee /tmp/route_signal.txt
   ```
   A finer `--grid-step` (0.05, or 0.025 AT ≤0.4 mm pitch — a part *at* 0.4 mm
   needs 0.025: "sub-0.4" reads as excluding it, and measurement says otherwise)
   is the complementary
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
> - board_step1.kicad_pcb (after fanout)
> - board_step1c.kicad_pcb (after GND/VCC pours)
> - board_step2.kicad_pcb (after the all-nets route + in-run plane finalize)
> - board_step4.kicad_pcb (after GND return vias, if run)
>
> The final routed board is: board_step2.kicad_pcb (or board_step4 if GND vias ran)
>
> Would you like me to delete the intermediate files?"

## Step 9: Emit the plan as an executable artifact (always)

The plan is not finished as prose — it ships as TWO machine-loadable files
next to the board, so the exact tuned choices replay with no LLM:

1. **`<board>_plan.sh`** — every board-mutating command from the plan, one
   per line, fully quoted, in execution order, env-knob prefixes included
   (the redo_commands.sh format; `tests/stress/redo_stress_test.py` replays
   it verbatim). This is the authoritative form: it carries EVERYTHING,
   including the Step 2c environment block and iteration passes.
   **Begin the file with an explicit `cd <repo>` line** (not just a
   `# cwd=` comment) so a bare `bash <board>_plan.sh` works — relative
   `py_router/` tool paths fail from any other directory.
   **Verification commands go in as COMMENTS, never executable lines** —
   the file's exit code is read by executors as the chain verdict, and a
   trailing read-only check that exits non-zero (violations found, or a
   wrong relative path) makes a chain whose `final.kicad_pcb` is perfectly
   good report as failed (measured: half the 15-board wave reported
   rc≠0 from exactly this). The last executable line must be the final
   `route.py`.
2. **`<board>_plan.json`** — the GUI AI-tab form:

   ```bash
   python3 tests/stress/manifest_to_plan.py <board>_plan.sh -o <board>_plan.json
   ```

   The AI tab's Load button accepts this file and the plan executor runs
   the same chain through the GUI engine path. KNOWN LIMITATION: env-only
   knobs (the Step 2c plan/attraction stack) do not survive into the JSON —
   only `--flag` params map to plan params. Until those knobs are promoted
   to first-class engine params + dialog controls (the #513 discipline),
   the `.sh` form is the one that reproduces the tuned result exactly; say
   so in the summary when the two forms differ.

A per-board plan file that ever gets improved (a better recipe found on a
later pass) should be REGENERATED through this skill, not hand-patched —
the skill is the tuner; the plan file is its output.

## Step 10: Plan-authoring precedence rules (resolve these conflicts explicitly)

Lessons from a dry-run audit (an agent following this skill end-to-end):

1. **No stackup ⇒ no impedance passes, period.** When the board has KiCad's
   default stackup, SKIP every `--impedance` step (including the DDR SSTL
   40Ω pass) and lead the plan with the /recommend-stackup warning. The
   no-stackup rule OUTRANKS every interface-specific impedance
   recommendation.
2. **Populated-array escape:** `dogbone` supersedes the older
   "channel-infeasible → underpad" advice for populated BGAs; underpad is
   for WLCSP/inner-row cases where no inter-pad gap exists at all.
3. **Fanout `--layers` must EXCLUDE any layer carrying a solid plane**
   (e.g. the In1 GND plane) — escapes on the solid plane shred it, and
   the fanout does not avoid poured layers on its own.
4. **The route flag is `--no-bga-zones` (plural).** The singular spelling
   fails argparse.
5. **No pipes in the emitted plan.sh.** `2>&1 | tee ...` is for
   interactive runs only — `manifest_to_plan.py` tokenizes pipe segments
   into net globs and corrupts the JSON. Plain redirects or nothing.
6. **`--max-ripup`: 5 on dense boards** (any fine-pitch BGA present or
   >150 nets), else leave the default 3.
7. **A "pair" whose far end is bare test points** may stay in the diff
   step (the peel machinery degrades gracefully) — note it in the plan
   rather than agonizing.
8. **The Step 5a-tuned density gate outranks the pour-philosophy survey.**
   Outer-layer floods, per-zone fragility `=0`, and rail co-pours are
   DENSE-tier moves only; a STANDARD board gets inner-only pours and rails
   as wide traces, no matter how many pads a rail has (measured: the four
   flood-regressed wave boards all went to 0 opens and 3–5× faster on the
   inner-only map).
9. **Never split more than 2–3 rails onto one Voronoi layer** — prefer
   wide traces for the overflow (measured: six rails on In2 → +3V3 in 8
   islands, 7 opens).
10. **Stitch/GND-via tail only at high speed tier or above** (from
   `/find-high-speed-nets`) — and when it runs, the chain still ends on
   `route.py`. Low-speed boards end at the Step 2 route (plus Step 2d
   iterations) with no re-pour tail.

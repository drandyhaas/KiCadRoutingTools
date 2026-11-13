# QFN/QFP Fanout Generator

Creates escape routing for QFN (Quad Flat No-leads) and QFP (Quad Flat Package) packages in KiCad PCB files.

## Features

- **Generic package support** - Works with any QFN/QFP package regardless of pin count or size
- **Automatic geometry detection** - Analyzes pad positions and sizes to determine layout
- **Rotation-agnostic** - Edge classification runs in the footprint's local frame and the
  escape follows each pad's own long axis, so packages placed at any board angle (including
  non-orthogonal, e.g. an LQFP144 at -135°) fan out correctly. The previous global-bounding-
  box edge detection saw a rotated part as a diamond and escaped almost none of its pads.
- **Two-segment stubs** - Straight segment + 45-degree fan-out for optimal endpoint separation
- **Side detection** - Automatically determines which side each pad is on
- **Neighbour escape room** - A fan stops at the midline of any gap it shares with
  another chip's pins (a QFN, QFP or two-row IC with at least 8 perimeter pads):
  each chip's would-be stubs, cut at that midline, are copper the other's fan must
  clear. Only the 45-degree fan yields; the straight escape never does. Before
  this, whichever chip the chain fanned FIRST took the whole gap -- on rein_r1
  (two RP2040s, pin rows 1.7 mm apart) the second chip's USB pair could not escape
  in one order and coupled in the other. The result is now the same in either
  order (`tests/test_qfn_neighbour_escape_room.py`).
- **A stub ends on a same-net pad it reaches** (#1332) - every clearance test skips
  same-net pads, so a fan could clip its pin's own decoupling cap and run on past it,
  leaving a dangling tail route.py keeps as input copper. Each stub is now walked
  against the exact pad copper and ends at its deepest contact with the first
  same-net pad it overlaps. If that contact joins through a web below the board's
  connection-width floor (a corner graze), a short closing segment runs into the
  pad -- the same connector close_soft_joints adds (#416), decided by the same
  KiCad-exact erosion -- and only if it clears every foreign object. Such a stub has
  no tip for the router to launch from, so it is left out of the endpoint check.
- **Collision validation** - Checks endpoint spacing after generation
- **Pad-geometry sanity check** - Runs `check_pads.py` on the component first; if its pads
  overlap (a sign the pad rotation/size is modelled wrong) it warns before escaping.
- **Board rules honoured** - a `.kicad_dru` clearance rule for the escape layer replaces
  `--clearance` exactly (the stubs live on one layer, #498); stubs and vias keep the board's
  copper-to-edge clearance; a `--width` below the board's fab track floor is clamped up to
  it (#223); the project's same-net-pad record (#581) is read.
- **Auto-pick** - with no `--component`, the QFN (else QFP) candidates lose connector and
  marker parts and are ranked by name (QFN/QFP/DFN/MLF in the footprint name), then pin
  count; the ranking is printed.
- **Not a QFN, said so** - a part whose pads span a single line (an edge-finger row, a
  single-row header) is refused by name and nothing is written (#1195).

## Usage

```bash
# Basic fanout
python3 py_router/qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb

# With net filter (include pattern)
python3 py_router/qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb --nets "*DATA*"

# With exclusion pattern (all nets except GND and VCC)
python3 py_router/qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb \
    --nets "*" "!GND" "!VCC"

# Specify layer and track width
python3 py_router/qfn_fanout.py kicad_files/input.kicad_pcb --component U1 --output kicad_files/output.kicad_pcb \
    --layer F.Cu --width 0.15
```

## Net Pattern Syntax

The `--nets` option supports fnmatch-style wildcards (`*`, `?`) and exclusion patterns:

| Pattern | Description |
|---------|-------------|
| `*DATA*` | Nets containing "DATA" |
| `/*` | Nets starting with "/" (hierarchical) |
| `!GND` | Exclude net named "GND" |
| `!*VCC*` | Exclude nets containing "VCC" |
| `"*" "!GND" "!VCC"` | All nets except GND and VCC |
| `D[3]` | The net named `D[3]` (a pattern that spells a board net is that net, not a `[...]` class, #1641) |

**Note:** Nets starting with "unconnected-" (KiCad pins not connected in schematic) are automatically excluded,
and so is any net with **a single pad on the whole board** (#1342) -- a spare pin the schematic still names
(an unused DPHY/HDMI/SD lane on a CM4/CM5 module) has nothing to connect to, so its escape would be copper
that only blocks routing. This is bga_fanout's #122 rule (`single_pad_net_ids`), applied whatever `--nets`
says; the run prints `Skipping N single-pad/NC net(s)` and `JSON_SUMMARY` carries `skipped_nc`. Such a net
counts as neither requested nor failed.

**Selected nothing (#1641):** each `--nets` pattern that selects no pad on the part is printed and listed
in `JSON_SUMMARY`'s `unmatched`, with its `reason` (`not_on_part`, `single_pad`, `unconnected`, `no_such_net`); `requested` counts only what a
pattern selected. A `--nets` list none of whose patterns names a net on the board is refused, exit 2, with no
board written, as `route.py` refuses one.

**Interior pads a filter selected (#1717):** the surface fan has no edge for an interior pad, so a stub run
with `--nets` gives each interior pad its filter selects the under-pad via-drop after the fan (the
`--escape-method underpad` code, its clearance ladder, `--via-size`/`--via-drill`/`--allow-via-in-pad`), placed
clear of the fan's stubs. A pad the via-drop cannot place fails its net, as an underpad run does, and is
named in `JSON_SUMMARY`'s `skipped_interior` (`"U1.34"`) with one `WARNING` line. A run without `--nets` never
via-drops an interior or exposed pad and lists nothing.

**Power stubs drawn at `--width` (#1743):** the via-drop draws every pad-to-via stub at `--width`, and
`--power-nets` widens only a surface stub (where it fits, #1318). So `JSON_SUMMARY`'s `power_stubs_at_width`
lists each escape stub drawn at the track width on a net that reads as power or ground -- `pad` (`"U1.5"`),
`net`, `width` (as drawn), `escape` (`surface` or `via-drop`) and `why` -- with a console `NOTE` saying what
widens it. A net reads as power by the rules autoroute builds `--power-nets` from (`list_nets.
power_net_evidence`): a ground or rail name, power pins on two or more parts, or a pour the board draws for it.
A surface stub of a net `--power-nets` names is not listed; a via-drop stub is, while its named width is wider
than drawn. Disclosure only: the copper is unchanged.

**The via a run sizes itself (#1775, #1795):** with no `--via-size`/`--via-drill` the via-drop's via starts at the
board's Default net-class via, as on `route.py` and the GUI fanout tab (0.45/0.25 when the board declares
none), raised to the board's `.kicad_dru` via rules (#1676) and, under `--escalation board` only, to its
Board Setup `min_through_hole_diameter`, `min_via_diameter` and `min_via_annular_width` round the drill (a
0.13 ring raises 0.45/0.25 to 0.51/0.25). Under the default `fab` it keeps its size; with `--no-fix-drc-settings` a via
laid below a Board Setup minimum is named in a `WARNING` line and in `JSON_SUMMARY`'s `board_setup_vias`
(`rule`, `min`, `vias`, `below`, `smallest_ring` / `smallest`). bga_fanout.py does the same, from the
same class via (0.5/0.3 when the board declares none).

## Options

| Option | Description | Default |
|--------|-------------|---------|
| `--component`, `-c` | Component reference | Auto-detect QFN/QFP |
| `--output`, `-o` | Output PCB file | kicad_files/qfn_fanout_test.kicad_pcb |
| `--layer`, `-l` | Routing layer for the stubs; any layer but the part's own warns, since the stubs then miss its SMD pads (#96). GUI: the QFN panel's **Escape layer** | the layer the part is mounted on |
| `--width`, `--track-width`, `-w` | Track width (mm) | 0.1 |
| `--extension` | Extension past pad edge before bend (mm) | 0.05 |
| `--clearance` | Clearance to other-net copper (mm). Omitted, it is the board's **Default net-class** clearance, as on `route.py` (0.1 only when the board declares no Default class); `--clearance-ceiling` caps it. GUI: the QFN panel's **Clearance** box, unchecked = the board's value (#1328) | the board's Default class |
| `--nets`, `-n` | Net patterns to include | All nets |
| `--grid-step` | Routing grid step (mm); stub ends are snapped to it (#149). Match the `--grid-step` you give `route.py` | 0.1 |
| `--escape-method` | `stub` (surface 45° fan) or `underpad` (via-drop); with `--nets`, `underpad` also escapes **interior (under-body) pads** (issue #410) | stub |
| `--via-size` | Underpad escape via outer diameter (mm) | the board's Default class, else 0.45 |
| `--via-drill` | Underpad escape via drill diameter (mm) | the board's Default class, else 0.25 |
| `--board-edge-clearance` | Stub/via copper to the Edge.Cuts outline (mm); stubs are shortened or dropped, underpad vias near the edge rejected (#288) | the board's `min_copper_edge_clearance`, else `--clearance` |
| `--allow-via-in-pad` | Underpad escape: let the via overlap its **own** pad so it can stagger *inward* (via-in-pad) | off |
| `--same-net-pad-clearance` | `> 0` forbids via-in-pad (overrides `--allow-via-in-pad`) and is recorded in the sibling `.kicad_pro` for later steps (#581); `-1` allows it | the project's record, else allowed |
| `--add-teardrops` | Add teardrop settings to all pads and vias in the output | off |

The fab floor and escalation flags every routing step shares (`--fab-tier`,
`--fab-overrides`, `--escalation`, `--clearance-ceiling`, `--strict-sizes`) and
the post-run DRC-settings flags (`--no-fix-drc-settings`,
`--relax-drc-severities`, `--keep-thermal`, `--enable-used-layers`) are
described in [Configuration](../../docs/configuration.md#fab-tier-options).

### Clearance: the board's own, escalating only where a pad needs it (#1328)

An omitted `--clearance` is the board's Default net-class clearance, capped by
`--clearance-ceiling` -- the reading `route.py` gives its own flags, through one
resolver (`resolve_qfn_clearance`) that the GUI QFN panel calls too. It used to
default to 0.1, and since a fanout's writeback clamps every net class to the
clearance it escaped at, a chain's `--clearance-ceiling 0.15` lowered the
Default and Power classes 0.15 -> 0.1 and every later step routed the board at
0.1, while the same part escaped every pad at 0.15.

A surface stub whose straight escape cannot clear at the run clearance used to
be dropped. When `--escalation` allows a descent (the default, `fab`), it is
retried down a ladder (`qfn_clearance_ladder`: 0.005 mm steps plus the tier's
own rungs, to the deepest rung `fab_tiers.escalation_rungs` allows -- the fab
floor under `fab`, the board's declared minimum under `board`, nothing under
`off`) and keeps the first clearance it clears at; its 45-degree fan is
shortened at that same clearance. A `.kicad_dru` rule on the escape layer
states that layer's clearance, so a ruled layer does not escalate.

Each escalated stub is disclosed the way `route.py` discloses a descent: a
`clearance` row (site `qfn escape`) in `JSON_SUMMARY.design_rules` and the
end-of-run `Design rules [...]` line, a fab-tier escalation when the rung is
below the tier's nominal floor, and `JSON_SUMMARY.escalated` (pad, net, rung).
The project floor written back is the lowest rung a stub actually cleared at --
never the fab floor the ladder was allowed to reach -- and a Default class it
lowers is recorded and announced by the writeback's descent record (#1160), so
the next step says so. A run in which nothing escalates writes the run
clearance, as before. `drc_grazes` is graded at that written floor.

### Net classes: spaced pairwise, and kept (#1529)

The run clearance is the Default class. Every other class is read from the
board's `.kicad_pro` (the GUI reads the live board's), as `route.py` reads it,
and every pair the escape tests -- a stub or via against foreign copper, a pad,
another fanned net's copper, a neighbouring chip's reserved escape room, this
run's own stubs -- is priced at KiCad's `max(clearance, class A, class B)`,
which a `.kicad_dru` rule on the layer replaces. A stub escalates (#1328) no
lower than its own class. The run prints `Netclass clearances for N net(s)
...` when the board has any.

The writeback then treats the classes as `route.py` does: kept when
`--clearance-ceiling` is omitted, each capped at `min(class, ceiling)` when it
is given. A class is never lowered to the clearance the escape ran at, so a
class whose nets the part does not touch keeps its value. `drc_grazes` is
graded at the written project's classes too. (`bga_fanout.py` does the same;
see its README.)

### Class- and type-scoped `.kicad_dru` rules (#1565)

A `.kicad_dru` clearance rule scoped by net class or item type (the #1491
channel, e.g. `A.NetClass == 'HV' && B.NetClass == 'HV'`) and a track-scoped
class rule (#735) price every pair the escape tests, through the same readers
the route steps use (`kicad_dru.install_pair_rule_channels`): the obstacle maps
stamp foreign copper at the rule, and the pairs above are priced for the two
items' types, a track rule on a track pair pair-exact. A rule holds at every
escalation rung. The run prints `.kicad_dru: N class/type-scoped ... clearance
rule(s) held by this escape (#1565)`; a board with no such rule is unchanged.

### Under-pad (via-drop) escape — `--escape-method underpad` (issue #164)

The default `stub` fan spreads each pad laterally at 45°, so on a **crowded
fine-pitch edge** (a neighbour pair on one side, a foreign track on the other)
it runs into them and the stub is dropped. `--escape-method underpad` instead
drops a **through-via just past each pad** and lets signal routing pick the net
up on an inner/back layer — going *straight out* past the lateral congestion
rather than fanning into it. Adjacent vias are **staggered** along the escape
axis so two neighbours one pitch apart still clear (e.g. 0.45 mm vias on 0.5 mm
pitch escape with centres ~0.56 mm apart). Each via **and its stub** are
obstacle-checked against foreign tracks/vias/pads — and "foreign" means *any net
other than the one being escaped, even on the same chip* (a routed neighbour
pair, a crossing track), so the via can't land on a neighbour's copper and short
it (issue #161). A via that can't find a clear offset is dropped and reported.
Pair the via with `--via-drill` to keep a sane annular ring (e.g.
`--via-size 0.45 --via-drill 0.25`).

Placement is greedy per edge, but if the default nearest-first stagger **drops**
a leg the edge is re-tried under alternative stagger configurations — reversed
order, and per-leg direction biases that send one leg back while its neighbour
goes forward — keeping whichever escapes the most legs (issue #161 follow-up).
The default is tried first, so an edge that already escapes fully is unchanged;
the alternatives only kick in to rescue an otherwise-dropped leg.

#### `--allow-via-in-pad` (issue #161)

On a genuinely boxed-in pair (the outer leg has a neighbour pad one pitch away
*and* the neighbour's diagonal escape sweeping through the only outward room) the
via has nowhere to go outward, so the plain escape **drops** it. With
`--allow-via-in-pad` the escape via may sit on its **own** pad (via-in-pad), and
the search gains an extra ladder of **signed offsets along the escape axis**,
tried *before* the outward one, plus four extra stagger configurations. So each
leg independently takes whichever escapes — a boxed-in leg staggers *inward*,
away from the neighbour, instead of being dropped, while a leg with outward room
still goes out. The via still must clear every other-net pad, via and track — it
only gains permission to overlap its own pad.

**That inward ladder is not confined to the pad (#846).** Its increment is the
*inter-net* stagger — the centre-to-centre a via needs from a **different** net's
via at this pitch — which on a fine-pitch part is larger than the pad. On
`routed_output`'s QFN-76 (pitch 0.40, via 0.45, clearance 0.1) the step is
0.4275 mm against a pad whose escape-axis extent is 0.875 mm, so rung 1 is
already half a pad-length off centre and rung 8 reaches ±3.42 mm. Measured, the
flag's rungs are load-bearing: with it on, this ladder won 66 of 84 accepted
offsets on `routed_output` U2 and every one on `tigard`, `qfn_underpad_coupling`
and `qfn_diffpair_escape`, and it owns the longest emitted stub there (3.0125 mm;
the ladder asks for a 2.9924 mm offset at that rung, before the via is snapped to
the routing grid). So the offsets stay and the name was the wrong half.
`KICAD_QFN_ONPAD_REACH` confines them: `pad` keeps the via centre on the pad,
`barrel` the whole barrel; `full` (the default, and any unrecognised value) leaves
the ladder whole.

A via that **does** overlap its pad is clamped to the pad edge so it cannot bulge
past it (#202), and needs IPC-4761 Type VII (filled + capped + plated) — the run
prints a FAB NOTE naming the pads. `JSON_SUMMARY` carries `allow_via_in_pad`,
`via_in_pad`, `via_in_pad_clamped`, `via_in_pad_offcentre` and `max_stub_mm`.
Name and behaviour match the "Allow via-in-pad" option elsewhere in the tools.

A pad that already has its via escape gets no second one (#1804): a same-net
via reaching the escape layer either overlaps the pad's copper or is joined to
it by one same-net track on the pad's layer (`pad_has_via_escape`). Such a pad
on the input is left out before the escape runs (the run prints `N pad(s)
already have a via escape, skipped`), so the step re-run on its own output adds
nothing and a retry tries only the pads still unserved; a pad stacked with
another (an exposed pad's rectangle and the thermal hole at its centre) whose
copper a via of the same run already overlaps is served by it, not dropped.
The via-drop's PADS are in `JSON_SUMMARY` beside the per-net counts:
`pads_requested` = `pads_escaped` + `pads_already_served` + the dropped pads,
named in `dropped_pads` (`"U1.57"`); `pads_reused` (bridged to an existing
via) and `vias_placed`. A net whose selected pads were all served counts as
escaped.
Example:

```bash
python3 py_router/qfn_fanout.py board.kicad_pcb --component U1 --nets 'DP1*' \
    --escape-method underpad --allow-via-in-pad \
    --via-size 0.45 --via-drill 0.2 --clearance 0.15 --grid-step 0.05
```

#### Interior pads — `--escape-method underpad --nets ...` (issue #410)

Some packages carry pads **under the body** that are not on the perimeter:
AQFN inner rings, and connector-style parts whose signal pins sit surrounded by
ground pads. The surface 45° fan can never escape these (there is no free edge
to route from), so the fanout classifies them `center` and skips them. A
via-drop needs no free surface edge, though — the via goes straight down — so
on a run that is **both** `--escape-method underpad` **and** scoped with
`--nets`, interior pads are escaped like any other pad, each along its own
long-axis direction.

Requiring `--nets` is a deliberate safety guard: an unscoped run never
via-drops the exposed/thermal pad. The EP escapes only if your filter
explicitly matches its net (e.g. `--nets 'GND'`), which is a valid, deliberate
choice — not a default. Interior pads are usually boxed in by their neighbours,
so pair this with `--allow-via-in-pad`; the via then sits centred on the pad
and no stub is emitted at all. Example:

```bash
python3 py_router/qfn_fanout.py board.kicad_pcb --component U1 --nets 'INT*' \
    --escape-method underpad --allow-via-in-pad \
    --via-size 0.45 --via-drill 0.2 --clearance 0.15 --grid-step 0.05
```

## Module Structure

```
qfn_fanout/
├── __init__.py    # Main fanout logic and public API
├── layout.py      # Layout analysis functions
├── geometry.py    # Stub position calculations
└── types.py       # Data types: QFNLayout, PadInfo, FanoutStub
```

### Key Components

- **`generate_qfn_fanout()`** - Main function that generates fanout tracks
- **`analyze_qfn_layout()`** - Extracts package parameters from pad geometry
- **`analyze_pad()`** - Determines pad side and escape direction
- **`calculate_fanout_stub()`** - Calculates two-segment stub positions
- **`QFNLayout`** - Package layout parameters
- **`PadInfo`** - Analyzed pad information
- **`FanoutStub`** - Represents a fanout stub with two segments

## Output

The fanned board is written to `--output` (passed through unchanged, project
and all, when nothing was drawn), and the run ends with a `JSON_SUMMARY:` line:
`requested` / `escaped` / `failed` / `unescaped_nets` (a stub-less via-in-pad
escape counts as escaped), `clearance`, `track_width`, `escape_method`,
`via_size`, `via_drill`, `layer`, `drc_grazes` (a `check_drc` pass over the
output at `--clearance`: `pad_via`, `via_segment`, `pad_segment`,
`segment_segment`, `total`; #179), `min_clearance_used`, `allow_via_in_pad`
(plus `via_in_pad`, `via_in_pad_clamped`, `via_in_pad_offcentre`,
`max_stub_mm` for a run that via-dropped), the per-pad `pads_requested` =
`pads_escaped` + `pads_already_served` + the pads named in `dropped_pads`,
with `pads_reused` and `vias_placed` (the surface fan's pads and the
via-drop's together), `env_knobs`, and `design_rules` when a
feature was delivered below its requested size. Unless `--no-fix-drc-settings`,
the output's `.kicad_pro` DRC floors are lowered to what the fanout drew (#160);
a run that drew no copper writes no floors, and the via floors are written only
when it drew a via.

## Algorithm

1. **Layout Analysis** - Extract package center, dimensions, and pitch from pad positions
2. **Pad Classification** - Determine which side (top/bottom/left/right) each pad is on
3. **Escape Direction** - Calculate perpendicular escape direction for each pad
4. **Stub Generation** - Create two-segment stubs:
   - Straight segment: perpendicular to chip edge (pad_width / 2 + extension, clears pad before bend)
   - Fan segment: angles outward; both its length AND its angle grow with
     position along the edge (0 at the edge midpoint, full length and a 45°
     angle at the outermost/corner pad)
5. **Collision Check** - Validate endpoint spacing meets minimum spacing requirements

A pad that already has its surface escape gets no second stub (#1834): same-net
track copper on the pad's layer that starts on the pad's copper and, through
tracks that touch, reaches a same-net via or ENDS outside the hull of the
part's pads (a track end no other track continues: a stub's tip, a route's far
end) -- or a via escape (`pad_has_via_escape`, #1804) -- serves it
(`pad_has_stub_escape`). Such a pad is left out before the fan is planned (the
run prints `N pad(s) already have their surface escape, skipped`) and counted
in `pads_already_served`; its stub stays on the board, where the new stubs of
other nets clear it. So the step re-run on its own output, with the same or
other parameters, adds nothing, and a pad whose copper stays inside the hull (a
stub cut back to the pad, a track inward, a tie to the next pin), or is a
closed outline of tracks drawn round the pad, is escaped again. The fan angles are ramped over every selected pad, served or not, so a
pad still to escape gets the stub a first run would have given it.

## Stub Pattern

```
Corner pad: long, full 45° fan-out
    ╲
     ╲
      ●  (pad)

Mid-edge pad: shorter, shallower angle
     ╲
      ●  (pad)

Edge-midpoint pad: straight only (~0° fan)
      |
      ●  (pad)
```

The fan **length** varies linearly with edge position (0 at the midpoint,
max_diagonal_length = chip_width / 3 at the corner). The fan **angle** ramps
separately from ~0° at the midpoint to a full 45° at the *outermost pad* on each
side (issue #200). A uniform 45° fan keeps every adjacent stub parallel, so
neighbouring tips stay only pitch/√2 apart — too tight on a fine-pitch edge to
fit an escape via between them. Ramping the angle makes adjacent stubs *diverge*
instead, spreading the tips apart while the corner pad keeps its full 45° reach.
Tips are then snapped to the routing grid in a direction-preserving way, so a
near-midpoint stub's bend never reverses toward its neighbour.

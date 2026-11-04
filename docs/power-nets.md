# Power Net Analysis

Identify power nets and determine appropriate track widths for routing.

## Overview

Power nets (GND, VCC, etc.) typically require wider tracks than signal nets to handle higher currents. This document covers two approaches:

1. **Manual patterns** - Specify net patterns and widths via CLI
2. **AI-powered analysis** - Use the `/analyze-power-nets` skill for datasheet lookup and component analysis

A board whose net classes already size its power nets needs neither: when `--track-width` is omitted, every net routes at its own net class's track width (and any `.kicad_dru` `track_width` rule that matches it), so a `Power` class drawn at 0.5 mm routes at 0.5 mm. `--power-nets` is for the nets the classes do not size, and outranks the class width for the nets it names.

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--power-nets` | - | Glob patterns for power nets (e.g., `"*GND*" "*VCC*"`) |
| `--power-nets-widths` | - | Track widths in mm for each pattern (must match length) |
| `--neckdown-length` | 2.5 | Narrow track length from the pad on a necked-down power route (mm) |
| `--neckdown-taper-length` | 0.5 | Narrow-to-wide width taper (mm, 0 = abrupt) |
| `--no-power-tap-neckdown` | off | Disable the neck-down retry of failed wide power routes |

The plugin's Route tab has the same fields (Power Nets, Power Widths, and the "Power route neck-down" checkbox).

```bash
# Route with wider tracks for power nets
python3 py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net*" \
  --power-nets "*GND*" "*VCC*" "+3.3V" \
  --power-nets-widths 0.4 0.5 0.35
```

This routes:
- Nets matching `*GND*` with 0.4mm tracks
- Nets matching `*VCC*` with 0.5mm tracks
- Nets matching `+3.3V` with 0.35mm tracks
- All other nets at the run's track width (`--track-width`; omitted, each net's own net-class width, where an unclassed net takes the Default class's, else 0.3mm)

Patterns are matched in order - first matching pattern determines the width. Obstacle clearances automatically adjust for wider power traces.

**Note:** Power net widths are automatically enforced to be at least the run's base track width (`--track-width`, else the Default net class's). This prevents accidentally specifying power widths smaller than signal widths.

**The widths asked for are measured, not graded.** Each `route.py` step (and the GUI's routing tab) records the widths it was given in the output's `kicad_routing_tools` records (`power_widths`, net name -> mm), and `board_score.py` measures every recorded net against them with no flag: copper narrower than the request is reported in `net_widths.requested_under` (and on a `REQUESTED POWER WIDTHS` line), except a pad neck -- copper within the router's pad-entry zone of a narrower pad of its own net, at no less than that pad's short side -- which is excused and listed per net (`pad_neck_allowed_mm`). It is not counted in `blocking`: the router narrows a power route where its width does not fit and discloses it, so a request is what it aimed for. A width a net must have goes in `--net-min-widths`, which is graded (with the same pad-neck allowance) and outranks the record. A net class's own `track_width` is a drawing default, not a spec: the router honours it and discloses copper under it, and nothing grades it.

## Neck-Down for Failed Power Routes

A wide power track sometimes cannot fit where it matters most - escaping a fine-pitch pad,
or threading a congested region. When a power route (a 2-pad route, a multipoint main
connection, or a tap edge) fails at its power width, the router automatically retries it at
the layer's default track width and applies neck-down widths to the result:

- Segments within `--neckdown-length` (default 2.5mm) of a pad endpoint are laid at the
  pad-entry width -- the power width capped at that pad's short side -- wherever it fits, and
  at the default width only where it does not (#1318). Whether it fits is measured exactly
  against the neighbouring pads, tracks and vias where the routing grid would refuse it (the
  grid rounds, so it refuses some pin entries that clear); near the board edge, a bare drill
  hole, a keep-out or a filled copper graphic the grid's answer stands. An end on a via or a
  track takes the power width there. They used to stay at the default width whole, fit or not, so a short edge
  (a 1 mm pin-to-pin run, a 1.2 mm drop to a via at a 0.5 mm-pitch LQFP) shipped narrow end to
  end.
- Farther segments return to the power width wherever the wide clearance actually fits;
  a mid-route pinch stays narrow while open stretches go wide.
- Each narrow/wide transition gets a stepped width taper (`--neckdown-taper-length`,
  default 0.5mm, 4 steps; set 0 for an abrupt width change).

Disable the feature with `--no-power-tap-neckdown` (CLI) or the "Power route neck-down"
checkbox under the Power Widths field in the plugin. The narrowed sections are short pad
escapes, so the current-carrying impact matches a hand-routed neck-down.

Under `--escalation off` (the GUI's Escalation `off`) the neck-down is bounded to the pad
entry (`routing_defaults.PAD_ENTRY_NECK_MM`, 2 mm): a failed wide edge whose pads are at
most 2 mm apart retries narrower when the narrow route is at most 2 mm long, and a
via-to-pad tap drawn narrower than the net's width is refused past the same length. A
longer edge (or a short one whose narrow route is longer) is routed in two legs (#1452):
at the power width to a landing near the pin, then at the default width into it, kept only
when that narrow leg fits the pad-entry zone -- which under `off` is the allowance less the
taper (`--neckdown-length` capped at 2 mm minus `--neckdown-taper-length`), so the zone and
its taper together stay within 2 mm at each pad. The trunk keeps the power width and only
the pin entry narrows. When no such route exists the edge fails at the power width rather
than shipping a thin trunk, and the log says so: `Neck-down refused (--escalation off)`
with the edge's span, and a hint naming the allowance instead of advising a smaller track.

## AI-Powered Power Net Analysis

Use the `/analyze-power-nets` Claude Code skill for AI-powered datasheet lookup and component analysis. This is the recommended approach as it catches mislabeled power pins and provides current-based track width recommendations.

### When to Use AI Analysis

- You need to identify all power nets in a board
- You need current-based track width recommendations
- KiCad symbols have missing or incorrect pintype annotations
- You're working with unfamiliar components

### How to Invoke

```
# Ask the AI to analyze your board
Analyze the power nets in kicad_files/my_board.kicad_pcb and recommend track widths

# Or invoke the skill directly
/analyze-power-nets kicad_files/my_board.kicad_pcb
```

In the plugin, the **Ask AI** button beside the Power Nets field runs the same skill headless and fills the Power Nets and Power Widths fields from its recommendation.

### What the Skill Does

1. **Auto-classify obvious components** - Resistors, capacitors, inductors, ferrite beads, fuses, LEDs are classified automatically
2. **Identify unknowns** - Components that need datasheet lookup (ICs, connectors, transistors, diodes)
3. **Datasheet lookup** - Uses WebSearch to look up datasheets for unknown components
4. **Component classification** - Determines each component's role:
   - POWER_SOURCE: Regulators, power input connectors
   - CURRENT_SINK: ICs that consume power (MCUs, CPLDs, FPGAs)
   - PASS_THROUGH: Series elements (inductors, ferrite beads, fuses, power switches)
   - SHUNT: Decoupling capacitors, pull-up resistors
5. **Trace power paths** - Traces current flow from sinks back to sources
6. **Width recommendations** - Calculates track widths from the IPC-2221 chart fit and estimated current (see the note under Track Width Guidelines: this is 2221, not 2152)

### Why AI Analysis Helps

Many KiCad symbols have incorrect pintype annotations. Common mislabeling patterns:

| Actual Function | Often Mislabeled As | Examples |
|-----------------|---------------------|----------|
| PLL power supply | `passive` | VDDPLL, PLLVDD |
| Analog power | `input` | VCCA, AVDD, VDDA |
| Analog ground | `input` | VSSA, AGND, GNDA |
| Reference voltage | `input` | VREF, VRH, VRL |
| Core power | `passive` | VDDCORE, VCORE |
| Regulator input | `input` | VIN, IN |

**Example:** The MCF5213's VDDPLL pin is often marked as "passive" in KiCad symbols, but the [NXP datasheet](https://www.nxp.com/docs/en/data-sheet/MCF5213EC.pdf) shows it's the PLL power supply requiring filtered 3.3V.

### Example Output

```
## Power Net Analysis for my_board.kicad_pcb

### AI-Classified Components
| Ref  | Value   | Role          | Current | Notes |
|------|---------|---------------|---------|-------|
| J201 | JACK_2P | POWER_SOURCE  | 1000mA  | DC power input |
| U102 | MCF5213 | CURRENT_SINK  | 200mA   | ColdFire MCU |
| VR201| LT1129  | POWER_SOURCE  | 700mA   | 3.3V LDO output |

### Power Paths Traced
  TB201 (power input) -> SW_ONOFF201 -> F201 -> VR201 -> +3.3V -> U102 (MCU)

### Recommended Power Nets
| Net Name | Width | Reason |
|----------|-------|--------|
| Net-(TB201-P1) | 0.5mm | Power input path |
| Net-(F201-Pad1) | 0.5mm | After fuse |
| +3.3V | 0.5mm | Main rail, 500mA total |
| /VDDPLL | 0.3mm | PLL supply, 10mA |

### Routing Command
--power-nets "GND" "+3.3V" "/VDDPLL" "/VCCA" "Net-(TB201-P1)" --power-nets-widths 0.5 0.5 0.3 0.3 0.5
```

## Track Width Guidelines (IPC-2221)

Use these guidelines to select appropriate track widths based on expected current.

These come from the **IPC-2221** current-carrying charts (`I = k × ΔT^0.44 ×
A^0.725`), which this project labeled IPC-2152 in several places until #489 §6.
IPC-2152 is the later standard and reverses 2221's 2× derating of *internal*
layers, so the widths below are conservative for inner-layer routing. The
recommended column already carries safety margin over the bare formula.

| Current | 1oz Cu, 10C rise | 1oz Cu, 20C rise | Recommended |
|---------|-------------------|-------------------|-------------|
| <100mA  | 0.15mm (6 mil)    | 0.10mm (4 mil)    | 0.25mm      |
| 100-500mA | 0.25mm (10 mil) | 0.20mm (8 mil)    | 0.35mm      |
| 500mA-1A | 0.40mm (16 mil)  | 0.30mm (12 mil)   | 0.50mm      |
| 1-2A    | 0.60mm (24 mil)   | 0.45mm (18 mil)   | 0.60mm      |
| 2-5A    | 1.00mm (40 mil)   | 0.75mm (30 mil)   | 1.00mm      |
| >5A     | Use planes        | Use planes        | Plane       |

### General Recommendations

- **Ground nets**: Use widest practical width (0.4-0.5mm) or copper planes
- **Main power** (3.3V, 5V): 0.35-0.5mm depending on total load
- **Low-current supplies** (VREF, PLL): 0.25-0.3mm
- **Regulator outputs**: Size for max output current + margin
- **Regulator inputs**: Same width as output (input current = output current + quiescent current)

### Special Considerations

- **Analog supplies** (VCCA, VDDA): Route with star topology from main supply, use ferrite bead isolation
- **PLL supplies** (VDDPLL): Keep traces short to filter components, use LC filtering
- **Analog ground** (GNDA, VSSA): Separate star-ground connection, may need isolation from digital ground

### Related: GND Return Vias for Signal Integrity

When using GND planes, signal vias that transition between layers need nearby GND return vias
for proper return current paths. Use the `/find-high-speed-nets` skill to identify which signals
need this, and `route_planes.py --add-gnd-vias` to place them. See
[GND Return Via Placement](route-plane.md#gnd-return-via-placement) for details.

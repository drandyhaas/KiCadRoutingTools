# Power Net Analysis

Identify power nets and determine appropriate track widths for routing.

## Overview

Power nets (GND, VCC, etc.) typically require wider tracks than signal nets to handle higher currents. This document covers two approaches:

1. **Manual patterns** - Specify net patterns and widths via CLI
2. **AI-powered analysis** - Use the `/analyze-power-nets` skill for datasheet lookup and component analysis

## Command-Line Options

| Option | Default | Description |
|--------|---------|-------------|
| `--power-nets` | - | Glob patterns for power nets (e.g., `"*GND*" "*VCC*"`) |
| `--power-nets-widths` | - | Track widths in mm for each pattern (must match length) |

```bash
# Route with wider tracks for power nets
python route.py input.kicad_pcb output.kicad_pcb --nets "Net*" \
  --power-nets "*GND*" "*VCC*" "+3.3V" \
  --power-nets-widths 0.4 0.5 0.3 \
  --track-width 0.2  # default for non-power nets
```

This routes:
- Nets matching `*GND*` with 0.4mm tracks
- Nets matching `*VCC*` with 0.5mm tracks
- Nets matching `+3.3V` with 0.3mm tracks
- All other nets with 0.2mm tracks (the default `--track-width`)

Patterns are matched in order - first matching pattern determines the width. Obstacle clearances automatically adjust for wider power traces.

**Note:** Power net widths are automatically enforced to be at least `--track-width`. This prevents accidentally specifying power widths smaller than signal widths.

## AI-Powered Power Net Analysis

Use the `/analyze-power-nets` Claude Code skill for AI-powered datasheet lookup and component analysis. This is the recommended approach as it catches mislabeled power pins and provides current-based track width recommendations.

### When to Use AI Analysis

- You need to identify all power nets in a board
- You need current-based track width recommendations
- KiCad symbols have missing or incorrect pintype annotations
- You're working with unfamiliar components

### How to Invoke

```
# Ask Claude to analyze your board
Analyze the power nets in kicad_files/my_board.kicad_pcb and recommend track widths

# Or invoke the skill directly
/analyze-power-nets kicad_files/my_board.kicad_pcb
```

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
6. **Width recommendations** - Calculates track widths based on IPC-2152 and estimated current

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

## Track Width Guidelines (IPC-2152)

Use these guidelines to select appropriate track widths based on expected current:

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

# Power Net Analysis

Identify power nets and determine appropriate track widths for routing.

## Overview

Power nets (GND, VCC, etc.) typically require wider tracks than signal nets to handle higher currents. This document covers three approaches:

1. **Manual patterns** - Specify net patterns and widths via CLI
2. **Automatic detection** - Use Python helpers to detect from KiCad pintype annotations
3. **AI-powered analysis** - Use datasheet lookup when annotations are missing or incorrect

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

## Automatic Power Net Detection

Instead of manually specifying patterns, use Python helpers to automatically detect power nets from KiCad's pintype annotations:

```python
from kicad_parser import parse_kicad_pcb
from net_queries import identify_power_nets_by_pintype, auto_assign_power_widths

pcb = parse_kicad_pcb("kicad_files/my_board.kicad_pcb")

# Detect power nets from KiCad pintype annotations
power_nets = identify_power_nets_by_pintype(pcb)
for net_id, info in power_nets.items():
    print(f"{info['name']}: {info['type']} ({info['pin_count']} pins)")

# Auto-assign track widths based on net type and fanout
widths = auto_assign_power_widths(pcb)
# Returns: {net_id: width_mm, ...}
```

### How Detection Works

The `identify_power_nets_by_pintype()` function detects power nets using KiCad's `pintype` annotations from schematic symbols:
- `pintype="power_in"` → supply or ground pins
- `pintype="power_out"` → regulator outputs

Returns a dict with:
- `name`: Net name
- `type`: `'ground'`, `'supply'`, or `'regulator_output'`
- `pintype`: Original KiCad pintype
- `components`: Set of component refs connected to this net
- `pin_count`: Number of power pins on this net

### Auto Width Assignment

The `auto_assign_power_widths()` function assigns track widths based on net type and fanout:

| Net Type | Condition | Default Width |
|----------|-----------|---------------|
| Ground | Any | 0.5mm |
| Regulator output | Any | 0.5mm |
| Main supply | ≥5 pins | 0.4mm |
| Low-fanout supply | <5 pins | 0.3mm |

All defaults are configurable via function parameters.

### Automatic Power Width Propagation

The router automatically propagates power net widths through series elements. This ensures that the entire power path uses appropriate track widths, not just the nets directly connected to power pins.

```python
from net_queries import propagate_power_widths_through_passives

# After identifying power nets
power_net_widths = identify_power_nets(pcb_data, power_nets, power_nets_widths)

# Propagate through series elements
power_net_widths = propagate_power_widths_through_passives(pcb_data, power_net_widths)
```

**What gets propagated through:**

| Component Type | Prefix | Example | Propagates? |
|----------------|--------|---------|-------------|
| Inductors | L* | L101 | Yes |
| Ferrite beads | FB* | FB201 | Yes |
| Fuses | F* | F1 | Yes |
| Low-value resistors | R* | R100 (0R, 4R7) | Yes (if ≤ V_max/0.1A) |
| Voltage regulators | VR*, U* | VR201 | Yes (output → input) |
| Capacitors | C* | C101 | No (not series elements) |
| High-value resistors | R* | R100 (10K) | No |

**Voltage regulator detection:**

The propagation automatically detects voltage regulators and propagates widths from output to input. Regulator inputs carry the same current as outputs (plus quiescent current), so they need the same track width capacity.

```
Example power path:
DC Jack → F1 (fuse) → D1 (diode) → VR1 (regulator) → +3.3V
                                        ↑
                       Input net gets same width as +3.3V
```

**Voltage-based resistor inclusion:**

Low-value resistors (current sense, etc.) are included based on the formula: R ≤ V_max / 0.1A

The maximum voltage is auto-detected from power net names (e.g., "+5V", "VCC_3V3") with a 5V minimum for safety. At 5V max, resistors ≤50Ω are included in propagation.

## AI-Powered Power Net Analysis

When KiCad pintype annotations are missing or incorrect, use the `/analyze-power-nets` Claude Code skill for AI-powered datasheet lookup.

### When to Use AI Analysis

- KiCad symbols have missing or incorrect pintype annotations
- You need current-based track width recommendations
- You want to verify power net identification before routing
- You're working with unfamiliar components

### How to Invoke

```
# Ask Claude to analyze your board
Analyze the power nets in kicad_files/my_board.kicad_pcb and recommend track widths

# Or invoke the skill directly
/analyze-power-nets kicad_files/my_board.kicad_pcb
```

### What the Skill Does

1. **Automated detection** - Finds power nets using KiCad's `pintype` annotations
2. **Mislabel detection** - Identifies power-named nets that weren't detected (VDDPLL, VCCA, etc.)
3. **Datasheet lookup** - Searches for component datasheets to verify power pins and current requirements
4. **Regulator input analysis** - Identifies voltage regulator inputs that need same current capacity as outputs
5. **Width recommendations** - Calculates track widths based on IPC-2152 and estimated current

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

### Components Analyzed
| Ref  | Part Number | Function        | Datasheet |
|------|-------------|-----------------|-----------|
| U102 | MCF5213     | ColdFire MCU    | [NXP](https://nxp.com/...) |
| U301 | XCR3256     | CoolRunner CPLD | [Xilinx](https://...) |
| VR201| LT1129      | 3.3V LDO        | [ADI](https://...) |

### Identified Power Nets
| Net Name | Type          | Est. Current | Recommended Width |
|----------|---------------|--------------|-------------------|
| GND      | Ground        | 500mA+       | 0.5mm or plane    |
| +3.3V    | Main supply   | 400mA        | 0.5mm             |
| /VDDPLL  | PLL supply    | 5mA          | 0.3mm             |
| /VCCA    | Analog supply | 10mA         | 0.3mm             |

### Regulator Power Paths
| Regulator | Input Net | Output Net | Max Current | Input Width |
|-----------|-----------|------------|-------------|-------------|
| VR201 (LT1129) | Net-(D201-K) | +3.3V | 700mA | 0.5mm |

### Ready-to-Use Configuration
```
--power-nets "GND" "+3.3V" "/VDDPLL" "/VCCA" "GNDA" --power-nets-widths 0.5 0.5 0.3 0.3 0.3
```

**Note:** The router automatically propagates power widths through series elements (fuses, inductors, ferrite beads) and voltage regulator inputs. You typically only need to specify the main power nets - downstream nets are handled automatically.

## Track Width Guidelines (IPC-2152)

Use these guidelines to select appropriate track widths based on expected current:

| Current | 1oz Cu, 10°C rise | 1oz Cu, 20°C rise | Recommended |
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
- **Regulator inputs**: Same width as output (input current ≈ output current + quiescent current)

### Special Considerations

- **Analog supplies** (VCCA, VDDA): Route with star topology from main supply, use ferrite bead isolation
- **PLL supplies** (VDDPLL): Keep traces short to filter components, use LC filtering
- **Analog ground** (GNDA, VSSA): Separate star-ground connection, may need isolation from digital ground

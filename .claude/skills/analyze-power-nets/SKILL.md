---
name: analyze-power-nets
description: Analyzes KiCad PCB files to identify power nets by looking up component datasheets via AI. Use when you need to determine which nets are power/ground nets and what track widths to use, especially when KiCad pintype annotations are missing or unreliable.
---

# Analyze Power Nets

When this skill is invoked, perform a full AI-powered analysis of the PCB to identify power nets and recommend track widths.

## Step 1: Load and Extract Components

```python
from analyze_power_paths import (
    analyze_pcb, get_components_needing_analysis, classify_component,
    trace_power_paths, get_power_net_recommendations, format_analysis_report,
    ComponentRole
)

components, pcb_data = analyze_pcb("path/to/file.kicad_pcb")
unknown = get_components_needing_analysis(components)
```

## Step 2: Auto-Classify Obvious Components

These are already handled automatically - no AI needed:

| Prefix | Role | Reason |
|--------|------|--------|
| R (resistor) | SHUNT | Pull-ups, voltage dividers |
| C (capacitor) | SHUNT | Decoupling caps |
| L (inductor) | PASS_THROUGH | Series filter element |
| FB (ferrite bead) | PASS_THROUGH | Series filter element |
| F (fuse) | PASS_THROUGH | Series protection |
| LED | CURRENT_SINK | Consumes current |
| SW (switch) | PASS_THROUGH | Power switching |

## Step 3: AI Analysis for Unknown Components

For each component in the `unknown` list, you MUST:

### 3a. Identify What It Is

Use WebSearch to look up the component:
```
WebSearch: "<part_value> <footprint_hint> datasheet"
```

Examples:
- `"DB9 connector pinout"` - Is it signal or power?
- `"1N4004 diode datasheet"` - Rectifier vs signal diode
- `"3906 transistor datasheet"` - Power switch or signal amp?
- `"LT1129 regulator datasheet"` - Voltage regulator specs
- `"MCF5213 power consumption"` - IC current requirements

### 3b. Determine Its Role

Based on the datasheet/search results, classify as:

| Role | Criteria |
|------|----------|
| **POWER_SOURCE** | Supplies current to the board: power jacks, battery connectors, regulator outputs, USB power inputs |
| **CURRENT_SINK** | Consumes significant current: ICs, MCUs, CPLDs, FPGAs, motor drivers, high-power LEDs |
| **PASS_THROUGH** | Current flows through it: power switches, transistors used as switches, protection diodes in series |
| **SHUNT** | Branches off the power path: ESD protection diodes, TVS diodes, signal connectors |

### 3c. Estimate Current Rating

From the datasheet, find:
- For ICs: Total supply current (ICC, IDD)
- For regulators: Maximum output current
- For connectors: Expected load current

### 3d. Apply Classification

```python
classify_component(components, 'J201', ComponentRole.POWER_SOURCE,
                   current_rating_ma=1000,
                   notes="DC barrel jack - main power input")
```

## Step 4: Key Components to Analyze

Focus AI analysis on these component types:

### Power Input Connectors (J*, P*, TB*, CN*)
- **Question**: Is this a power input or signal connector?
- **Look for**: Connection to power nets, barrel jack footprints, terminal blocks
- **Classify as**: POWER_SOURCE if it brings power onto the board

### Diodes (D* followed by digit)
- **Question**: Protection diode, rectifier, or LED?
- **Look for**: Part number (1N4004 = rectifier, BAT54 = Schottky, LED = LED)
- **Classify as**:
  - SHUNT if reverse protection (cathode to power, anode to GND)
  - PASS_THROUGH if series rectifier in power path
  - CURRENT_SINK if LED

### Transistors (Q*)
- **Question**: Power switch or signal amplifier?
- **Look for**: Connections to power nets, gate/base drive signals
- **Classify as**: PASS_THROUGH if switching power, SHUNT if signal

### ICs (U*)
- **Question**: How much current does it consume?
- **Look for**: ICC/IDD in datasheet, number of I/O pins
- **Classify as**: CURRENT_SINK with appropriate current rating
- **Note**: Regulators are auto-detected by power_out pins

### Voltage Regulators (VR*, or U* with regulator part number)
- **Question**: What is the maximum output current?
- **Look for**: Output current spec in datasheet
- **Classify as**: POWER_SOURCE with max output current

## Step 5: Trace Power Paths

After classifying all components:

```python
paths = trace_power_paths(pcb_data, components)
recommendations = get_power_net_recommendations(pcb_data, components, paths)
```

## Step 6: Generate Report

```python
print(format_analysis_report(pcb_data, components, paths, recommendations))
```

Provide the user with:
1. Summary of component classifications
2. Traced power paths showing current flow
3. Recommended power nets and track widths
4. Ready-to-use `--power-nets` command line arguments

## Example Analysis Session

When analyzing a board like kit-dev-coldfire-xilinx_5213:

1. **J201 (JACK_2P)**: WebSearch "DC barrel jack 2.1mm" → POWER_SOURCE, 1000mA
2. **TB201 (CONN_2)**: Terminal block on same net as J201 → POWER_SOURCE, 1000mA
3. **D201 (1N4004)**: WebSearch "1N4004 datasheet" → Rectifier, but connected cathode-to-power → SHUNT (reverse protection)
4. **Q101 (3906)**: WebSearch "2N3906 PNP transistor" → Emitter on +3.3V → PASS_THROUGH (power switch)
5. **U102 (MCF5213)**: WebSearch "MCF5213 power consumption" → CURRENT_SINK, 200mA
6. **U301 (XCR3256)**: WebSearch "XCR3256 CPLD power" → CURRENT_SINK, 150mA

## Output Format

```
## Power Net Analysis for board.kicad_pcb

### AI-Classified Components
| Ref | Value | Role | Current | Notes |
|-----|-------|------|---------|-------|
| J201 | JACK_2P | POWER_SOURCE | 1000mA | DC power input |
| U102 | MCF5213 | CURRENT_SINK | 200mA | ColdFire MCU |

### Power Paths Traced
  TB201 (power input) → SW_ONOFF201 → F201 → VR201 → +3.3V → U102 (MCU)

### Recommended Power Nets
| Net | Width | Reason |
|-----|-------|--------|
| Net-(TB201-P1) | 0.5mm | Power input path |
| Net-(F201-Pad1) | 0.5mm | After fuse |
| +3.3V | 0.5mm | Main rail, 500mA total |
| /VDDPLL | 0.3mm | PLL supply, 10mA |

### Routing Command
--power-nets "GND" "+3.3V" "/VDDPLL" "/VCCA" "Net-(TB201-P1)" --power-nets-widths 0.5 0.5 0.3 0.3 0.5
```

## Important Notes

1. **Do not rely on component reference prefixes** for ICs, connectors, or transistors - always look up the actual part
2. **Check net connections** - a connector on a power net is likely a power connector
3. **Trace upstream** - if a component feeds into a known power net, its input is also a power net
4. **Ground nets** are always power nets - include GND, GNDA, AGND, VSS, etc.

---
name: analyze-power-nets
description: Analyzes KiCad PCB files to identify power nets by looking up component datasheets via AI. Use when you need to determine which nets are power/ground nets and what track widths to use, especially when KiCad pintype annotations are missing or unreliable.
---

# Analyze Power Nets

When this skill is invoked, perform a full AI-powered analysis of the PCB to identify power nets and recommend track widths.

## Step 1: Load and Extract Components

```python
import sys
sys.path.insert(0, "py_tools")
from analyze_power_paths import (
    analyze_pcb, get_components_needing_analysis, classify_component,
    trace_power_paths, get_power_net_recommendations, format_analysis_report,
    ComponentRole
)

components, pcb_data = analyze_pcb("path/to/file.kicad_pcb")
unknown = get_components_needing_analysis(components)
```

## Step 2: Auto-Classify Obvious Components

These are already handled automatically - no AI needed (`_auto_classify_component`;
a prefix counts only when a digit follows it, e.g. `R12`, `FB3`, except `SW` and
`LED`):

| Prefix | Role | Reason |
|--------|------|--------|
| any part with a `power_out` pin | POWER_SOURCE | Regulator output (checked first) |
| R (resistor) | SHUNT | Pull-ups, voltage dividers (default; reclassify a series power resistor) |
| C (capacitor) | SHUNT | Decoupling cap: only a two-terminal cap with one side on a GND net. Any other cap stays UNKNOWN (could be a series cap) |
| L (inductor) | PASS_THROUGH | Series filter element |
| FB (ferrite bead) | PASS_THROUGH | Series filter element |
| F (fuse) | PASS_THROUGH | Series protection |
| LED | CURRENT_SINK | Consumes current |
| SW, S (switch) | PASS_THROUGH | Power switching |
| U (IC) | CURRENT_SINK | Current unknown: counted as 100 mA until you classify it |
| VR (regulator) | POWER_SOURCE | Current unknown: counted as 100 mA until you classify it |

## Step 3: AI Analysis for Unknown Components

`unknown` holds only the UNKNOWN parts (diodes, transistors, J/P/TB/CN
connectors, caps that are not plain decoupling, anything unmatched). ICs and
VR regulators are already auto-classified, but with no current rating, so every
one counts as 100 mA in the recommendations: look them up too (Step 4) and
re-classify them with `current_rating_ma`.

For each component in the `unknown` list, and each IC/regulator, you MUST:

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

`recommendations` is `{net_name: width_mm}`. Each net's current is the sum of
the traced paths through it plus the power pins of the sinks and sources on it;
every GND-named net is sized for the total sink current; nets under 50 mA
(`min_current_ma`) are left out. Current maps to width by the IPC-2221 table
with a 2x margin (`docs/power-nets.md`): under 50 mA 0.3 mm, under 100 mA
0.5, under 500 mA 0.7, under 1 A 1.0, under 2 A 1.2, under 5 A 2.0, else 3.0.

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
| Net-(TB201-P1) | 0.7mm | Power input path, 350mA |
| Net-(F201-Pad1) | 0.7mm | After fuse, 350mA |
| +3.3V | 0.7mm | Main rail: U102 200mA + U301 150mA |
| GND | 0.7mm | Return for all sinks, 350mA |

### Routing Command
--power-nets "GND" "+3.3V" "Net-(TB201-P1)" "Net-(F201-Pad1)" --power-nets-widths 0.7 0.7 0.7 0.7
```

## Important Notes

0. **Widths are for TRACKS; most power nets should be POURS.** Human corpus
   boards deliver nearly every rail with more than a few pads as a copper
   pour, not a routed track (86–100% of 4/6/8-layer boards pour power) —
   `/recommend-plane-mappings` and `/plan-pcb-routing` decide which rails get
   poured on which layers. Still pass ALL power nets in `--power-nets` with
   these widths — and since #562 this matters MORE, not less: poured rails
   now ride the route step too (`--nets "*"`), so these widths size their
   pour-launch welds, the in-run finalize's taps, and any track segments the
   chain draws for small rails not worth a pour. Never turn this
   skill's width table into a reason to route a many-pad rail as tracks
   instead of pouring it.
1. **Do not rely on component reference prefixes** for ICs, connectors, or transistors - always look up the actual part
2. **Check net connections** - a connector on a power net is likely a power connector
3. **Trace upstream** - if a component feeds into a known power net, its input is also a power net
4. **Ground nets** are always power nets - include GND, GNDA, AGND, VSS, etc.

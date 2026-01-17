---
name: analyze-power-nets
description: Analyzes KiCad PCB files to identify power nets by looking up component datasheets via AI. Use when you need to determine which nets are power/ground nets and what track widths to use, especially when KiCad pintype annotations are missing or unreliable.
---

# Analyze Power Nets

AI-powered analysis of KiCad PCB files to identify power nets and recommend track widths based on component datasheets.

## When to Use This Skill

- User wants to know which nets are power nets
- User wants track width recommendations for power routing
- KiCad file has poor or missing pintype annotations
- User wants to verify power net identification before routing

## Quick Start: Use Built-in Helpers

The codebase has helper functions for automatic power net detection:

```python
from kicad_parser import parse_kicad_pcb
from net_queries import identify_power_nets_by_pintype, auto_assign_power_widths

pcb = parse_kicad_pcb("path/to/file.kicad_pcb")

# Detect power nets from pintype annotations
power_nets = identify_power_nets_by_pintype(pcb)
for net_id, info in power_nets.items():
    print(f"{info['name']}: {info['type']} ({info['pin_count']} pins)")

# Auto-assign track widths
widths = auto_assign_power_widths(pcb)
# Returns: {net_id: width_mm, ...}
```

**BUT** these only catch nets where pintype is correctly annotated. Use AI analysis (below) to find mislabeled power pins.

## Full Workflow (When Pintype is Unreliable)

### Step 1: Extract Components from PCB

Parse the KiCad PCB file to extract all components:

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb("path/to/file.kicad_pcb")
for ref, fp in pcb.footprints.items():
    print(f"{ref}: {fp.value}")  # fp.value has the part number
```

### Step 2: Identify Active Components

Filter to find ICs and active components that need datasheet lookup. Skip obvious passives:

**Skip these (passives):**
- R* (resistors) - values like "10K", "4.7K", "100"
- C* (capacitors) - values like "100nF", "10uF", "10pF"
- L* (inductors) - values like "10uH", "BEAD"
- D* (diodes) - values like "1N4004", "BAT54", "LED"
- F* (fuses)
- J*, P*, TB* (connectors)
- SW* (switches)
- TP* (test points)

**Analyze these (active components):**
- U* (ICs) - microcontrollers, voltage regulators, transceivers, etc.
- Q* (transistors used as switches/regulators)
- VR* (voltage regulators)
- Any component with a recognizable part number (letters + numbers like "MCF5213", "MAX202")

### Step 3: Look Up Datasheets

For each active component, search for its datasheet:

```
WebSearch: "<part_number> datasheet power pins VCC GND current"
```

**Key information to extract from datasheets:**

1. **Power pin names and numbers:**
   - VCC, VDD, VDDIO, VDDCORE, VDDA (positive supply)
   - VSS, GND, GNDA, DGND, AGND (ground)
   - VREF (reference voltage)

2. **Supply voltage ranges:**
   - Operating voltage (e.g., 3.0V - 3.6V)
   - Absolute maximum ratings

3. **Current requirements:**
   - Typical operating current
   - Maximum current per supply pin
   - Total device current

4. **Multiple power domains:**
   - Core vs I/O voltage
   - Analog vs digital supplies
   - PLL supplies

### Step 4: Map Power Pins to Nets

Cross-reference the datasheet power pins with the nets in the PCB:

1. Find pads for each component in pcb.footprints[ref].pads
2. Match pad.pinfunction to datasheet power pin names
3. Get the net_id and net_name for each power pad
4. Group nets by function (main power, ground, analog, PLL, etc.)

### Step 5: Recommend Track Widths

Based on current requirements, recommend track widths using IPC-2152:

| Current | 1oz Cu, 10°C rise | 1oz Cu, 20°C rise | Recommended |
|---------|-------------------|-------------------|-------------|
| <100mA  | 0.15mm (6 mil)    | 0.10mm (4 mil)    | 0.25mm      |
| 100-500mA | 0.25mm (10 mil) | 0.20mm (8 mil)    | 0.35mm      |
| 500mA-1A | 0.40mm (16 mil)  | 0.30mm (12 mil)   | 0.50mm      |
| 1-2A    | 0.60mm (24 mil)   | 0.45mm (18 mil)   | 0.60mm      |
| 2-5A    | 1.00mm (40 mil)   | 0.75mm (30 mil)   | 1.00mm      |
| >5A     | Use planes        | Use planes        | Plane       |

**General recommendations:**
- Ground nets: Use widest practical width (0.4-0.5mm) or planes
- Main power (3.3V, 5V): 0.35-0.5mm depending on total load
- Low-current supplies (VREF, PLL): 0.25-0.3mm
- Regulator outputs: Size for max output current + margin

## Output Format

Provide results in this format:

```
## Power Net Analysis for <filename>

### Components Analyzed
| Ref | Part Number | Function | Datasheet |
|-----|-------------|----------|-----------|
| U1  | MCF5213     | MCU      | [link]    |

### Identified Power Nets
| Net Name | Net ID | Type | Connected Components | Est. Current | Recommended Width |
|----------|--------|------|---------------------|--------------|-------------------|
| +3.3V    | 104    | Power | U1, U2, U3         | 500mA        | 0.5mm             |
| GND      | 91     | Ground | All                | 500mA        | 0.5mm (or plane)  |

### Routing Configuration
For use with route.py:
```
--power-nets "GND" "+3.3V" "VDDPLL"
--power-nets-widths 0.5 0.4 0.3
```

### Notes
- Any special considerations (separate analog ground, etc.)
```

## Example Analysis

For a board with MCF5213 + XCR3256 + MAX202:

1. **MCF5213 (ColdFire MCU):**
   - VDD pins (×8): 3.3V @ ~150mA total
   - VSS pins (×8): Ground
   - VDDA: Analog 3.3V @ 10mA
   - VDDPLL: PLL 3.3V @ 5mA

2. **XCR3256 (CPLD):**
   - VCCINT (×8): 3.3V core @ 50mA
   - VCCIO (×12): 3.3V I/O @ 30mA
   - GND (×20): Ground

3. **MAX202 (RS-232):**
   - VCC: 5V @ 10mA
   - GND: Ground

**Recommended configuration:**
```
--power-nets "GND" "+3.3V" "VDDPLL" "VCCA"
--power-nets-widths 0.5 0.5 0.3 0.3
```

## Finding Mislabeled Power Pins (AI Analysis)

When pintype annotations are wrong, AI datasheet lookup catches what automation misses.

### Common Mislabeling Patterns

| Actual Function | Often Mislabeled As | Examples |
|-----------------|---------------------|----------|
| PLL power supply | `passive` | VDDPLL, PLLVDD |
| Analog power | `input` | VCCA, AVDD, VDDA |
| Analog ground | `input` | VSSA, AGND, GNDA |
| Reference voltage | `input` | VREF, VRH, VRL |
| Core power | `passive` | VDDCORE, VCORE |

### Detection Strategy

1. **Run pintype detection first:**
   ```python
   power_nets = identify_power_nets_by_pintype(pcb)
   ```

2. **Find nets with power-related names NOT detected:**
   ```python
   # Check for nets that look like power but weren't detected
   power_keywords = ['VDD', 'VCC', 'VSS', 'GND', 'VCCA', 'VSSA', 'PLL', 'REF']
   for net_id, net in pcb.nets.items():
       if any(kw in net.name.upper() for kw in power_keywords):
           if net_id not in power_nets:
               print(f"MISSED: {net.name} - check datasheet!")
   ```

3. **Look up component datasheets for missed nets:**
   ```
   WebSearch: "<part_number> datasheet <pin_name> power supply"
   ```

4. **Cross-reference pin functions:**
   - Find which component the net connects to
   - Look up that pin in the datasheet
   - Verify if it's actually a power pin

### Example: MCF5213 VDDPLL

The MCF5213's VDDPLL pin is often marked as "passive" in KiCad symbols, but the [NXP datasheet](https://www.nxp.com/docs/en/data-sheet/MCF5213EC.pdf) shows:
- VDDPLL is the PLL power supply (3.3V)
- Requires filtering (LC network)
- Should be treated as a power net

This is exactly why AI datasheet analysis adds value beyond pintype detection.

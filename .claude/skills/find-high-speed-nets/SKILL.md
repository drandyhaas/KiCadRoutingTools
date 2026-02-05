---
name: find-high-speed-nets
description: Analyzes a KiCad PCB to identify high-speed nets by looking up component datasheets via AI. Classifies nets by speed tier (ultra-high/high/medium/low), estimates max frequencies and rise times per interface, and recommends GND return via distances for signal integrity.
---

# Find High-Speed Nets

When this skill is invoked with a KiCad PCB file, perform a comprehensive analysis to identify which nets carry high-speed signals and recommend appropriate signal integrity measures.

## Step 1: Load and Extract Components

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')

# Basic stats
print(f'Total nets: {len(pcb.nets)}')
print(f'Total footprints: {len(pcb.footprints)}')
```

Also run `list_nets.py` to get differential pairs and power nets (these inform the analysis):

```bash
python3 list_nets.py path/to/file.kicad_pcb --diff-pairs --power
```

Note: Differential pairs are handled separately by `route_diff.py`, which adds its own GND
return vias automatically. This analysis focuses on single-ended signal vias from `route.py`.

## Step 2: Pre-Classify Nets by Name Patterns

Scan all net names (case-insensitive) to get an initial speed estimate before datasheet lookup:

```python
speed_tiers = {
    'ultra_high': {  # >1 GHz
        'patterns': ['DDR3', 'DDR4', 'DDR5', 'LPDDR', 'PCIE', 'SATA',
                     'USB3', 'SGMII', 'XGMII', 'TMDS', '10G'],
        'typical_freq_mhz': 1600,
        'typical_rise_ns': 0.3,
    },
    'high': {  # 100 MHz - 1 GHz
        'patterns': ['DDR', 'DQ', 'DQS', 'DQM', 'RGMII', 'RMII',
                     'QSPI', 'QIO', 'SDIO', 'LVDS', 'HDMI',
                     'USB', 'ETH', 'ULPI', 'EMMC'],
        'typical_freq_mhz': 200,
        'typical_rise_ns': 1.0,
    },
    'medium': {  # 10 - 100 MHz
        'patterns': ['SPI', 'SCK', 'SCLK', 'MOSI', 'MISO',
                     'CLK', 'MCLK', 'BCLK', 'JTAG', 'TCK',
                     'TDI', 'TDO', 'TMS', 'SWDIO', 'SWCLK',
                     'CAN', 'SDMMC'],
        'typical_freq_mhz': 50,
        'typical_rise_ns': 3.0,
    },
    'low': {  # <10 MHz
        'patterns': ['I2C', 'SCL', 'SDA', 'UART', 'TX', 'RX',
                     'GPIO', 'LED', 'BTN', 'SW_', 'ADC', 'DAC',
                     'PWM', 'RST', 'RESET', 'EN', 'ENABLE',
                     'IRQ', 'INT'],
        'typical_freq_mhz': 1,
        'typical_rise_ns': 10.0,
    },
}

# Build initial classification
net_speed = {}  # {net_name: (tier, interface_guess, freq_mhz)}
for net in pcb.nets.values():
    if not net.name or net.name == '':
        continue
    name_upper = net.name.upper()
    for tier, info in speed_tiers.items():
        if any(pat in name_upper for pat in info['patterns']):
            net_speed[net.name] = (tier, 'name_match', info['typical_freq_mhz'])
            break
```

Report the initial classification to the user: how many nets in each tier, which patterns matched.

## Step 3: Pre-Classify Components by Footprint and Value

Check footprint names and component values for high-speed indicators:

```python
hs_component_keywords = {
    'FPGA':  ('ultra_high', 'Programmable logic - likely LVDS/DDR/SerDes'),
    'CPLD':  ('high',       'Programmable logic - check I/O speed'),
    'DDR':   ('ultra_high', 'DDR memory'),
    'SDRAM': ('high',       'SDRAM - check generation (DDR2/3/4)'),
    'LPDDR': ('ultra_high', 'Low-power DDR memory'),
    'USB':   ('high',       'USB interface - check version (1.1/2.0/3.x)'),
    'ETH':   ('high',       'Ethernet - check speed (10/100/1000)'),
    'PHY':   ('high',       'PHY transceiver - check interface type'),
    'SERDES':('ultra_high', 'Serializer/deserializer'),
    'XCVR':  ('ultra_high', 'Transceiver - multi-GHz serial'),
    'HDMI':  ('ultra_high', 'HDMI - TMDS lanes'),
}

high_speed_components = []  # [(ref, footprint, tier, notes)]
for ref, fp in pcb.footprints.items():
    name_upper = fp.footprint_name.upper()
    for kw, (tier, notes) in hs_component_keywords.items():
        if kw in name_upper:
            high_speed_components.append((ref, fp.footprint_name, tier, notes))
            break

# Also flag ICs with high pin count (>40 pins) as likely having fast interfaces
for ref, fp in pcb.footprints.items():
    if ref.upper().startswith('U') and len(fp.pads) > 40:
        if ref not in [c[0] for c in high_speed_components]:
            high_speed_components.append((ref, fp.footprint_name, 'unknown',
                                          f'{len(fp.pads)}-pin IC - needs datasheet lookup'))
```

Report components found and which need AI analysis.

## Step 4: AI Datasheet Lookup

For each IC (U*), non-trivial connector, and any component flagged in Step 3, use WebSearch
to find datasheet information about signal speeds.

### 4a. Search for Interface Speeds

```
WebSearch: "<part_value> <footprint_hint> datasheet maximum clock frequency"
```

Examples:
- `"MCF5213 ColdFire datasheet bus clock speed"` - Find MCU bus frequency
- `"XCR3256 CPLD datasheet I/O toggle rate"` - Find CPLD max speed
- `"KSZ9031 Ethernet PHY datasheet RGMII clock"` - Find PHY interface speed
- `"W25Q128 QSPI flash datasheet SPI clock frequency"` - Find flash max SPI clock
- `"FT2232H USB datasheet interface speed"` - Find USB version and speed
- `"IS42S16160 SDRAM datasheet CAS latency clock"` - Find SDRAM clock speed
- `"STM32F407 datasheet peripheral clock speeds"` - Find MCU interface speeds

### 4b. Extract Speed Information

From each datasheet result, extract:

| Field | What to Look For |
|-------|------------------|
| **Interface type** | SPI, I2C, UART, USB 2.0 HS, DDR3-1600, RGMII, etc. |
| **Max clock/data rate** | Bus clock in MHz/GHz, data rate in MT/s or Gbps |
| **Output rise time** | tr/tf in ns or ps (often in "Switching Characteristics" table) |
| **I/O standards** | LVCMOS, LVTTL, LVDS, HSTL, SSTL (for FPGAs) |

### 4c. Map to Specific Pins and Nets

For each interface found, identify which pins carry the fast signals:

```python
# Map IC interface pins to nets
for ref, fp in pcb.footprints.items():
    for pad in fp.pads:
        if pad.pinfunction:
            func_upper = pad.pinfunction.upper()
            # Check if this pin is part of a high-speed interface
            # e.g., pinfunction="SPI_CLK" on an MCU → that net is SPI speed
```

Record per-component findings:
- Component ref and value
- Each interface found: protocol, max frequency, rise time (if available)
- Which net names are associated with each interface

### 4d. Update Net Classifications

Upgrade net classifications based on datasheet findings. A datasheet-confirmed speed
always overrides the name-pattern estimate from Step 2.

## Step 5: Trace High-Speed Signals Through Series Passives

High-speed signals often pass through series components (termination resistors, AC coupling
caps, ferrite beads). The nets on both sides carry the same speed signal.

```python
# Build map of 2-pad series passives
series_passives = {}  # {ref: (net_a, net_b)}
for ref, fp in pcb.footprints.items():
    ref_upper = ref.upper()
    is_passive = any(ref_upper.startswith(p) and (len(ref_upper) <= 1 or
                     ref_upper[len(p):len(p)+1].isdigit())
                     for p in ['R', 'C', 'L', 'FB'])
    if is_passive and len(fp.pads) == 2:
        net_a = fp.pads[0].net_name
        net_b = fp.pads[1].net_name
        if net_a and net_b and net_a != net_b:
            series_passives[ref] = (net_a, net_b)

# BFS: propagate speed classification through series passives
# If net_a is classified high-speed, net_b inherits the same classification
from collections import deque

def propagate_speeds(net_speed, series_passives):
    # Build adjacency: net → [connected nets via passives]
    adjacency = {}
    for ref, (net_a, net_b) in series_passives.items():
        adjacency.setdefault(net_a, []).append(net_b)
        adjacency.setdefault(net_b, []).append(net_a)

    # BFS from each classified net
    propagated = {}
    for net_name, (tier, interface, freq) in list(net_speed.items()):
        queue = deque([net_name])
        visited = {net_name}
        while queue:
            current = queue.popleft()
            for neighbor in adjacency.get(current, []):
                if neighbor not in visited and neighbor not in net_speed:
                    propagated[neighbor] = (tier, f'{interface} via series passive', freq)
                    visited.add(neighbor)
                    queue.append(neighbor)

    net_speed.update(propagated)
    return propagated  # return newly classified nets for reporting
```

Report which nets were added by propagation (e.g., "Net-R1-Pad2 classified as high-speed
via series resistor R1 from SPI_CLK").

## Step 6: Generate Report

### Per-Interface Groups

Group nets by interface and source/destination components:

```
## High-Speed Net Analysis for board.kicad_pcb

### Interface Groups

**SPI bus: U1 (STM32F407) <-> U3 (W25Q128)**
- /SPI_CLK: 50 MHz (datasheet-confirmed)
- /SPI_MOSI: 50 MHz (same bus)
- /SPI_MISO: 50 MHz (same bus)
- /SPI_CS: 50 MHz (same bus)
- Speed class: Medium

**DDR3 memory: U1 <-> U5 (MT41K256)**
- /DDR_DQ0..DQ15: 800 MHz (DDR3-1600)
- /DDR_DQS0, DQS1: 800 MHz
- /DDR_CLK: 800 MHz
- /DDR_A0..A14: 800 MHz
- Speed class: Ultra-high
```

### Speed Summary Table

```
| Net Name | Interface | Component | Max Freq | Rise Time | Speed Class |
|----------|-----------|-----------|----------|-----------|-------------|
| /DDR_DQ0 | DDR3 | U1<->U5 | 800 MHz | ~0.3 ns | Ultra-high |
| /SPI_CLK | SPI | U1<->U3 | 50 MHz | ~3 ns | Medium |
| /I2C_SCL | I2C | U1<->U2 | 400 kHz | ~100 ns | Low |
```

### GND Return Via Recommendation

Based on the highest speed class found on the board:

| Speed Class | Frequency | Recommended `--gnd-via-distance` | Rationale |
|-------------|-----------|----------------------------------|-----------|
| Ultra-high | >1 GHz | 2.0 mm | Return path critical; lambda/20 ~ 7 mm at 1 GHz in FR4 |
| High | 100 MHz - 1 GHz | 3.0 mm | Good return path, moderate density |
| Medium | 10 - 100 MHz | 5.0 mm | Return current less localized |
| Low | <10 MHz | Skip | Plane provides adequate return path |
| **Minimum physical** | any | **3 x (via_size + clearance)** | Vias cannot physically fit closer |

For this board, the tightest interface is **[interface]** at **[freq]**, so use:

```
--add-gnd-vias --gnd-via-distance [recommended_value]
```

**Note:** Differential pairs (detected by `/plan-pcb-routing` or `list_nets.py --diff-pairs`)
are routed with `route_diff.py`, which adds its own GND return vias automatically.
The `--gnd-via-distance` recommendation here applies to single-ended signal vias only.

### Impedance Notes (if applicable)

If high-speed interfaces are detected, note relevant impedance targets:

| Interface | Typical Impedance | Notes |
|-----------|-------------------|-------|
| DDR3/4 | 40 ohm SE | SSTL, check memory controller spec |
| USB 2.0 HS | 90 ohm diff | Routed as diff pair |
| USB 3.x | 85 ohm diff | Routed as diff pair |
| Gigabit Ethernet | 100 ohm diff | Routed as diff pair |
| LVDS | 100 ohm diff | Routed as diff pair |
| PCIe | 85 ohm diff | Routed as diff pair |
| SPI/I2C/UART | 50 ohm SE (typical) | Usually not impedance-controlled |

## Important Notes

1. **Datasheet results override name-pattern guesses** - A net named "CLK" could be 1 MHz
   or 1 GHz; the datasheet determines the actual speed
2. **Check all ICs, not just the obvious ones** - A "simple" MCU may have USB HS, SDIO,
   or QSPI peripherals running at hundreds of MHz
3. **Series passives propagate speed** - The net on the other side of a termination resistor
   or AC coupling cap carries the same speed signal
4. **Use the tightest distance** - If the board has both DDR3 (ultra-high) and I2C (low),
   the GND return via distance should be set for the DDR3 signals (2.0 mm)
5. **Diff pairs are separate** - `route_diff.py` handles GND return vias for differential
   pairs independently; this analysis is for single-ended vias from `route.py`
6. **When in doubt, include GND return vias** - They cost only board space; omitting them
   on a board that needs them causes signal integrity and EMI problems

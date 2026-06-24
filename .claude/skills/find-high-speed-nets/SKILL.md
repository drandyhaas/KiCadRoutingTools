---
name: find-high-speed-nets
description: Analyzes a KiCad PCB to identify high-speed and impedance-controlled nets by looking up component datasheets via AI. Classifies nets by speed tier (ultra-high/high/medium/low), detects RF/antenna feeds and other controlled-impedance nets, estimates max frequencies and rise times per interface, and recommends GND return via distances AND impedance-controlled routing: target ohms, the route.py/route_diff.py --impedance commands, and near-fab-floor width/spacing (~0.1 mm gap & clearance, impedance-derived width clamped to the floor) that override and update the board's Default net class.
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

## Step 4.5: RF / Antenna Feeds and Controlled-Impedance Nets

Some nets must be routed to a **target impedance** regardless of their switching
"speed tier" — most importantly RF/antenna feeds, which are a blind spot of the
name-pattern and speed-tier logic above (a 915 MHz antenna trace may be named just
`RF` or `ANT`). Detect them explicitly and attach a target impedance + a routing
recommendation; these feed the plan's impedance-controlled routing step (see the
report below and `/plan-pcb-routing`).

### 4.5a. RF / antenna feeds (single-ended 50 ohm, or 100 ohm balanced)

The decisive signature is a net connecting an **RF source** to an **antenna
termination**:

This is a deliberately **broad candidate scan** — favour recall over precision.
The matching below will over-include (e.g. `SMA` hits "D_SMALL", `PA` hits "Pad",
GND touches the SMA shell). That is fine: **you then confirm each candidate with
judgment** — is it really a radio/transceiver/PA output going to an antenna
termination? — using the component knowledge and datasheet lookup from Step 4, and
discard the coincidental hits. The only hard pre-filter is power/ground and very
large nets, which are never a point-to-point 50 ohm feed.

```python
# Broad keyword lists (substring match). Radio/transceiver/PA/LNA parts:
RF_SOURCE_KEYWORDS = ['SX12', 'SX127', 'LORA', 'WIO', 'NRF', 'ESP32', 'CC1101',
                      'CC120', 'RFM9', 'SI446', 'RADIO', 'TRANSCEIVER', 'BALUN',
                      'PA', 'LNA', 'RF']
# Antenna terminations (RF connectors / chip antennas). 73391 = Molex SMA part no.
ANTENNA_KEYWORDS = ['SMA', 'U.FL', 'UFL', 'IPEX', 'IPX', 'MHF', 'MMCX', 'BNC',
                    'ANTENNA', 'ANT', 'CHIP_ANT', '73391']
RF_PIN_HINTS = ['RF', 'ANT']

def is_rf_source(fp):
    blob = (fp.footprint_name + ' ' + (fp.value or '')).upper()
    if any(k in blob for k in RF_SOURCE_KEYWORDS):
        return True
    return any(any(h in (p.pinfunction or '').upper() for h in RF_PIN_HINTS)
               for p in fp.pads)

def is_antenna_term(ref, fp):
    # include the reference designator so ANT1 / JANT1 / EXT_ANT1 match too
    blob = (ref + ' ' + fp.footprint_name + ' ' + (fp.value or '')).upper()
    return any(k in blob for k in ANTENNA_KEYWORDS)

# A net is an RF/antenna feed if it touches an antenna terminal AND (an RF source
# OR is RF-named) -- but NOT if it is a power/ground net or a large net. This guard
# matters: the SMA's shell/ground pads and the radio's ground pins both sit on GND,
# so GND would otherwise be falsely flagged. A real antenna feed is point-to-point
# (the signal pin plus at most a small matching network), never a 90-pad rail.
import re
POWER_GND_PREFIXES = ['GND', 'GROUND', 'VSS', 'VCC', 'VDD', 'VBAT', 'VBUS', 'VIN',
                      'VEE', 'VREF', 'PGND', 'AGND', 'DGND', 'PWR', 'POWER',
                      'VAA', 'VDDA', 'VSSA', '+', '-']
# `power_ground_nets` is the GND + power-net set from `list_nets.py --power` (Step 1);
# pass it in. The name checks are a fallback that also catches rails the pattern
# scan would miss (e.g. PWR2V5, +3V3, 1V8, 5V, V33).
def is_power_ground(name, power_ground_nets=()):
    nu = name.upper().lstrip('/')
    if name in power_ground_nets:
        return True
    if any(nu == p or nu.startswith(p) for p in POWER_GND_PREFIXES):
        return True
    return bool(re.search(r'\d+V\d*|V\d+', nu))  # voltage-style rail names

rf_sources = {ref for ref, fp in pcb.footprints.items() if is_rf_source(fp)}
antenna_terms = {ref for ref, fp in pcb.footprints.items() if is_antenna_term(ref, fp)}
rf_candidates = {}  # {net_name: [endpoints]}
for net in pcb.nets.values():
    if not net.name or is_power_ground(net.name, power_ground_nets) or len(net.pads) > 6:
        continue  # skip rails (GND touches both the SMA shell and the radio) and big nets
    refs = {pad.component_ref for pad in net.pads}
    nu = net.name.upper().lstrip('/')
    name_rf = any(nu == p or nu.startswith(p) for p in ['RF', 'ANT', 'RFIO', 'RFO', 'RFI'])
    if (refs & antenna_terms) and ((refs & rf_sources) or name_rf):
        rf_candidates[net.name] = [f'{p.component_ref}.{p.pad_number}' for p in net.pads]
```

### Confirm each candidate (read the datasheet)

`rf_candidates` is a broad first pass — **confirm each one before recommending
impedance routing**, exactly as Step 4 does for speed:

- **Read the part's datasheet** (WebSearch) for the component on the source side.
  Verify it is a radio/transceiver/PA/LNA and that the pad on this net is its **RF
  output/antenna pin** (e.g. `RFO`, `ANT`, `RFIO`, `RF_HF`). The datasheet/reference
  design also states the **target impedance** — almost always **50 ohm
  single-ended**, but confirm (a balun output is differential; some modules
  integrate matching and specify a particular port impedance).
- **Reject coincidental hits**: `SMA` inside "D_SMALL", `PA` inside "Pad", a logic
  net that merely touches a connector also tagged as an antenna, etc. A real feed
  is a short point-to-point net from the radio's RF pin to an antenna termination
  (often through a small matching network / DC-block — those series parts are fine).
- If a part is clearly an RF source but **no** antenna net was found (or vice-versa),
  say so and look closer — a missed antenna feed is worse than a candidate you reject.

Record confirmed feeds as `rf_nets = {net_name: 'single-ended 50'}` (or
`'differential 100'` for a balun/balanced port). Report each with its endpoints and
the datasheet basis, e.g.
`RF: IC1.15 (Wio-E5 / SX1262 radio, RFO_HF pin) -> CONN5.1 (Molex SMA) => 50 ohm SE (datasheet-confirmed)`.

### 4.5b. Other controlled-impedance interfaces

The high-speed interfaces classified above carry standard impedance targets. Record
a target for each so the plan can route them with `--impedance`:

| Interface (detected) | Impedance target | Routed by |
|----------------------|------------------|-----------|
| RF / antenna feed (single-ended) | **50 ohm SE** | `route.py --impedance 50` |
| RF / antenna feed (balanced pair) | **100 ohm diff** | `route_diff.py --impedance 100` |
| USB 2.0 HS | 90 ohm diff | `route_diff.py --impedance 90` |
| USB 3.x / PCIe / SATA | 85 ohm diff | `route_diff.py --impedance 85` |
| Gigabit Ethernet / LVDS | 100 ohm diff | `route_diff.py --impedance 100` |
| DDR3/4 data & strobe | 40 ohm SE (SSTL) | `route.py --impedance 40` |
| HDMI/DP (TMDS) | 100 ohm diff | `route_diff.py --impedance 100` |

Split the result into **single-ended impedance nets** (RF 50, DDR SSTL 40 — these
need the new single-ended impedance routing step) and **differential impedance
pairs** (the rest — handled by `route_diff.py --impedance`).

> **Stackup is required.** `--impedance` computes the trace width per layer from
> the board stackup. If the board has only KiCad's default stackup (see
> `/plan-pcb-routing` Step 2 / `/recommend-stackup`), the width will be wrong —
> flag this and recommend running `/recommend-stackup` first.

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

### Impedance-Controlled Routing Recommendation (actionable)

List every controlled-impedance net found in Steps 4.5/5 with its target and the
command that routes it. The router computes the per-layer width from the board
stackup, so **a real stackup must exist first** (`/recommend-stackup`).

**Single-ended impedance nets** (RF/antenna 50 ohm, DDR SSTL 40 ohm) — these need a
**dedicated `route.py --impedance` pass**, placed in the pipeline **after the
diff-pair step and before the general single-ended signal route** (a "Step 2b").
Like diff pairs, they are highly constrained (fixed width, want a short/direct path
over a clean ground reference) so they must claim their channel before the bulk
signals fill the area. They must then be **excluded from the general signal route**
(`"*" "!GND" "!VCC" "!RF"`) so a later rip-up cannot re-route them at the wrong
width, and counted as "claimed by the impedance step" in the coverage ledger.

```bash
# Step 2b: impedance-controlled single-ended nets (e.g. the antenna feed), on an
# outer layer over the GND plane created later; short/direct is the router default.
python3 -X utf8 route.py board_diff.kicad_pcb board_imp.kicad_pcb \
    --nets RF --impedance 50 --layers F.Cu --clearance <floor> ...
```

**Differential impedance pairs** keep being routed in the diff-pair step (Step 2),
just add `--impedance` — **and ride the fab floor for the gap and clearance:**

```bash
python3 -X utf8 route_diff.py board.kicad_pcb board_diff.kicad_pcb \
    --nets "*USB*" "*LVDS*" --impedance 90 \
    --diff-pair-gap 0.1 --clearance 0.1 \
    --layers F.Cu In1.Cu In2.Cu B.Cu
```

**Width and spacing: choose them near the fab floor (~0.1 mm), overriding the
net class.** The stock Default net class is usually wide (e.g. `diff_pair_gap`
0.25 mm, `diff_pair_width`/`track_width` 0.2 mm). A wide pair is a *fatter
bundle* that needs more lateral room, so on a congested board the router drops
pairs it would otherwise route — measured: on `glasgow_revC` all 13 FPGA pairs
couple at `--diff-pair-gap 0.1`, but 2 fail at `0.25`. So for every
impedance-controlled net:

- **Spacing (`--diff-pair-gap`) and `--clearance`: the fab floor (~0.1 mm).** Do
  NOT read the net-class `diff_pair_gap`; recommend the floor. Tighter coupling
  is also better signal integrity for the pair.
- **Width: keep `--impedance` (it computes the per-layer width from the stackup
  for the target ohms), but the result is clamped to the fab floor (~0.1 mm) — it
  will not go thinner than the board can make.** Width stays impedance-correct;
  only the gap/clearance are forced tight.

**These override the net class, and the routed board's net class is updated to
match.** `route_diff.py` auto-invokes `fix_kicad_drc_settings.py` after routing,
which lowers the Default net class `diff_pair_gap` / `diff_pair_width` /
`clearance` to the values just routed (only-loosen — never raised). So after the
diff-pair step the `.kicad_pro` net class reads ~0.1 mm, not the stock 0.25, and
a later planner re-reading it won't resurrect the wide value. (To write it
without routing — e.g. before the run — call it directly:
`python3 fix_kicad_drc_settings.py board.kicad_pcb --diff-pair-gap 0.1 --diff-pair-width 0.1 --clearance 0.1`.)

| Interface | Target | Routed by | Pipeline step |
|-----------|--------|-----------|---------------|
| **RF / antenna feed (SE)** | **50 ohm SE** | `route.py --impedance 50` | **2b (new)** |
| DDR3/4 data & strobe | 40 ohm SE (SSTL) | `route.py --impedance 40` | 2b (new) |
| RF balanced / Ethernet / LVDS / HDMI | 100 ohm diff | `route_diff.py --impedance 100` | 2 (diff pairs) |
| USB 2.0 HS | 90 ohm diff | `route_diff.py --impedance 90` | 2 (diff pairs) |
| USB 3.x / PCIe / SATA | 85 ohm diff | `route_diff.py --impedance 85` | 2 (diff pairs) |
| SPI / I2C / UART / GPIO | not impedance-controlled | general `route.py` | 3 (signal) |

**RF/antenna extras:** route the feed **short and direct** on an outer layer over a
continuous ground plane (the planes step provides the reference); if the board has
an antenna/RF region, recommend the user draw a `User.2` keepout around it and pass
`--keepout` so other tracks/vias stay clear (you describe where; the user draws it
— see `/plan-pcb-routing`).

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
7. **RF/antenna feeds are controlled-impedance, not "fast logic"** - A net from a radio/PA/LNA
   to an SMA/U.FL/chip-antenna is a 50 ohm single-ended line even if its name (`RF`, `ANT`) and
   low logic-speed make the tier logic miss it. Detect it by the source->antenna topology
   (Step 4.5), route it in the dedicated single-ended `--impedance` step (2b), and require a
   real stackup first. Balanced antenna ports are 100 ohm differential via `route_diff.py`.
8. **Impedance width needs the stackup** - `--impedance` is meaningless on KiCad's default
   stackup; run `/recommend-stackup` before any impedance-controlled routing.

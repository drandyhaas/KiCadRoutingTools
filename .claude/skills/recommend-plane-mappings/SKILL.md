---
name: recommend-plane-mappings
description: Recommends net -> plane-layer assignments for a KiCad PCB with signal-integrity rationale. Reads the stackup, identifies plane-worthy nets (GND plus power rails by pad count and current), and assigns copper layers (GND adjacency for return paths, GND/VCC pairing for interplane capacitance, split layers for multiple rails). Use before creating planes on the Planes tab or with route_planes.py.
---

# Recommend Plane Mappings

When this skill is invoked with a KiCad PCB file, recommend which nets should get
copper planes and on which layers, with the reasoning a layout engineer would apply.

## Step 1: Read the Board

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')

print(pcb.board_info.copper_layers)                # e.g. ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
for layer in pcb.board_info.stackup:               # List[StackupLayer], ordered top to bottom
    print(layer.name, layer.layer_type, layer.thickness, layer.epsilon_r)

# Existing zones - don't silently duplicate them
for zone in pcb.zones:
    print(zone.net_name if hasattr(zone, 'net_name') else zone, getattr(zone, 'layer', '?'))
```

Report the layer count up front. On a **2-layer board** there are no inner layers to
give away: recommend a pour on B.Cu (and/or F.Cu) for GND, note the compromise, and
suggest `/recommend-stackup` if the board's signal content justifies 4 layers.

## Step 2: Identify Plane-Worthy Nets

Use pad counts plus the `/analyze-power-nets` approach (datasheet lookup for supply
pins and current estimates) rather than name matching alone:

```bash
python3 -X utf8 list_nets.py path/to/file.kicad_pcb --power
```

- **GND** (or the dominant ground net): always plane-worthy.
- **Power rails**: plane-worthy when pad count is high or estimated current is large;
  a rail feeding a handful of pins is better served by wide traces
  (`--power-nets` / `--power-nets-widths`) - say so instead of forcing a plane.
- Multiple low-current rails can share one split layer when their regions don't
  interleave; flag the split seams as return-path hazards for any signals that
  cross them.

## Step 3: Assign Layers with SI Rationale

- Put **GND adjacent to the primary signal layers** (return paths for outer-layer
  routing; on 4-layer boards: signals on F.Cu -> GND on In1.Cu).
- Pair **a power plane against GND** where the stackup allows (interplane
  capacitance; e.g. VCC on In2.Cu against GND on In1.Cu).
- High-speed content raises the stakes: reference-plane adjacency for the layers
  carrying fast signals comes first, rail convenience second.
- Mention existing zones that already cover a recommendation instead of repeating them.

## Step 3b: Routability Budget (do this BEFORE finalizing layers)

Planes compete with signal routing for the same layers. The corpus triage of the
worst-connectivity boards (ulx3s, butterstick, orangecrab, zynq_ad9364,
ottercast) found the single most damaging planning error was **giving every
inner layer to solid planes** while the signal steps also priced inner layers
3x — the router is left a 2-layer board around dense BGAs and 20-50 nets ship
open. The human-routed originals of those same boards all keep inner layers
partially signal-routable. Rules:

- **On a 4-layer board with any BGA >= ~100 balls, never plane BOTH inner
  layers.** One inner layer = GND plane; the other stays a routing layer or
  becomes a SPLIT power plane (rails as region pours, signals may still cross
  in the gaps). The human ulx3s puts GND pours on F/B + In2 and a split power
  plane on In1, leaving In2 the long-haul signal highway; the human zynq routes
  a 20-net LVDS bundle *through* its In1 GND plane.
- **Check the fanout escape layers first.** If BGA fanout has already run
  (escape stubs + vias exist), list which layers the escape stubs land on —
  `python3 -c "...count segments per layer under the BGA courtyard..."` or eye
  the board. A plane assigned to a layer carrying escape stubs will rip or
  strand those escapes (route_planes rips blockers; every rip risks a
  casualty). Prefer plane layers the escapes do NOT use.
- **On 6+ layer boards, put split power on a middle layer (In3/In4-style) and
  keep the layers adjacent to the BGA escape depth signal-routable** — the
  human butterstick planes In3/In4 and routes DDR3 on In2/In5; our failed run
  planed In2 (the DDR layer) instead.
- **2-layer boards: pour AFTER routing, never plane-first.** A B.Cu plane +
  3x layer cost turns the board into single-layer routing (neo6502: humans put
  47% of routed length on B.Cu and pour GND around the routes afterward).
  Recommend: route signals on both layers, then B.Cu/F.Cu GND pours + stitching
  (`route_planes.py` after `route.py`, or zones poured around existing copper).
- **Say which layer costs the signal steps should use.** When one inner layer
  is planed and one is free, recommend `--layer-costs` ~1.0-1.5 for the free
  inner layer (3.0 starves it and pushes everything onto F/B).
- **Power rails at fine-pitch BGAs (<= 0.8 mm) cannot arrive as 0.3-0.5 mm
  trunks** — at 0.5 mm pitch only one ~0.09 mm track fits between balls. If a
  rail feeds interior balls, either give it a plane/region (vias reach it
  vertically) or note that the power step must neck the trunk down near the
  courtyard (`--power-nets-widths` sized for the LAST reachable segment, not
  the whole run).

## Step 4: Report and Machine-Readable Result

Present the recommended mappings as a table (net(s), layer, rationale), note any
nets deliberately left to wide traces, and flag stackup problems.

End the reply with exactly one line:

```
RESULT=GND:In1.Cu;VCC|+3V3:In2.Cu
```

- Assignment groups separated by `;`
- Nets within a group joined by `|` (they share the layer)
- One copper layer per group, after the final `:` - a net needing planes on two
  layers appears in two groups
- Use exact net names as they appear on the board (including any `/` prefix)

This line maps 1:1 onto the Planes tab's assignment list and `route_planes.py
--nets ... --plane-layers ...` arguments.

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

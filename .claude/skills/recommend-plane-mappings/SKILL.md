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
python3 -X utf8 py_router/list_nets.py path/to/file.kicad_pcb --power
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
- **Outer layers (F.Cu/B.Cu): pick the flood net by same-layer SMD pad count.**
  An outer pour reaches its own-layer SMD pads by fill contact — zero vias for
  every such pad (the fanout's pour-direct drop skips the barrel when fill
  reaches the ball). `list_nets.py --power` prints per-net `(F.Cu n SMD,
  B.Cu n SMD, TH n)` for exactly this decision: between candidate nets for an
  outer flood, the one with the most SMD pads on that layer saves the most
  vias and escape congestion. TH-heavy nets gain nothing from layer choice
  (barrels connect everywhere), so don't let TH counts sway it.
- Mention existing zones that already cover a recommendation instead of repeating them.

## Step 3b: Routability Budget (do this BEFORE finalizing layers)

Planes compete with signal routing for the same layers — but the answer is
never to give up pours. **Two symmetric planning errors, both measured on the
corpus, and both cost 20–50 open nets on dense boards:**

1. **Over-planing:** giving every inner layer to solid planes while the signal
   steps also price inner layers 3× leaves the router a 2-layer board around
   dense BGAs. The human-routed originals keep inner layers partially
   signal-routable and route long-haul nets *through* them (a human 4-layer
   FPGA board routes a 20-net LVDS bundle through its GND plane layer's gaps).
2. **Under-pouring:** pouring GND only and delivering every power rail as wide
   tracks. Human boards pour EVERY rail with more than a few pads (a rail pad
   connects by dropping a via into a pour, not by a routed track); a corpus
   6-layer BGA board planned GND-only spent ~20% of its track copper on rails
   the human never routes, congested exactly the escape channels its RAM bus
   needed, and shipped ~26% incomplete.

**Recommend pours for every GND and power net with more than a few pads**
(across ~400 human corpus boards the median poured net has ≈ 3 pads, and
power-rail pouring is near-universal on 4+ layers: 86% of 4-layer, 97% of
6-layer, 100% of 8-layer boards; on 2-layer boards GND-only flood on both
sides is the norm and rail pours are a ~38% minority), then budget the
LAYERS like this:

- **High-speed reference first:** each layer carrying high-speed routes
  (DDR/RAM, USB HS, SerDes, RF) needs a solid UNSPLIT GND plane on an adjacent
  layer. Reserve those planes before anything else; split planes and
  route+pour go anywhere that isn't a high-speed reference.
- **On a 4-layer board with any BGA >= ~100 balls, never make BOTH inner
  layers solid planes.** One inner = solid GND; the other stays routable —
  either a SPLIT power plane (rails as region pours, signals cross in the
  gaps) or a route+pour layer (signals first, rail pours flow around them
  late). Pours on routing layers are normal human practice, not a fallback:
  most human boards pour their OUTER routing layers too (GND flood after
  routing).
- **Check the fanout escape layers first.** If BGA fanout has already run
  (escape stubs + vias exist), list which layers the escape stubs land on —
  `python3 -c "...count segments per layer under the BGA courtyard..."` or eye
  the board. A solid plane assigned to a layer carrying escape stubs is
  carved up by them: the pour never rips anything (#562), it flows around
  foreign copper, so that plane arrives as islands the route step must then
  join. Prefer solid-plane layers the escapes do NOT use.
- **On 6+ layer boards**, solid GND planes go nearest the outer signal layers,
  split power on a middle layer, and the layers at the BGA escape depth stay
  signal-routable (a human 8-layer DDR3 corpus board planes In3/In4 and routes
  the DDR bus on In2/In5; a failed run planed the DDR layer instead).
- **2-layer boards: pour AFTER routing, never plane-first.** A B.Cu plane +
  3x layer cost turns the board into single-layer routing (a dense 2-layer
  corpus board's human original puts 47% of routed length on B.Cu and pours
  GND around the routes afterward). Recommend: route signals on both layers,
  then GND + many-pad rail pours on both sides (`route_planes.py` after
  `route.py`, or zones poured around existing copper), with stitching only
  when the speed tier calls for it (`/plan-pcb-routing`'s finalize-planes
  step). **Then close the chain with another `route.py` whose `--nets` covers
  the poured nets** (`"*"` does): a pour alone connects nothing, and only that
  step's in-run plane finalize welds and taps the poured pads at the chain's
  own sizes (#562, #1112). Never end on the pour, or on `repair_planes.py`.
- **Say which layer costs the signal steps should use.** When one inner layer
  is planed and one is free, recommend `--layer-costs` ~1.0-1.5 for the free
  inner layer (3.0 starves it and pushes everything onto F/B).
- **Weight the recommendation by board type** (human-corpus measurements):
  fine-pitch big BGAs (≥100 balls, ≤0.5 mm) are the most pour-heavy class
  (median 5/17/21 poured nets on 4/6/8 layers) yet keep a median of ONE solid
  plane — recommend many split/route+pour rails and few solid reservations,
  with every rail near the BGA delivered by pour+via. RF boards want ONE GND
  net poured on every layer around the RF path and few rail pours.
  Power/motor boards deliver every heavy-current rail (V+, GNDPWR, phases) as
  pours on every layer it visits, never as wide tracks. Light MCU 2-layer
  boards: GND flood both sides, a couple of rail pours at most.
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

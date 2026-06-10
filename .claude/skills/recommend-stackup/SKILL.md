---
name: recommend-stackup
description: Reviews a KiCad board's stackup and recommends a fab-realistic one. Flags untouched default stackups, validates that target impedances are achievable with manufacturable widths using the project's own IPC-2141 formulas, and outputs the routing arguments (--impedance, plane layers) that follow. Use before impedance-controlled routing or time matching.
---

# Recommend Stackup

When this skill is invoked with a KiCad PCB file, review the stackup and recommend one suited to the board's impedance and plane requirements. The router's `--impedance` widths and `--time-matching` delays are computed *from the stackup* — a default stackup silently skews both.

## Step 1: Read the Current Stackup

```python
from kicad_parser import parse_kicad_pcb
pcb = parse_kicad_pcb('path/to/file.kicad_pcb')
for layer in pcb.stackup:
    print(layer.name, layer.thickness, layer.epsilon_r)
```

Judge whether it looks deliberate or default:
- No stackup section at all, or all dielectric layers with identical thickness and ε_r ≈ 4.5, suggests KiCad's untouched default.
- Real fab stackups have distinct core/prepreg thicknesses (e.g. JLCPCB 4-layer: ~0.21mm prepreg, ~1.0mm core).

Report the verdict explicitly: "stackup appears to be KiCad's default — impedance calculations will not match your fab" vs. "stackup looks deliberate."

## Step 2: Gather Requirements

- **Layer count and roles**: copper layers from the board; which should be planes (GND/power) vs signal.
- **Impedance targets**: from `/identify-diff-pairs` and `/find-high-speed-nets` results if available; otherwise infer from net names (USB 90Ω, LVDS/Ethernet/HDMI 100Ω, PCIe 85Ω, 50Ω single-ended for RF/clocks).
- **The fab**: ask the user. If they name one (JLCPCB, PCBWay, OSH Park, Eurocircuits…), WebSearch its standard multilayer stackup and impedance-control offerings rather than inventing thicknesses.

## Step 3: Recommend and Validate

Propose: layer assignment (e.g. 4-layer: F.Cu signal / In1.Cu GND plane / In2.Cu power plane / B.Cu signal), dielectric thicknesses and ε_r per the fab's standard stackup, and copper weights.

Validate with the project's own formulas so the recommendation matches what the router will compute:

```python
from impedance import calculate_width_for_impedance, calculate_impedance_for_layer

# After (hypothetically) applying the proposed stackup values:
w = calculate_width_for_impedance(pcb, 'F.Cu', target_z0=50.0)
```

Warn when a target is unachievable with manufacturable geometry (e.g. 50Ω microstrip needing a wider trace than the design's clearances allow, or a width below the fab's minimum) and propose the nearest workable option (thinner prepreg, different layer, relaxed target).

## Step 4: Output

1. The recommended stackup as a table (layer, type, thickness, ε_r) with the resulting trace widths per impedance target per layer.
2. The routing arguments that follow:
   ```
   --layers F.Cu In1.Cu In2.Cu B.Cu
   --impedance 50                      # route.py, single-ended
   --impedance 100                     # route_diff.py, differential
   route_planes.py --nets GND --plane-layers In1.Cu
   ```
3. Optionally, the KiCad `(stackup ...)` s-expression block for the user to apply via Board Setup → Physical Stackup. **Do not modify the board file directly** — stackup is a fab-facing decision the user must own.
4. If the stackup was changed, remind the user to re-run any impedance-based routing, since previously computed widths are now stale.

# Impedance API (`impedance`)

Characteristic-impedance formulas (IPC-2141 style), inverse solving (width
for a target impedance), stackup integration, and propagation-delay
calculation. This module powers `--impedance` routing and `--time-matching`.

Units: dimensions in mm, impedance in Ω, time in ps.

## Contents

- [Closed-form formulas](#closed-form-formulas) (no board needed)
- [Width for a target impedance](#width-for-a-target-impedance)
- [Stackup-aware calculations](#stackup-aware-calculations) (from a parsed board)
- [Propagation delay](#propagation-delay)
- [Reports](#reports)

## Closed-form formulas

Pure functions of geometry and dielectric — usable standalone:

```python
microstrip_z0(w, h, t, er) -> float            # outer layer, IPC-2141
stripline_z0(w, h, t, er) -> float             # symmetric inner layer
stripline_z0_asymmetric(w, h1, h2, t, er) -> float
differential_microstrip_z0(w, s, h, t, er) -> (zdiff, zodd)
differential_stripline_z0(w, s, h, t, er) -> (zdiff, zodd)
```

- `w` — trace width
- `s` — edge-to-edge spacing (differential)
- `h` — dielectric height to the reference plane (`h1`/`h2`: to the upper
  and lower planes for asymmetric stripline)
- `t` — copper thickness
- `er` — relative permittivity (FR4 ≈ 4.3)

```python
from impedance import microstrip_z0, differential_microstrip_z0

# 0.3mm trace over 0.2mm FR4, 35um copper
print(f"Z0 = {microstrip_z0(0.3, 0.2, 0.035, 4.3):.1f} ohms")

zdiff, zodd = differential_microstrip_z0(0.2, 0.15, 0.1, 0.035, 4.3)
print(f"Zdiff = {zdiff:.1f} ohms (Zodd = {zodd:.1f})")
```

## Width for a target impedance

Bisection solvers over the formulas above (search range 0.05–5.0 mm; return
0 when the target is unreachable):

```python
microstrip_width_for_z0(z0_target, h, t, er,
                        tolerance=0.1, max_iterations=50) -> float
stripline_width_for_z0(z0_target, h, t, er,
                       tolerance=0.1, max_iterations=50) -> float
differential_microstrip_width_for_z0(zdiff_target, s, h, t, er,
                                     tolerance=0.5, max_iterations=50) -> float
differential_stripline_width_for_z0(zdiff_target, s, h, t, er,
                                    tolerance=0.5, max_iterations=50) -> float
```

```python
from impedance import microstrip_width_for_z0
w = microstrip_width_for_z0(50.0, h=0.2, t=0.035, er=4.3)
print(f"50 ohm microstrip needs w = {w:.3f} mm")
```

Note `IMPEDANCE_WIDTH_SCALE = 0.90`: the *stackup-aware* functions below
multiply solved widths by 0.90, because the closed-form formulas
systematically overestimate width compared to field solvers and online
calculators. The raw `*_width_for_z0` solvers do **not** apply it.

## Stackup-aware calculations

These read geometry and Er from the parsed board's stackup
(`pcb.board_info.stackup`); layers default to Er = 4.0 when the stackup
doesn't specify it.

```python
get_layer_impedance_params(pcb, layer_name) -> Optional[LayerImpedanceParams]
```

Resolves a copper layer to its impedance context: copper thickness,
dielectric height(s), Er, and whether it's microstrip (outer) or stripline
(inner). `LayerImpedanceParams` fields: `layer_name`, `copper_thickness`,
`dielectric_height`, `dielectric_constant`, `is_outer_layer`,
`height_above`, `height_below`, `er_above`, `er_below`.

```python
calculate_impedance_for_layer(pcb, layer_name, trace_width,
                              spacing=0.0) -> dict
```

Impedance of a given width on a given layer. Returns
`{'layer', 'is_microstrip', 'z0', 'params', ...}` plus `'zdiff'`/`'zodd'`
when `spacing > 0`. Picks microstrip vs stripline automatically, and the
asymmetric stripline formula when the two plane distances differ by > 10%.

```python
calculate_width_for_impedance(pcb, layer_name, target_z0,
                              spacing=0.0, is_differential=False) -> dict
```

Inverse: returns `{'calculated_width_mm', 'calculated_width_mils',
'verified_z0', ...}` (or an `'error'` key).

```python
calculate_layer_widths_for_impedance(pcb, layers, target_z0,
                                     spacing=0.0, is_differential=False,
                                     fallback_width=0.1,
                                     min_width=0.0) -> Dict[str, float]
```

The function impedance-controlled routing actually uses: width per layer
(0.90-scaled, clamped to `min_width`, `fallback_width` on failure), ready to
assign to `GridRouteConfig.layer_widths`.

```python
from kicad_parser import parse_kicad_pcb
from impedance import calculate_layer_widths_for_impedance

pcb = parse_kicad_pcb('kicad_files/test_diffpair_ram.kicad_pcb')
layers = pcb.board_info.copper_layers
widths = calculate_layer_widths_for_impedance(pcb, layers, target_z0=50.0)
for layer, w in widths.items():
    print(f"{layer:8s} -> {w:.3f} mm for 50 ohms")
```

## Propagation delay

For time matching (`--time-matching`): signals travel faster on outer
layers (microstrip, part of the field in air) than inner ones (stripline).

```python
get_layer_epsilon_eff(pcb, layer_name) -> float   # (er+1)/2 microstrip, er stripline
get_layer_ps_per_mm(pcb, layer_name) -> float
get_via_barrel_epsilon_eff(pcb, layer1, layer2) -> float
calculate_route_propagation_time_ps(segments, vias=None,
                                    pcb_data=None) -> float
```

`calculate_route_propagation_time_ps` sums per-segment delay by layer plus
via barrel delays. Without `pcb_data` it assumes FR4 microstrip
(eps_eff ≈ 2.65) everywhere.

```python
from kicad_parser import parse_kicad_pcb
from impedance import get_layer_ps_per_mm, calculate_route_propagation_time_ps

pcb = parse_kicad_pcb('kicad_files/test_diffpair_ram.kicad_pcb')
for layer in pcb.board_info.copper_layers:
    print(f"{layer:8s} {get_layer_ps_per_mm(pcb, layer):.2f} ps/mm")

net = max(pcb.nets.values(), key=lambda n: sum(
    1 for s in pcb.segments if s.net_id == n.net_id))
segs = [s for s in pcb.segments if s.net_id == net.net_id]
vias = [v for v in pcb.vias if v.net_id == net.net_id]
print(f"{net.name}: {calculate_route_propagation_time_ps(segs, vias, pcb):.1f} ps")
```

## Reports

```python
from kicad_parser import parse_kicad_pcb
from impedance import print_stackup_impedance_table, print_impedance_routing_plan

pcb = parse_kicad_pcb('kicad_files/test_diffpair_ram.kicad_pcb')
print_stackup_impedance_table(pcb, trace_width=0.15, spacing=0.15)
print_impedance_routing_plan(pcb, pcb.board_info.copper_layers, target_z0=50.0)
```

Human-readable tables: Z0/Zdiff per layer at a given geometry, and the
per-layer width plan for a target impedance (the same output `route.py
--impedance` prints).

## Gotchas

- **No stackup → defaults.** Boards without a stackup section fall back to
  Er 4.0 and generic geometry; results are then rough. Run
  `/recommend-stackup` to set up a real stackup first.
- **The 0.90 width scale** applies only in the stackup-aware
  `calculate_*` functions, not the raw solvers.
- **These are closed-form approximations** (IPC-2141 / Hammerstad), good to
  a few percent in normal geometries — fine for routing decisions, not a
  substitute for your fab's impedance calculator on critical interfaces.

# Impedance API (`impedance`)

Characteristic-impedance formulas (IPC-2141 style), inverse solving (width
for a target impedance), stackup integration, and propagation-delay
calculation. This module powers `--impedance` routing and `--time-matching`.

Units: dimensions in mm, impedance in Ω, time in ps.

## Contents

- [Closed-form formulas](#closed-form-formulas) (no board needed)
- [Coplanar waveguide over ground](#coplanar-waveguide-over-ground) (#486)
- [Width for a target impedance](#width-for-a-target-impedance)
- [Stackup-aware calculations](#stackup-aware-calculations) (from a parsed board)
- [Propagation delay](#propagation-delay)
- [Reports](#reports)
- [What the model assumes](#what-the-model-assumes) — **read this before trusting a number**

## Closed-form formulas

Pure functions of geometry and dielectric — usable standalone:

```python
microstrip_z0(w, h, t, er) -> float            # outer layer, IPC-2141
stripline_z0(w, h, t, er) -> float             # symmetric inner layer
stripline_z0_asymmetric(w, h1, h2, t, er) -> float
differential_microstrip_z0(w, s, h, t, er) -> (zdiff, zodd)
differential_stripline_z0(w, s, h, t, er) -> (zdiff, zodd)
differential_stripline_z0_asymmetric(w, s, h1, h2, t, er) -> (zdiff, zodd)
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

## Coplanar waveguide over ground

A trace running *through a ground pour on its own layer* is not a microstrip.
The side copper adds capacitance and pulls Z0 down — hard. On a 0.2 mm
dielectric a 0.36 mm trace is ~51 Ω as a microstrip but only ~34 Ω with ground
0.1 mm away on each side. Routing a microstrip-derived width through a pour is
the error #486 describes.

```python
cpwg_z0(w, s, h, t, er) -> float                    # grounded CPW (CBCPW)
cpwg_epsilon_eff(w, s, h, t, er) -> float
cpwg_applies(s, h, ratio_limit=3.0) -> bool
differential_cpwg_z0(w, s_pair, s_gnd, h, t, er) -> (zdiff, zodd)
complete_elliptic_k(k) -> float                     # K(k), by AGM
```

- `s` — **side gap**, trace edge to coplanar ground edge
- `s_pair` / `s_gnd` — P-to-N spacing vs. the gap to coplanar ground

The model is the standard conformal-mapping solution (Ghione & Naldi; Simons,
*Coplanar Waveguide Circuits, Components and Systems*, ch. 3). Two limits fall
straight out and are pinned by the unit tests: `h → ∞` (no plane below) gives
the ungrounded-CPW value `(er+1)/2`, and `h → 0` gives `er`.

```python
from impedance import cpwg_z0, microstrip_z0, cpwg_applies

for gap in (0.1, 0.2, 0.4, 1.0):
    print(f"gap {gap}mm: Z0 = {cpwg_z0(0.36, gap, 0.2, 0.035, 4.3):5.1f} ohm"
          f"  (coplanar-coupled: {cpwg_applies(gap, 0.2)})")
print(f"microstrip:  Z0 = {microstrip_z0(0.36, 0.2, 0.035, 4.3):5.1f} ohm")
```

**Domain.** Accurate while the side gap is comparable to or smaller than `h`.
As `s/h` grows the conformal map degrades — it does *not* converge cleanly to
the microstrip answer but drifts toward `er_eff → er`. Use `cpwg_applies()`
(default: `s ≤ 3h`) to decide which model governs rather than trusting
`cpwg_z0` at a wide gap.

`differential_cpwg_z0` is an **approximation**, in the same spirit as
`differential_microstrip_z0`: single-ended CPWG impedance reduced by the
edge-coupling factor for the pair spacing. A full treatment needs the coupled
six-conductor map. It captures the first-order effect — coplanar ground pulls
Zdiff down just as it pulls Z0 down.

### Declaring coplanar routing

Because the pour does not exist when `route.py` runs, the gap is a
**declaration**, not something the router can measure:

```bash
# 1. route, declaring the intended gap (narrower trace than microstrip)
python3 py_router/route.py board.kicad_pcb routed.kicad_pcb \
    --impedance 50 --coplanar-gap 0.2 --coplanar-nets "RF_*"

# 2. pour the plane with a MATCHING zone clearance
python3 py_router/route_planes.py routed.kicad_pcb poured.kicad_pcb \
    --nets GND GND --plane-layers F.Cu B.Cu --zone-clearance 0.2

# 3. verify the declaration actually held
python3 py_tools/check_impedance.py poured.kicad_pcb --coplanar-gap 0.2
```

Omitting `--coplanar-nets` treats every net in the call as coplanar.
`route_diff.py` takes `--coplanar-gap` but has no `--coplanar-nets`: the
diff engine bakes one width per layer into the obstacle map, so split
interfaces into separate calls to mix.

Nothing enforces the gap during routing — a coplanar-declared net whose pour
never arrives is simply routed at a width that assumes a ground it does not
have. Step 3 is what catches that.

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
stripline_asymmetric_width_for_z0(z0_target, h1, h2, t, er,
                                  tolerance=0.1, max_iterations=50) -> float
differential_stripline_asymmetric_width_for_z0(zdiff_target, s, h1, h2, t, er,
                                               tolerance=0.5, max_iterations=50) -> float
cpwg_width_for_z0(z0_target, s, h, t, er,
                  tolerance=0.1, max_iterations=50) -> float
differential_cpwg_width_for_z0(zdiff_target, s_pair, s_gnd, h, t, er,
                               tolerance=0.5, max_iterations=50) -> float
```

```python
from impedance import microstrip_width_for_z0
w = microstrip_width_for_z0(50.0, h=0.2, t=0.035, er=4.3)
print(f"50 ohm microstrip needs w = {w:.3f} mm")
```

`IMPEDANCE_WIDTH_SCALE` is **1.0** as of #486. It used to be 0.90, described as
compensating for formulas that "overestimate width". That diagnosis was
backwards: `microstrip_z0`'s thickness correction applied the full *air-medium*
width widening inside the dielectric, which under-reported Z0 and made the
solver return traces ~12% too narrow on a 0.1 mm dielectric — and the 0.90
scale then narrowed them another 10% in the **same** direction. Combined, a
nominal 50 Ω trace came out at 0.144 mm where 0.181 mm is right (≈57 Ω).

With the Hammerstad–Jensen correction in place, solved widths land within ~1%
of the reference across h = 0.1–1.6 mm, so no fudge is warranted. The constant
is kept as the documented knob for biasing widths to a fab's measured coupon
data.

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
                                     min_width=0.0,
                                     coplanar_gap=0.0,
                                     floor_desc="--track-width; ...",
                                     clamp_report=None) -> Dict[str, float]
```

The function impedance-controlled routing actually uses: width per layer
(`IMPEDANCE_WIDTH_SCALE`-scaled, clamped to `min_width`, `fallback_width` on
failure), ready to assign to `GridRouteConfig.layer_widths`. Pass a dict as
`clamp_report` to collect `{layer: [solved_mm, floor_mm]}` for every clamped
layer — the routing engines forward it to `JSON_SUMMARY` as
`impedance_width_clamped` (#610). `floor_desc` names `min_width`'s source in
the clamp warning.

```python
impedance_width_floor(track_width, width_from_class,
                      copper_layer_count) -> Tuple[float, str]
```

The `min_width` the engines pass (#610): an explicit `--track-width`
(`width_from_class=False`) is honored verbatim; with the flag omitted the
impedance request sets the floor it implies, bounded below only by the active
fab tier's track minimum — so `--impedance 90` alone yields 90 Ω geometry
instead of being silently clamped to the 0.3 mm default width. Returns
`(floor_mm, floor_desc)`.

```python
from kicad_parser import parse_kicad_pcb
from impedance import calculate_layer_widths_for_impedance

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
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

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
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

pcb = parse_kicad_pcb('kicad_files/routed_output.kicad_pcb')
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
- **`IMPEDANCE_WIDTH_SCALE` is 1.0** (#486) and applies only in the
  stackup-aware `calculate_*` functions, not the raw solvers.
- **These are closed-form approximations** (IPC-2141 / Hammerstad), good to
  a few percent in normal geometries — fine for routing decisions, not a
  substitute for your fab's impedance calculator on critical interfaces.

## What the model assumes

Every number here rests on assumptions the formulas cannot check. #486 exists
because two of them were silently wrong on real boards.

- **The reference plane is ASSUMED, never verified.** Microstrip vs. stripline
  is chosen purely by the trace's *index in the stackup*
  (`get_layer_impedance_params`). Nothing confirms the adjacent layer actually
  has copper **under the trace's path**. A trace crossing a gap or slot in its
  reference plane still gets the ideal-plane number — the classic return-path
  discontinuity, which inflates Z0, wrecks the return current and radiates.
  `check_impedance.py` measures this on the routed board; the formulas here
  cannot.
- **The only ground in the microstrip/stripline model is the plane(s).** Side
  copper on the trace's own layer is ignored unless you use the CPWG functions
  and tell them the gap.
- **Widths are resolved per LAYER, before routing.**
  `calculate_layer_widths_for_impedance` returns a static `{layer: width}` dict,
  so it cannot react to where a trace actually ends up. That is why the coplanar
  gap is a declaration you then verify.
- **Er comes from the stackup, at one frequency.** No dispersion, no
  copper-roughness loss, no resin-content variation across the panel.
- **Asymmetric stripline uses the harmonic mean** of the two plane distances,
  and the solver and the verifier now agree on that choice
  (`is_asymmetric_stripline` is the single source of truth) on the
  single-ended **and** the differential path. Before #486 the
  solver bisected the *symmetric* model at the averaged height while the
  verifier scored the asymmetric one, so a solved width did not reproduce its
  own target — on hackrf_one's In1.Cu, 0.543 mm solved for 50 Ω verified at
  31 Ω. The correct width there is 0.287 mm. #607 was the same bug surviving on
  the *differential* path: on a 6-layer stack (0.1 mm prepreg, 0.55 mm core),
  `--impedance 90` returned 0.2385 mm on every inner layer and scored it 89.8 Ω
  when a field solver measured 67 Ω — and it erred *wide*, though a trace with
  planes on both sides must be narrower than the equivalent microstrip. It now
  solves 0.135 mm there, within ~3% of the solver-correct 0.131 mm.
- **The differential coupling factor is still calibrated for *symmetric*
  stripline.** The harmonic-mean substitution fixes the first-order error, not
  all of it: at a fixed width the asymmetric Zdiff prediction runs ~10% low
  against a method-of-moments solve. Because Zdiff is steep in `w`, the solved
  *width* is much closer (~3%), which is what routing consumes. Closing the rest
  needs a two-plane closed form or fab-calibrated tables. Deriving the coupling
  from the true plate separation `h1+h2+t` was tried and measured *worse*
  (~18% low), so the near plane appears to govern coupling as well as
  capacitance. Treat inner-layer differential targets on strongly asymmetric
  stacks as ~5% figures, and confirm against your fab's coupon data when the
  window is tight.

Post-route verification lives in `check_impedance.py`:

```bash
python3 py_tools/check_impedance.py routed.kicad_pcb            # reference-plane + side-gap audit
python3 py_tools/check_impedance.py routed.kicad_pcb --json report.json
```

It reports, per net: length over a reference-plane void, plane-split crossings,
the measured coplanar side-gap distribution, and the implied Z0 error against
what the route call assumed.

"""
Impedance calculation utilities for PCB trace design.

Provides formulas for calculating characteristic impedance of:
- Microstrip (outer layer traces with one reference plane)
- Stripline (inner layer traces between two reference planes)
- Differential pair versions of both

Based on IPC-2141 and industry-standard approximations.
"""

import math
from typing import Optional, Tuple, List, Dict
from dataclasses import dataclass

from kicad_parser import PCBData, StackupLayer


@dataclass
class LayerImpedanceParams:
    """Parameters needed for impedance calculation on a specific layer."""
    layer_name: str
    copper_thickness: float  # mm
    dielectric_height: float  # mm (distance to nearest reference plane)
    dielectric_constant: float  # Er
    is_outer_layer: bool  # True = microstrip, False = stripline
    # For stripline, heights to both reference planes
    height_above: Optional[float] = None  # mm
    height_below: Optional[float] = None  # mm
    er_above: Optional[float] = None
    er_below: Optional[float] = None


def microstrip_z0(w: float, h: float, t: float, er: float) -> float:
    """
    Calculate single-ended microstrip characteristic impedance.

    Uses IPC-2141 formulas with smooth transition between narrow and wide traces.

    Args:
        w: Trace width in mm
        h: Dielectric height (distance to reference plane) in mm
        t: Trace thickness (copper) in mm
        er: Dielectric constant (epsilon_r)

    Returns:
        Characteristic impedance Z0 in ohms
    """
    if w <= 0 or h <= 0 or er <= 0:
        return 0.0

    # Effective width accounting for trace thickness
    if t > 0:
        delta_w = (t / math.pi) * math.log(1 + 4 * math.e * h / (t * (1 + (h / w / 1.1)**2)))
        w_eff = w + delta_w
    else:
        w_eff = w

    u = w_eff / h

    # Effective dielectric constant (frequency-independent approximation)
    er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 / math.sqrt(1 + 12 / u))

    # Characteristic impedance using IPC-2141 formulas
    if u <= 1:
        # Narrow trace formula
        z0 = (60 / math.sqrt(er_eff)) * math.log(8 / u + 0.25 * u)
    else:
        # Wide trace formula
        z0 = (120 * math.pi / math.sqrt(er_eff)) / (u + 1.393 + 0.667 * math.log(u + 1.444))

    return max(z0, 0.0)


def microstrip_z0_hammerstad(w: float, h: float, t: float, er: float) -> float:
    """
    Calculate microstrip impedance using Hammerstad-Jensen formula.

    More accurate for a wider range of w/h ratios.

    Args:
        w: Trace width in mm
        h: Dielectric height in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Characteristic impedance Z0 in ohms
    """
    if w <= 0 or h <= 0 or er <= 0:
        return 0.0

    # Effective width correction for thickness
    if t > 0:
        delta_w = (t / math.pi) * math.log(1 + (4 * math.e * h) / (t * (1 / math.tanh(math.sqrt(6.517 * w / h)))**2))
        w_eff = w + delta_w
    else:
        w_eff = w

    u = w_eff / h

    # Effective dielectric constant
    a = 1 + (1/49) * math.log((u**4 + (u/52)**2) / (u**4 + 0.432)) + (1/18.7) * math.log(1 + (u/18.1)**3)
    b = 0.564 * ((er - 0.9) / (er + 3))**0.053
    er_eff = (er + 1) / 2 + ((er - 1) / 2) * (1 + 10/u)**(-a * b)

    # Characteristic impedance in free space
    f = 6 + (2 * math.pi - 6) * math.exp(-(30.666 / u)**0.7528)
    z0_air = (60 / math.sqrt(er_eff)) * math.log(f / u + math.sqrt(1 + (2/u)**2))

    return z0_air


def stripline_z0(w: float, h: float, t: float, er: float) -> float:
    """
    Calculate single-ended stripline characteristic impedance.

    For symmetric stripline (equal distance to both reference planes).
    Uses IPC-2141 approximation formulas.

    Args:
        w: Trace width in mm
        h: Distance to ONE reference plane in mm (total separation = 2*h + t)
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Characteristic impedance Z0 in ohms
    """
    if w <= 0 or h <= 0 or er <= 0:
        return 0.0

    # Total dielectric thickness between ground planes
    b = 2 * h + t

    # Effective width due to fringing (continuous formula)
    # For stripline, the effective width correction is smaller than microstrip
    m = 6 * h / (3 * h + t)
    w_eff = w + (t / math.pi) * (1 - 0.5 * math.log((t / (2 * h + t))**2 + (t / (m * math.pi * w + 1.1 * t))**2))

    # Stripline impedance formula (IPC-2141)
    cf = 1 / (1 + (w_eff / b / 0.35)**2.5)  # Correction factor for narrow traces
    z0 = (60 / math.sqrt(er)) * math.log(1.9 * b / (0.8 * w_eff + t) * (1 + cf * 0.4))

    return max(z0, 0.0)


def stripline_z0_asymmetric(w: float, h1: float, h2: float, t: float, er: float) -> float:
    """
    Calculate asymmetric stripline impedance.

    For traces not centered between reference planes.

    Args:
        w: Trace width in mm
        h1: Distance to upper reference plane in mm
        h2: Distance to lower reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Characteristic impedance Z0 in ohms
    """
    if w <= 0 or h1 <= 0 or h2 <= 0 or t <= 0 or er <= 0:
        return 0.0

    # For asymmetric stripline, use the harmonic mean approach
    # This is an approximation that works reasonably well
    h_eff = (2 * h1 * h2) / (h1 + h2)

    return stripline_z0(w, h_eff, t, er)


def differential_microstrip_z0(w: float, s: float, h: float, t: float, er: float) -> Tuple[float, float]:
    """
    Calculate differential microstrip impedance.

    Args:
        w: Trace width in mm
        s: Spacing between traces (edge to edge) in mm
        h: Dielectric height in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Tuple of (Zdiff, Zodd) where:
        - Zdiff: Differential impedance in ohms
        - Zodd: Odd-mode impedance in ohms
    """
    # Single-ended impedance
    z0 = microstrip_z0(w, h, t, er)

    if z0 <= 0 or s <= 0:
        return (0.0, 0.0)

    # Coupling factor for differential microstrip
    # Zdiff = 2 * Z0 * (1 - k) where k is coupling coefficient
    k = 0.48 * math.exp(-0.96 * s / h)

    z_odd = z0 * (1 - k)
    z_diff = 2 * z_odd

    return (z_diff, z_odd)


def differential_stripline_z0(w: float, s: float, h: float, t: float, er: float) -> Tuple[float, float]:
    """
    Calculate differential stripline (edge-coupled) impedance.

    Args:
        w: Trace width in mm
        s: Spacing between traces (edge to edge) in mm
        h: Distance to ONE reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Tuple of (Zdiff, Zodd) where:
        - Zdiff: Differential impedance in ohms
        - Zodd: Odd-mode impedance in ohms
    """
    # Single-ended impedance
    z0 = stripline_z0(w, h, t, er)

    if z0 <= 0 or s <= 0:
        return (0.0, 0.0)

    # Coupling factor for stripline (stronger coupling than microstrip)
    k = 0.347 * math.exp(-2.9 * s / (2 * h))

    z_odd = z0 * (1 - k)
    z_diff = 2 * z_odd

    return (z_diff, z_odd)


def microstrip_width_for_z0(z0_target: float, h: float, t: float, er: float,
                            tolerance: float = 0.1, max_iterations: int = 50) -> float:
    """
    Calculate trace width needed for target microstrip impedance.

    Uses iterative search (bisection method).

    Args:
        z0_target: Target impedance in ohms
        h: Dielectric height in mm
        t: Trace thickness in mm
        er: Dielectric constant
        tolerance: Acceptable error in ohms
        max_iterations: Maximum iterations for convergence

    Returns:
        Required trace width in mm, or 0 if not achievable
    """
    if z0_target <= 0 or h <= 0 or er <= 0:
        return 0.0

    # Search bounds (typical PCB trace widths)
    w_min = 0.05  # 50 microns
    w_max = 5.0   # 5mm

    # Check if target is achievable
    z_at_min = microstrip_z0(w_min, h, t, er)
    z_at_max = microstrip_z0(w_max, h, t, er)

    if z0_target > z_at_min or z0_target < z_at_max:
        # Target outside achievable range
        return 0.0

    # Bisection search (impedance decreases as width increases)
    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        z_mid = microstrip_z0(w_mid, h, t, er)

        if abs(z_mid - z0_target) < tolerance:
            return w_mid

        if z_mid > z0_target:
            w_min = w_mid  # Need wider trace
        else:
            w_max = w_mid  # Need narrower trace

    return (w_min + w_max) / 2


def stripline_width_for_z0(z0_target: float, h: float, t: float, er: float,
                           tolerance: float = 0.1, max_iterations: int = 50) -> float:
    """
    Calculate trace width needed for target stripline impedance.

    Args:
        z0_target: Target impedance in ohms
        h: Distance to ONE reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant
        tolerance: Acceptable error in ohms
        max_iterations: Maximum iterations for convergence

    Returns:
        Required trace width in mm, or 0 if not achievable
    """
    if z0_target <= 0 or h <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    z_at_min = stripline_z0(w_min, h, t, er)
    z_at_max = stripline_z0(w_max, h, t, er)

    if z0_target > z_at_min or z0_target < z_at_max:
        return 0.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        z_mid = stripline_z0(w_mid, h, t, er)

        if abs(z_mid - z0_target) < tolerance:
            return w_mid

        if z_mid > z0_target:
            w_min = w_mid
        else:
            w_max = w_mid

    return (w_min + w_max) / 2


def differential_microstrip_width_for_z0(zdiff_target: float, s: float, h: float, t: float, er: float,
                                         tolerance: float = 0.5, max_iterations: int = 50) -> float:
    """
    Calculate trace width needed for target differential microstrip impedance.

    Args:
        zdiff_target: Target differential impedance in ohms
        s: Spacing between traces in mm
        h: Dielectric height in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Required trace width in mm
    """
    if zdiff_target <= 0 or s <= 0 or h <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        zdiff, _ = differential_microstrip_z0(w_mid, s, h, t, er)

        if abs(zdiff - zdiff_target) < tolerance:
            return w_mid

        if zdiff > zdiff_target:
            w_min = w_mid
        else:
            w_max = w_mid

    return (w_min + w_max) / 2


def differential_stripline_width_for_z0(zdiff_target: float, s: float, h: float, t: float, er: float,
                                        tolerance: float = 0.5, max_iterations: int = 50) -> float:
    """
    Calculate trace width needed for target differential stripline impedance.

    Args:
        zdiff_target: Target differential impedance in ohms
        s: Spacing between traces in mm
        h: Distance to ONE reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Required trace width in mm
    """
    if zdiff_target <= 0 or s <= 0 or h <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        zdiff, _ = differential_stripline_z0(w_mid, s, h, t, er)

        if abs(zdiff - zdiff_target) < tolerance:
            return w_mid

        if zdiff > zdiff_target:
            w_min = w_mid
        else:
            w_max = w_mid

    return (w_min + w_max) / 2


def get_layer_impedance_params(pcb: PCBData, layer_name: str) -> Optional[LayerImpedanceParams]:
    """
    Get impedance calculation parameters for a specific copper layer.

    Extracts dielectric height, Er, and copper thickness from the stackup.

    Args:
        pcb: Parsed PCB data
        layer_name: Name of copper layer (e.g., "F.Cu", "In1.Cu")

    Returns:
        LayerImpedanceParams or None if layer not found
    """
    stackup = pcb.board_info.stackup
    if not stackup:
        return None

    # Find the layer index
    layer_idx = None
    for i, layer in enumerate(stackup):
        if layer.name == layer_name:
            layer_idx = i
            break

    if layer_idx is None:
        return None

    layer = stackup[layer_idx]
    if layer.layer_type != 'copper':
        return None

    copper_thickness = layer.thickness

    # Determine if outer layer (first or last copper layer)
    copper_indices = [i for i, l in enumerate(stackup) if l.layer_type == 'copper']
    is_outer = layer_idx == copper_indices[0] or layer_idx == copper_indices[-1]

    # Find adjacent dielectric layers
    # Look above (lower index)
    h_above = 0.0
    er_above = 4.0  # Default FR4
    for i in range(layer_idx - 1, -1, -1):
        if stackup[i].layer_type in ('core', 'prepreg'):
            h_above = stackup[i].thickness
            er_above = stackup[i].epsilon_r if stackup[i].epsilon_r > 0 else 4.0
            break

    # Look below (higher index)
    h_below = 0.0
    er_below = 4.0
    for i in range(layer_idx + 1, len(stackup)):
        if stackup[i].layer_type in ('core', 'prepreg'):
            h_below = stackup[i].thickness
            er_below = stackup[i].epsilon_r if stackup[i].epsilon_r > 0 else 4.0
            break

    if is_outer:
        # Microstrip - use the dielectric below for top layer, above for bottom
        if layer_idx == copper_indices[0]:
            # Top layer - reference plane is below
            return LayerImpedanceParams(
                layer_name=layer_name,
                copper_thickness=copper_thickness,
                dielectric_height=h_below,
                dielectric_constant=er_below,
                is_outer_layer=True,
                height_above=h_above,
                height_below=h_below,
                er_above=er_above,
                er_below=er_below
            )
        else:
            # Bottom layer - reference plane is above
            return LayerImpedanceParams(
                layer_name=layer_name,
                copper_thickness=copper_thickness,
                dielectric_height=h_above,
                dielectric_constant=er_above,
                is_outer_layer=True,
                height_above=h_above,
                height_below=h_below,
                er_above=er_above,
                er_below=er_below
            )
    else:
        # Stripline - between two reference planes
        # Use average Er if different
        er_avg = (er_above + er_below) / 2 if er_above > 0 and er_below > 0 else max(er_above, er_below)

        return LayerImpedanceParams(
            layer_name=layer_name,
            copper_thickness=copper_thickness,
            dielectric_height=(h_above + h_below) / 2,  # Average for symmetric approximation
            dielectric_constant=er_avg,
            is_outer_layer=False,
            height_above=h_above,
            height_below=h_below,
            er_above=er_above,
            er_below=er_below
        )


def calculate_impedance_for_layer(pcb: PCBData, layer_name: str, trace_width: float,
                                  spacing: float = 0.0) -> dict:
    """
    Calculate impedance for a trace on a specific layer.

    Args:
        pcb: Parsed PCB data
        layer_name: Copper layer name
        trace_width: Trace width in mm
        spacing: Spacing for differential pairs (0 for single-ended)

    Returns:
        Dictionary with impedance values:
        {
            'layer': layer name,
            'is_microstrip': bool,
            'z0': single-ended impedance,
            'zdiff': differential impedance (if spacing > 0),
            'params': LayerImpedanceParams
        }
    """
    params = get_layer_impedance_params(pcb, layer_name)
    if params is None:
        return {'error': f'Layer {layer_name} not found in stackup'}

    result = {
        'layer': layer_name,
        'is_microstrip': params.is_outer_layer,
        'params': params
    }

    if params.is_outer_layer:
        # Microstrip
        result['z0'] = microstrip_z0(
            trace_width,
            params.dielectric_height,
            params.copper_thickness,
            params.dielectric_constant
        )
        if spacing > 0:
            zdiff, zodd = differential_microstrip_z0(
                trace_width, spacing,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
            result['zdiff'] = zdiff
            result['zodd'] = zodd
    else:
        # Stripline
        # Use asymmetric formula if heights differ significantly
        if params.height_above and params.height_below:
            if abs(params.height_above - params.height_below) / max(params.height_above, params.height_below) > 0.1:
                result['z0'] = stripline_z0_asymmetric(
                    trace_width,
                    params.height_above,
                    params.height_below,
                    params.copper_thickness,
                    params.dielectric_constant
                )
            else:
                result['z0'] = stripline_z0(
                    trace_width,
                    params.dielectric_height,
                    params.copper_thickness,
                    params.dielectric_constant
                )
        else:
            result['z0'] = stripline_z0(
                trace_width,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )

        if spacing > 0:
            zdiff, zodd = differential_stripline_z0(
                trace_width, spacing,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
            result['zdiff'] = zdiff
            result['zodd'] = zodd

    return result


def calculate_width_for_impedance(pcb: PCBData, layer_name: str, target_z0: float,
                                  spacing: float = 0.0, is_differential: bool = False) -> dict:
    """
    Calculate required trace width for target impedance on a layer.

    Args:
        pcb: Parsed PCB data
        layer_name: Copper layer name
        target_z0: Target impedance (single-ended or differential based on is_differential)
        spacing: Spacing for differential pairs
        is_differential: If True, target_z0 is differential impedance

    Returns:
        Dictionary with calculated width and verification
    """
    params = get_layer_impedance_params(pcb, layer_name)
    if params is None:
        return {'error': f'Layer {layer_name} not found in stackup'}

    result = {
        'layer': layer_name,
        'target_z0': target_z0,
        'is_differential': is_differential,
        'params': params
    }

    if is_differential and spacing > 0:
        if params.is_outer_layer:
            width = differential_microstrip_width_for_z0(
                target_z0, spacing,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        else:
            width = differential_stripline_width_for_z0(
                target_z0, spacing,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
    else:
        if params.is_outer_layer:
            width = microstrip_width_for_z0(
                target_z0,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        else:
            width = stripline_width_for_z0(
                target_z0,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )

    result['calculated_width_mm'] = width
    result['calculated_width_mils'] = width * 39.3701  # Convert to mils

    # Verify by calculating impedance at this width
    if width > 0:
        verify = calculate_impedance_for_layer(pcb, layer_name, width, spacing)
        result['verified_z0'] = verify.get('zdiff' if is_differential else 'z0', 0)

    return result


def print_stackup_impedance_table(pcb: PCBData, trace_width: float = 0.15, spacing: float = 0.15):
    """
    Print a table showing impedance for each copper layer.

    Args:
        pcb: Parsed PCB data
        trace_width: Trace width in mm (default 0.15mm = ~6 mils)
        spacing: Differential pair spacing in mm
    """
    print(f"\nImpedance Table (width={trace_width}mm, spacing={spacing}mm)")
    print("=" * 85)
    print(f"{'Layer':<12} {'Type':<12} {'h(mm)':<8} {'Er':<6} {'t(mm)':<8} {'Z0(Ω)':<8} {'Zdiff(Ω)':<10}")
    print("-" * 85)

    for layer in pcb.board_info.stackup:
        if layer.layer_type == 'copper':
            result = calculate_impedance_for_layer(pcb, layer.name, trace_width, spacing)
            if 'error' not in result:
                params = result['params']
                layer_type = "Microstrip" if params.is_outer_layer else "Stripline"
                z0 = result.get('z0', 0)
                zdiff = result.get('zdiff', 0)
                print(f"{layer.name:<12} {layer_type:<12} {params.dielectric_height:<8.4f} "
                      f"{params.dielectric_constant:<6.2f} {params.copper_thickness:<8.4f} "
                      f"{z0:<8.1f} {zdiff:<10.1f}")

    print("=" * 85)


# Empirical scaling factor to match online calculators (formulas tend to overestimate width)
IMPEDANCE_WIDTH_SCALE = 0.90


def calculate_layer_widths_for_impedance(pcb: PCBData, layers: List[str], target_z0: float,
                                         spacing: float = 0.0, is_differential: bool = False,
                                         fallback_width: float = 0.1,
                                         min_width: float = 0.0) -> Dict[str, float]:
    """
    Calculate trace widths for each layer to achieve target impedance.

    This is the main function for impedance-controlled routing.
    Returns a dictionary mapping layer names to widths.

    For differential pairs, the spacing is fixed (from --diff-pair-gap) and
    width is calculated to achieve the target differential impedance.

    Args:
        pcb: Parsed PCB data with stackup information
        layers: List of copper layer names to calculate for
        target_z0: Target impedance in ohms (single-ended or differential)
        spacing: Differential pair spacing in mm (required if is_differential=True)
        is_differential: If True, target_z0 is differential impedance
        fallback_width: Width to use if impedance calculation fails
        min_width: Minimum allowed track width (from --track-width parameter)

    Returns:
        Dict mapping layer name to trace width in mm
    """
    layer_widths = {}

    for layer_name in layers:
        result = calculate_width_for_impedance(
            pcb, layer_name, target_z0,
            spacing=spacing, is_differential=is_differential
        )

        if 'error' in result or result.get('calculated_width_mm', 0) <= 0:
            # Use fallback width if calculation fails
            layer_widths[layer_name] = fallback_width
        else:
            # Apply scaling factor to match online calculators
            calculated_width = result['calculated_width_mm'] * IMPEDANCE_WIDTH_SCALE

            # Enforce minimum width
            if calculated_width < min_width:
                print(f"  WARNING: {layer_name} calculated width {calculated_width:.4f}mm < min {min_width:.4f}mm, using min")
                calculated_width = min_width

            layer_widths[layer_name] = calculated_width

    return layer_widths


def print_impedance_routing_plan(pcb: PCBData, layers: List[str], target_z0: float,
                                 spacing: float = 0.0, is_differential: bool = False,
                                 min_width: float = 0.0):
    """
    Print the impedance-controlled routing plan showing width per layer.

    Args:
        pcb: Parsed PCB data
        layers: List of layers that will be used for routing
        target_z0: Target impedance in ohms
        spacing: Differential pair spacing in mm
        is_differential: Whether this is differential impedance
        min_width: Minimum allowed track width (for warning display)
    """
    imp_type = "differential" if is_differential else "single-ended"
    print(f"\nImpedance-Controlled Routing Plan: {target_z0}Ω {imp_type}")
    if is_differential:
        print(f"Differential pair spacing: {spacing}mm ({spacing * 39.3701:.2f} mil)")
    print(f"Width scaling factor: {IMPEDANCE_WIDTH_SCALE:.0%}")
    print("=" * 80)
    print(f"{'Layer':<12} {'Type':<12} {'Width(mm)':<12} {'Width(mil)':<12} {'Verified Z':<15}")
    print("-" * 80)

    for layer_name in layers:
        result = calculate_width_for_impedance(
            pcb, layer_name, target_z0,
            spacing=spacing, is_differential=is_differential
        )

        if 'error' in result:
            print(f"{layer_name:<12} {'ERROR':<12} {'-':<12} {'-':<12} {result['error']}")
        else:
            params = result.get('params')
            layer_type = "Microstrip" if params and params.is_outer_layer else "Stripline"
            raw_width = result.get('calculated_width_mm', 0)
            width_mm = raw_width * IMPEDANCE_WIDTH_SCALE
            width_mil = width_mm * 39.3701
            z_unit = "Ω diff" if is_differential else "Ω"

            # Calculate verified impedance at scaled width
            if width_mm > 0:
                verify = calculate_impedance_for_layer(pcb, layer_name, width_mm, spacing)
                verified = verify.get('zdiff' if is_differential else 'z0', 0)

                note = ""
                if width_mm < min_width:
                    note = f" (clamped to min {min_width:.3f}mm)"
                    width_mm = min_width
                    width_mil = width_mm * 39.3701

                print(f"{layer_name:<12} {layer_type:<12} {width_mm:<12.4f} {width_mil:<12.2f} {verified:.1f} {z_unit}{note}")
            else:
                print(f"{layer_name:<12} {layer_type:<12} {'N/A':<12} {'N/A':<12} {'Not achievable'}")

    print("=" * 80)


# =============================================================================
# Propagation Time Calculations (for time-matching)
# =============================================================================

SPEED_OF_LIGHT_MM_PER_NS = 299.792458  # mm/ns (speed of light in vacuum)


def get_layer_epsilon_eff(pcb: PCBData, layer_name: str) -> float:
    """
    Get effective dielectric constant for propagation delay calculation.

    For microstrip (outer layers): epsilon_eff = (Er + 1) / 2
    For stripline (inner layers): epsilon_eff = Er

    Args:
        pcb: Parsed PCB data with stackup
        layer_name: Copper layer name (e.g., "F.Cu", "In1.Cu")

    Returns:
        Effective dielectric constant (defaults to 4.0 for FR4 if layer not found)
    """
    params = get_layer_impedance_params(pcb, layer_name)
    if params is None:
        return 4.0  # Default FR4

    er = params.dielectric_constant
    if params.is_outer_layer:
        # Microstrip: field partially in air, partially in dielectric
        return (er + 1) / 2
    else:
        # Stripline: field entirely in dielectric
        return er


def get_layer_ps_per_mm(pcb: PCBData, layer_name: str) -> float:
    """
    Get propagation delay in picoseconds per millimeter for a layer.

    Uses the layer's effective dielectric constant to calculate signal velocity:
    v = c / sqrt(epsilon_eff)
    delay = 1/v = sqrt(epsilon_eff) / c

    Args:
        pcb: Parsed PCB data with stackup
        layer_name: Copper layer name

    Returns:
        Propagation delay in ps/mm
    """
    eps_eff = get_layer_epsilon_eff(pcb, layer_name)
    speed_mm_per_ns = SPEED_OF_LIGHT_MM_PER_NS / math.sqrt(eps_eff)
    # Convert ns/mm to ps/mm (1 ns = 1000 ps)
    return 1000.0 / speed_mm_per_ns


def get_via_barrel_epsilon_eff(pcb: PCBData, layer1: str, layer2: str) -> float:
    """
    Get effective dielectric constant for a via barrel between two layers.

    Calculates a thickness-weighted average of dielectric constants through
    the via span.

    Args:
        pcb: Parsed PCB data with stackup
        layer1: First copper layer
        layer2: Second copper layer

    Returns:
        Weighted average epsilon_eff for the via barrel
    """
    stackup = pcb.board_info.stackup
    if not stackup:
        return 4.0  # Default FR4

    # Find layer indices
    idx1 = idx2 = -1
    for i, layer in enumerate(stackup):
        if layer.name == layer1:
            idx1 = i
        elif layer.name == layer2:
            idx2 = i

    if idx1 < 0 or idx2 < 0:
        return 4.0

    start_idx = min(idx1, idx2)
    end_idx = max(idx1, idx2)

    # Calculate thickness-weighted average epsilon
    total_thickness = 0.0
    weighted_epsilon = 0.0

    for i in range(start_idx, end_idx + 1):
        layer = stackup[i]
        if layer.layer_type in ('core', 'prepreg'):
            er = layer.epsilon_r if layer.epsilon_r > 0 else 4.0
            weighted_epsilon += er * layer.thickness
            total_thickness += layer.thickness

    if total_thickness <= 0:
        return 4.0

    return weighted_epsilon / total_thickness


def calculate_route_propagation_time_ps(
    segments: List,
    vias: List = None,
    pcb_data: PCBData = None
) -> float:
    """
    Calculate total propagation time for a route in picoseconds.

    Sums the propagation time for each segment based on its layer's
    effective dielectric constant, plus via barrel delays.

    Args:
        segments: List of Segment objects
        vias: Optional list of Via objects
        pcb_data: PCB data with stackup (required for layer-specific delays)

    Returns:
        Total propagation time in picoseconds
    """
    if pcb_data is None:
        # Fallback: assume FR4 microstrip everywhere
        from net_queries import calculate_route_length
        length = calculate_route_length(segments, vias, None)
        # Default FR4 microstrip: eps_eff = (4.3 + 1) / 2 = 2.65
        default_ps_per_mm = 1000.0 / (SPEED_OF_LIGHT_MM_PER_NS / math.sqrt(2.65))
        return length * default_ps_per_mm

    total_time_ps = 0.0

    # Time for each segment
    for seg in segments:
        dx = seg.end_x - seg.start_x
        dy = seg.end_y - seg.start_y
        length_mm = math.sqrt(dx * dx + dy * dy)
        ps_per_mm = get_layer_ps_per_mm(pcb_data, seg.layer)
        total_time_ps += length_mm * ps_per_mm

    # Time for via barrels
    if vias:
        for via in vias:
            if via.layers and len(via.layers) >= 2:
                barrel_length = pcb_data.get_via_barrel_length(via.layers[0], via.layers[1])
                if barrel_length > 0:
                    eps_eff = get_via_barrel_epsilon_eff(pcb_data, via.layers[0], via.layers[1])
                    speed = SPEED_OF_LIGHT_MM_PER_NS / math.sqrt(eps_eff)
                    via_time_ps = (barrel_length / speed) * 1000.0
                    total_time_ps += via_time_ps

    return total_time_ps

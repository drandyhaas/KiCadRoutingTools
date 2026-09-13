"""
Impedance calculation utilities for PCB trace design.

Provides formulas for calculating characteristic impedance of:
- Microstrip (outer layer traces with one reference plane)
- Stripline (inner layer traces between two reference planes)
- Differential pair versions of both

Based on IPC-2141 and industry-standard approximations.
"""
from __future__ import annotations

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


def _microstrip_z0_air(u: float) -> float:
    """Microstrip impedance of the SAME geometry in a homogeneous air medium.

    This is the IPC-2141 / Hammerstad shape function with er_eff factored out:
    the published Z0 expressions are exactly ``_microstrip_z0_air(u) /
    sqrt(er_eff)``. Isolating it lets the thickness correction below use the
    Hammerstad-Jensen er_eff reduction, which is expressed as a ratio of two
    air-medium impedances.
    """
    if u <= 0:
        return 0.0
    if u <= 1:
        return 60.0 * math.log(8.0 / u + 0.25 * u)
    return (120.0 * math.pi) / (u + 1.393 + 0.667 * math.log(u + 1.444))


def microstrip_z0(w: float, h: float, t: float, er: float) -> float:
    """
    Calculate single-ended microstrip characteristic impedance.

    Uses IPC-2141 formulas with smooth transition between narrow and wide
    traces, plus the Hammerstad-Jensen finite-thickness correction.

    Args:
        w: Trace width in mm
        h: Dielectric height (distance to reference plane) in mm
        t: Trace thickness (copper) in mm
        er: Dielectric constant (epsilon_r)

    Returns:
        Characteristic impedance Z0 in ohms

    Note on the thickness correction (#486): copper thickness widens the trace
    ELECTRICALLY, and the widening is not the same in air as in the dielectric.
    Hammerstad-Jensen give an air-medium widening dw1 and a smaller
    dielectric-medium widening dwr = 0.5*(1 + 1/cosh(sqrt(er-1))) * dw1 -- for
    FR4 a factor of ~0.66. The previous implementation applied the FULL air
    widening in the dielectric and skipped the er_eff thickness reduction
    entirely, which depressed Z0 by ~10% on a 0.1mm dielectric (vs ~4% real).
    The width solver then handed back traces ~12% too narrow, and
    IMPEDANCE_WIDTH_SCALE narrowed them another 10% in the SAME direction --
    see that constant's note. At t=0 this function is bit-identical to the old
    one; the whole change lives in the t>0 branch.
    """
    if w <= 0 or h <= 0 or er <= 0:
        return 0.0

    u = w / h

    if t > 0:
        # Hammerstad-Jensen air-medium width correction. coth^2 is written as
        # 1/tanh^2 to keep the small-u branch well conditioned.
        th = math.tanh(math.sqrt(6.517 * u))
        du1 = (t / math.pi) * math.log(1.0 + (4.0 * math.e * h / t) * th * th)
        # ...reduced for the inhomogeneous (dielectric) medium.
        du_r = 0.5 * (1.0 + 1.0 / math.cosh(math.sqrt(max(er - 1.0, 0.0)))) * du1
        u1 = (w + du1) / h
        ur = (w + du_r) / h
    else:
        u1 = ur = u

    # Effective dielectric constant (frequency-independent approximation)
    er_eff = (er + 1) / 2 + (er - 1) / 2 * (1 / math.sqrt(1 + 12 / ur))

    # Thickness reduction of er_eff: a thicker trace puts proportionally more
    # field in the air above it. Unity when t == 0.
    if t > 0:
        z_air_1 = _microstrip_z0_air(u1)
        z_air_r = _microstrip_z0_air(ur)
        if z_air_r > 0:
            er_eff *= (z_air_1 / z_air_r) ** 2

    if er_eff <= 0:
        return 0.0

    # Written in the original two-branch form (rather than reusing
    # _microstrip_z0_air) so that at t == 0 this is bit-identical to the
    # pre-#486 function -- the only behavioural delta is the t > 0 thickness
    # handling above. Float ordering matters here: solved widths become track
    # widths, and a ULP is enough to flip an A* tie-break.
    if ur <= 1:
        z0 = (60 / math.sqrt(er_eff)) * math.log(8 / ur + 0.25 * ur)
    else:
        z0 = (120 * math.pi / math.sqrt(er_eff)) / (ur + 1.393 + 0.667 * math.log(ur + 1.444))

    return max(z0, 0.0)


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


def differential_stripline_z0_asymmetric(w: float, s: float, h1: float, h2: float,
                                         t: float, er: float) -> Tuple[float, float]:
    """
    Differential (edge-coupled) stripline between UNEQUALLY spaced planes.

    The differential twin of stripline_z0_asymmetric: the effective height is
    the harmonic mean 2*h1*h2/(h1+h2), which is always SMALLER than the
    arithmetic average and therefore yields a lower Zdiff (a trace between two
    planes needs to be narrower than the equivalent microstrip, not wider).

    #607: calculate_impedance_for_layer tested is_asymmetric_stripline() for
    the single-ended answer and then called the symmetric differential model at
    the AVERAGED height three lines later, discarding the asymmetry. On a
    6-layer stack with 0.1mm prepreg and a 0.55mm core, --impedance 90 returned
    a width that the tool scored at 89.83 ohm and a field solver measured at
    67 -- outside USB's 81-99 window, and wide-of-target in the direction
    OPPOSITE to the physics.

    The harmonic-mean substitution is the same approximation the single-ended
    path has used since #486. It is a large improvement, not an exact answer:
    on the stackup above it moves the prediction from +34% to about -10%
    against a method-of-moments solve. The residual is the coupling factor
    itself, which is calibrated for symmetric stripline; closing it needs a
    two-plane closed form or fab-calibrated tables. Deriving the coupling from
    the true plate separation (h1+h2+t) instead was measured WORSE (-18%), so
    the near plane evidently governs the coupling as well as the capacitance.

    Args:
        w: Trace width in mm
        s: Spacing between traces (edge to edge) in mm
        h1: Distance to the upper reference plane in mm
        h2: Distance to the lower reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Tuple of (Zdiff, Zodd) in ohms
    """
    if w <= 0 or h1 <= 0 or h2 <= 0 or er <= 0 or s <= 0:
        return (0.0, 0.0)

    h_eff = (2 * h1 * h2) / (h1 + h2)

    return differential_stripline_z0(w, s, h_eff, t, er)


# =============================================================================
# Coplanar waveguide over ground (#486 part A)
#
# A trace running through a ground pour on its OWN layer is not a microstrip:
# the side copper adds capacitance and pulls Z0 down. The structure is a
# conductor-backed coplanar waveguide (CPWG, a.k.a. grounded CPW / CBCPW), and
# the standard model is the conformal-mapping solution (Ghione & Naldi; Simons,
# "Coplanar Waveguide Circuits, Components and Systems", ch. 3).
#
# Geometry, per side-gap `s`:
#     a = w/2                 half-width of the signal trace
#     b = w/2 + s             half-width out to the coplanar ground edge
#     h                       height to the ground plane BELOW
#
# DOMAIN. The model is accurate while the side gap is comparable to or smaller
# than h. As s/h grows the conformal map degrades: it does NOT converge to the
# microstrip answer but drifts towards er_eff -> er. Use `cpwg_applies()` to
# decide which model governs rather than trusting cpwg_z0 at a wide gap.
# =============================================================================

# Beyond this side-gap-to-dielectric-height ratio the coplanar ground is far
# enough that microstrip governs (the issue's ">3-5x h" rule of thumb).
CPWG_GAP_RATIO_LIMIT = 3.0


def complete_elliptic_k(k: float) -> float:
    """Complete elliptic integral of the first kind K(k), by the AGM.

    K(k) = pi / (2 * AGM(1, sqrt(1 - k^2))). The arithmetic-geometric mean
    converges quadratically, so ~5 iterations reach double precision.

    Implemented here rather than taken from scipy.special.ellipk on purpose:
    KiCad's bundled Python has no scipy, and the GUI shares this module.

    Args:
        k: Modulus, 0 <= k < 1

    Returns:
        K(k); returns a large finite value as k -> 1 (where K diverges).
    """
    if k <= 0.0:
        return math.pi / 2.0
    if k >= 1.0:
        return 1.0e12          # K diverges; a finite sentinel keeps ratios sane
    a = 1.0
    b = math.sqrt(1.0 - k * k)
    for _ in range(60):
        if abs(a - b) < 1e-15 * max(a, 1.0):
            break
        a, b = (a + b) / 2.0, math.sqrt(a * b)
    if a <= 0.0:
        return 1.0e12
    return math.pi / (2.0 * a)


def _k_ratio(k: float) -> float:
    """K(k)/K(k') with k' = sqrt(1 - k^2), guarded at both endpoints."""
    k = min(max(k, 0.0), 1.0)
    kp = math.sqrt(max(0.0, 1.0 - k * k))
    denom = complete_elliptic_k(kp)
    if denom <= 0.0:
        return 1.0e12
    return complete_elliptic_k(k) / denom


def cpwg_epsilon_eff(w: float, s: float, h: float, t: float, er: float) -> float:
    """Effective dielectric constant of a conductor-backed CPW.

    Args:
        w: Trace width in mm
        s: Side gap, trace edge to coplanar ground edge, in mm
        h: Height to the ground plane below, in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Effective epsilon (1 < er_eff <= er), or 0.0 on degenerate geometry.

    Two limits fall out of the formula and are covered by the unit tests:
    h -> inf (no plane below) gives the ungrounded-CPW value (er + 1) / 2, and
    h -> 0 gives er.
    """
    if w <= 0 or s <= 0 or h <= 0 or er <= 0:
        return 0.0

    # Finite-thickness correction: metal thickness widens the strip and
    # narrows the gap (Wadell / Simons).
    if t > 0:
        d = (1.25 * t / math.pi) * (1.0 + math.log(4.0 * math.pi * w / t))
        w_t = w + d
        s_t = max(s - d, 1e-6)
    else:
        w_t = w
        s_t = s

    a = w_t / 2.0
    b = a + s_t
    k1 = a / b
    # k3 carries the lower ground plane: -> k1 as h -> inf (plain CPW),
    # -> 1 as h -> 0 (the plane dominates and er_eff -> er).
    tb = math.tanh(math.pi * b / (2.0 * h))
    k3 = (math.tanh(math.pi * a / (2.0 * h)) / tb) if tb > 0 else 1.0

    r1 = _k_ratio(k1)          # K(k1)/K(k1')
    r3 = _k_ratio(k3)          # K(k3)/K(k3')
    if r1 <= 0:
        return 0.0
    q = r3 / r1                # == [K(k1')/K(k1)] * [K(k3)/K(k3')]

    er_eff = (1.0 + er * q) / (1.0 + q)

    # Thickness reduction of er_eff (more field in the air-filled gap).
    if t > 0 and s > 0:
        ts = t / s
        er_eff -= (0.7 * (er_eff - 1.0) * ts) / (r1 + 0.7 * ts)

    return max(1.0, min(er_eff, er))


def cpwg_z0(w: float, s: float, h: float, t: float, er: float) -> float:
    """
    Characteristic impedance of a conductor-backed coplanar waveguide.

    A trace of width `w` with coplanar ground `s` away on both sides and a
    ground plane `h` below. Reduces towards the microstrip answer as s grows,
    but only within the model's domain -- see CPWG_GAP_RATIO_LIMIT.

    Args:
        w: Trace width in mm
        s: Side gap (trace edge to coplanar ground edge) in mm
        h: Height to the ground plane below in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Z0 in ohms, or 0.0 on degenerate geometry.
    """
    if w <= 0 or s <= 0 or h <= 0 or er <= 0:
        return 0.0

    er_eff = cpwg_epsilon_eff(w, s, h, t, er)
    if er_eff <= 0:
        return 0.0

    if t > 0:
        d = (1.25 * t / math.pi) * (1.0 + math.log(4.0 * math.pi * w / t))
        w_t = w + d
        s_t = max(s - d, 1e-6)
    else:
        w_t = w
        s_t = s

    a = w_t / 2.0
    b = a + s_t
    k1 = a / b
    tb = math.tanh(math.pi * b / (2.0 * h))
    k3 = (math.tanh(math.pi * a / (2.0 * h)) / tb) if tb > 0 else 1.0

    denom = _k_ratio(k1) + _k_ratio(k3)
    if denom <= 0:
        return 0.0

    z0 = (60.0 * math.pi / math.sqrt(er_eff)) / denom
    return max(z0, 0.0)


def cpwg_applies(s: float, h: float, ratio_limit: float = CPWG_GAP_RATIO_LIMIT) -> bool:
    """Is the coplanar ground close enough to govern the impedance?

    True when s <= ratio_limit * h. Above that the side coupling is negligible,
    microstrip is the right model, and cpwg_z0 is outside its accurate domain.
    """
    if s <= 0 or h <= 0:
        return False
    return s <= ratio_limit * h


def differential_cpwg_z0(w: float, s_pair: float, s_gnd: float, h: float,
                         t: float, er: float) -> Tuple[float, float]:
    """
    Differential impedance of an edge-coupled pair inside a coplanar ground.

    APPROXIMATION, in the same spirit as differential_microstrip_z0: take the
    single-ended CPWG impedance (which already carries the side ground and the
    plane below) and apply the edge-coupling reduction for the pair spacing.
    A full treatment needs the coupled six-conductor conformal map; this
    captures the first-order effect, which is that coplanar ground pulls Zdiff
    down just as it pulls Z0 down.

    Args:
        w: Trace width in mm
        s_pair: Spacing between P and N, edge to edge, in mm
        s_gnd: Side gap from the OUTER edge of each trace to coplanar ground, mm
        h: Height to the ground plane below in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        (Zdiff, Zodd) in ohms.
    """
    z0 = cpwg_z0(w, s_gnd, h, t, er)
    if z0 <= 0 or s_pair <= 0 or h <= 0:
        return (0.0, 0.0)

    # Same coupling form as the microstrip pair.
    k = 0.48 * math.exp(-0.96 * s_pair / h)
    z_odd = z0 * (1 - k)
    return (2 * z_odd, z_odd)


def cpwg_width_for_z0(z0_target: float, s: float, h: float, t: float, er: float,
                      tolerance: float = 0.1, max_iterations: int = 50) -> float:
    """
    Trace width for a target impedance as a coplanar waveguide over ground.

    The coplanar ground pulls Z0 down, so the width that hits a given target is
    always NARROWER than the microstrip answer for the same stackup -- often
    dramatically so at tight gaps. Routing a microstrip-derived width through a
    ground pour is exactly the error #486 describes.

    Args:
        z0_target: Target impedance in ohms
        s: Side gap (trace edge to coplanar ground edge) in mm -- the DESIGN
           value, i.e. what the pour clearance will be
        h: Height to the ground plane below in mm
        t: Trace thickness in mm
        er: Dielectric constant
        tolerance: Acceptable error in ohms
        max_iterations: Bisection iterations

    Returns:
        Required trace width in mm, or 0.0 if the target is unreachable within
        the search bounds (e.g. a gap so tight that Z0 cannot be raised to the
        target at any width).
    """
    if z0_target <= 0 or s <= 0 or h <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    # Z0 decreases as width increases, same as microstrip.
    if z0_target > cpwg_z0(w_min, s, h, t, er):
        return 0.0
    if z0_target < cpwg_z0(w_max, s, h, t, er):
        return 0.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        z_mid = cpwg_z0(w_mid, s, h, t, er)

        if abs(z_mid - z0_target) < tolerance:
            return w_mid

        if z_mid > z0_target:
            w_min = w_mid  # Need wider trace
        else:
            w_max = w_mid  # Need narrower trace

    return (w_min + w_max) / 2


def differential_cpwg_width_for_z0(zdiff_target: float, s_pair: float,
                                   s_gnd: float, h: float, t: float, er: float,
                                   tolerance: float = 0.5,
                                   max_iterations: int = 50) -> float:
    """
    Trace width for a target differential impedance inside a coplanar ground.

    Args:
        zdiff_target: Target differential impedance in ohms
        s_pair: Spacing between P and N (edge to edge) in mm
        s_gnd: Side gap from each trace's OUTER edge to coplanar ground, mm
        h: Height to the ground plane below in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Required trace width in mm (0.0 if unreachable).
    """
    if zdiff_target <= 0 or s_pair <= 0 or s_gnd <= 0 or h <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    if zdiff_target > differential_cpwg_z0(w_min, s_pair, s_gnd, h, t, er)[0]:
        return 0.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        zdiff, _ = differential_cpwg_z0(w_mid, s_pair, s_gnd, h, t, er)

        if abs(zdiff - zdiff_target) < tolerance:
            return w_mid

        if zdiff > zdiff_target:
            w_min = w_mid
        else:
            w_max = w_mid

    return (w_min + w_max) / 2


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


def stripline_asymmetric_width_for_z0(z0_target: float, h1: float, h2: float,
                                      t: float, er: float,
                                      tolerance: float = 0.1,
                                      max_iterations: int = 50) -> float:
    """
    Trace width for a target impedance on an ASYMMETRIC stripline layer.

    Most real 4-layer stackups are asymmetric (a thin prepreg to the nearby
    plane, a thick core to the far one), and the effective height is the
    harmonic mean 2*h1*h2/(h1+h2), not the arithmetic average -- always the
    SMALLER of the two, so the correct trace is much narrower.

    #486: calculate_impedance_for_layer already switched to the asymmetric
    formula when the two heights differ by >10%, but the width SOLVER did not,
    so it bisected over the symmetric model at the averaged height. The two
    disagreed on the same layer: hackrf_one's In1.Cu solved to 0.543mm for a
    50-ohm target and then verified at 31 ohms. This solver closes that gap.

    Args:
        z0_target: Target impedance in ohms
        h1: Distance to the upper reference plane in mm
        h2: Distance to the lower reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Required trace width in mm, or 0.0 if not achievable.
    """
    if z0_target <= 0 or h1 <= 0 or h2 <= 0 or er <= 0:
        return 0.0

    w_min = 0.05
    w_max = 5.0

    if z0_target > stripline_z0_asymmetric(w_min, h1, h2, t, er):
        return 0.0
    if z0_target < stripline_z0_asymmetric(w_max, h1, h2, t, er):
        return 0.0

    for _ in range(max_iterations):
        w_mid = (w_min + w_max) / 2
        z_mid = stripline_z0_asymmetric(w_mid, h1, h2, t, er)

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


def differential_stripline_asymmetric_width_for_z0(zdiff_target: float, s: float,
                                                   h1: float, h2: float,
                                                   t: float, er: float,
                                                   tolerance: float = 0.5,
                                                   max_iterations: int = 50) -> float:
    """
    Trace width for a target DIFFERENTIAL impedance on an asymmetric stripline.

    Must bisect over the same model calculate_impedance_for_layer scores, or a
    solved width does not reproduce its own target -- the #486 failure, which
    #607 found still open on the differential path: the solver worked at the
    averaged height and handed back traces ~80% too wide on a 6-layer stack.

    Args:
        zdiff_target: Target differential impedance in ohms
        s: Spacing between traces (edge to edge) in mm
        h1: Distance to the upper reference plane in mm
        h2: Distance to the lower reference plane in mm
        t: Trace thickness in mm
        er: Dielectric constant

    Returns:
        Required trace width in mm, or 0.0 if the inputs are degenerate.
    """
    if zdiff_target <= 0 or s <= 0 or h1 <= 0 or h2 <= 0 or er <= 0:
        return 0.0

    h_eff = (2 * h1 * h2) / (h1 + h2)

    return differential_stripline_width_for_z0(zdiff_target, s, h_eff, t, er,
                                               tolerance=tolerance,
                                               max_iterations=max_iterations)


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


def is_asymmetric_stripline(params: LayerImpedanceParams,
                            tolerance: float = 0.1) -> bool:
    """Do this inner layer's two reference planes sit at different distances?

    Single source of truth for the symmetric/asymmetric choice, used by BOTH
    the Z0 calculation and the width solver. They each carried their own copy
    of this test and drifted (#486): the solver bisected the symmetric model
    while the verifier scored the asymmetric one, so a solved width did not
    reproduce its own target.
    """
    if params.is_outer_layer:
        return False
    ha, hb = params.height_above, params.height_below
    if not ha or not hb:
        return False
    return abs(ha - hb) / max(ha, hb) > tolerance


def calculate_impedance_for_layer(pcb: PCBData, layer_name: str, trace_width: float,
                                  spacing: float = 0.0,
                                  coplanar_gap: float = 0.0) -> dict:
    """
    Calculate impedance for a trace on a specific layer.

    Args:
        pcb: Parsed PCB data
        layer_name: Copper layer name
        trace_width: Trace width in mm
        spacing: Spacing for differential pairs (0 for single-ended)
        coplanar_gap: Side gap to same-layer ground pour in mm (#486). When
            > 0 on an OUTER layer, the trace is modelled as a coplanar
            waveguide over ground instead of a microstrip. 0 = microstrip,
            which is the historical behaviour. Ignored on inner layers, where
            the enclosing planes already dominate and the CPWG map (one plane
            below) does not apply.

    Returns:
        Dictionary with impedance values:
        {
            'layer': layer name,
            'is_microstrip': bool,
            'is_coplanar': bool (only when the CPWG model was used),
            'coplanar_gap': the gap used (only when coplanar),
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

    if params.is_outer_layer and coplanar_gap and coplanar_gap > 0 \
            and params.dielectric_height > 0:
        # Coplanar waveguide over ground (#486): the trace runs through a pour
        # on its OWN layer, `coplanar_gap` away on each side.
        result['is_coplanar'] = True
        result['coplanar_gap'] = coplanar_gap
        result['z0'] = cpwg_z0(
            trace_width, coplanar_gap,
            params.dielectric_height,
            params.copper_thickness,
            params.dielectric_constant
        )
        if spacing > 0:
            zdiff, zodd = differential_cpwg_z0(
                trace_width, spacing, coplanar_gap,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
            result['zdiff'] = zdiff
            result['zodd'] = zodd
    elif params.is_outer_layer:
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
        # Stripline. One asymmetry decision governs BOTH the single-ended and
        # the differential answer -- they drifted apart in #607, where the flag
        # was tested for z0 and ignored for zdiff three lines later.
        asymmetric = is_asymmetric_stripline(params)

        if asymmetric:
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

        if spacing > 0:
            if asymmetric:
                zdiff, zodd = differential_stripline_z0_asymmetric(
                    trace_width, spacing,
                    params.height_above,
                    params.height_below,
                    params.copper_thickness,
                    params.dielectric_constant
                )
            else:
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
                                  spacing: float = 0.0, is_differential: bool = False,
                                  coplanar_gap: float = 0.0) -> dict:
    """
    Calculate required trace width for target impedance on a layer.

    Args:
        pcb: Parsed PCB data
        layer_name: Copper layer name
        target_z0: Target impedance (single-ended or differential based on is_differential)
        spacing: Spacing for differential pairs
        is_differential: If True, target_z0 is differential impedance
        coplanar_gap: DESIGN side gap to same-layer ground pour in mm (#486).
            > 0 selects the coplanar-waveguide-over-ground model on outer
            layers, which yields a NARROWER trace than microstrip for the same
            target. This is a declaration, not a measurement: the pour does not
            exist yet at route time, so the plane step must be run with a
            matching zone clearance (route_planes --zone-clearance) and
            check_impedance.py verifies afterwards that it was achieved.

    Returns:
        Dictionary with calculated width and verification. 'is_coplanar' says
        which model was used.
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

    # A coplanar gap only changes the model on an OUTER layer (the CPWG map
    # has exactly one plane, below). Inner layers keep the stripline answer.
    use_cpwg = bool(coplanar_gap and coplanar_gap > 0
                    and params.is_outer_layer and params.dielectric_height > 0)
    result['is_coplanar'] = use_cpwg
    if use_cpwg:
        result['coplanar_gap'] = coplanar_gap

    if is_differential and spacing > 0:
        if use_cpwg:
            width = differential_cpwg_width_for_z0(
                target_z0, spacing, coplanar_gap,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        elif params.is_outer_layer:
            width = differential_microstrip_width_for_z0(
                target_z0, spacing,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        elif is_asymmetric_stripline(params):
            # Must match calculate_impedance_for_layer's model choice on the
            # differential path too, not just the single-ended one (#607).
            width = differential_stripline_asymmetric_width_for_z0(
                target_z0, spacing,
                params.height_above,
                params.height_below,
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
        if use_cpwg:
            width = cpwg_width_for_z0(
                target_z0, coplanar_gap,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        elif params.is_outer_layer:
            width = microstrip_width_for_z0(
                target_z0,
                params.dielectric_height,
                params.copper_thickness,
                params.dielectric_constant
            )
        elif is_asymmetric_stripline(params):
            # Must match calculate_impedance_for_layer's model choice, or the
            # solved width does not reproduce its own target (#486).
            width = stripline_asymmetric_width_for_z0(
                target_z0,
                params.height_above,
                params.height_below,
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
        verify = calculate_impedance_for_layer(pcb, layer_name, width, spacing,
                                               coplanar_gap=coplanar_gap)
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


# Width correction applied to every solved impedance width.
#
# RESOLVED (#486): this was 0.90, commented "to match online calculators
# (formulas tend to overestimate width)". That diagnosis was backwards. The
# formulas UNDER-estimated width, because microstrip_z0's thickness correction
# over-widened the trace electrically (see its note) and so under-reported Z0;
# the bisection solver compensated by returning a trace ~12% too NARROW on a
# 0.1mm dielectric. Scaling by 0.90 pushed it the same way, not the opposite --
# the two compounded to ~20% too narrow (0.144mm where 0.181mm is the
# reference 50-ohm width), i.e. ~57 ohms on a nominally 50-ohm trace.
#
# With the thickness correction fixed, solved widths land within ~1% of the
# Hammerstad-Jensen reference across h = 0.1..1.6mm, so no empirical fudge is
# warranted and this is 1.0. It is kept as a named constant (rather than
# deleted) because it is the documented knob for anyone who needs to bias
# widths to a specific fab's measured coupon data.
IMPEDANCE_WIDTH_SCALE = 1.0


def impedance_width_floor(track_width: float, width_from_class: bool,
                          copper_layer_count: int) -> Tuple[float, str]:
    """#610: the width floor impedance-solved widths are clamped to.

    An EXPLICIT --track-width (width_from_class=False) is honored verbatim --
    the operator asked for that geometry. With --track-width OMITTED, the
    impedance request sets the floor it implies, bounded below only by the
    active fab tier's track minimum: clamping solved widths to the resolved
    default width silently converted an electrical spec into a geometric one
    (90 ohm requested -> 0.3 mm ~45 ohm copper shipped, invisible to
    pipelines).

    Returns (floor_mm, floor_desc) where floor_desc names the floor's source
    for the clamp warning.
    """
    if not width_from_class:
        return track_width, "--track-width; lower it to reach the target"
    from fab_tiers import fab_floors
    floor = fab_floors(copper_layer_count or 4).get('track_width', 0.0) or 0.0
    return floor, ("the fab-tier track minimum (--track-width not given); the "
                   "target needs a narrower trace than this fab tier can build")


def calculate_layer_widths_for_impedance(pcb: PCBData, layers: List[str], target_z0: float,
                                         spacing: float = 0.0, is_differential: bool = False,
                                         fallback_width: float = 0.1,
                                         min_width: float = 0.0,
                                         coplanar_gap: float = 0.0,
                                         floor_desc: str = "--track-width; lower it to reach the target",
                                         clamp_report: Optional[Dict[str, List[float]]] = None,
                                         unsolved_report: Optional[List[str]] = None) -> Dict[str, float]:
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
        min_width: Minimum allowed track width (an explicit --track-width, or
            the fab-tier track floor when --track-width was omitted -- see
            impedance_width_floor, #610)
        coplanar_gap: DESIGN side gap to same-layer ground pour in mm (#486).
            > 0 uses the coplanar-waveguide-over-ground model on outer layers.
            Callers pass this straight through from --coplanar-gap.
        floor_desc: names min_width's source in the clamp warning (#610).
        clamp_report: optional dict the caller owns; every clamped layer is
            recorded as {layer: [solved_mm, floor_mm]} so the run summary can
            surface the clamp (#610 -- it was loud on a terminal and invisible
            in JSON_SUMMARY).
        unsolved_report: optional LIST the caller owns; every layer that fell
            back to ``fallback_width`` because the model could not solve it is
            appended (#906). The return value alone cannot say this -- a
            fallback and a genuine solve are the same kind of number, and a
            solved width may coincide with the fallback -- so a caller deciding
            whether an impedance was actually ACHIEVED must read this rather
            than compare widths. "The board has a stackup" is not the same
            question: a stackup listing copper with no adjacent dielectric, or
            with names that do not match the routed layers, solves nothing and
            every layer lands here.

    Returns:
        Dict mapping layer name to trace width in mm
    """
    layer_widths = {}

    for layer_name in layers:
        result = calculate_width_for_impedance(
            pcb, layer_name, target_z0,
            spacing=spacing, is_differential=is_differential,
            coplanar_gap=coplanar_gap
        )

        if 'error' in result or result.get('calculated_width_mm', 0) <= 0:
            # Use fallback width if calculation fails
            layer_widths[layer_name] = fallback_width
            if unsolved_report is not None:      # #906
                unsolved_report.append(layer_name)
        else:
            # Apply scaling factor to match online calculators
            calculated_width = result['calculated_width_mm'] * IMPEDANCE_WIDTH_SCALE

            # Enforce minimum width. Say what the clamp COSTS, not just that it
            # happened (#607): naming the impedance the clamped trace will
            # actually have makes that visible in a scripted run's log. #610
            # narrowed when this fires at all: with --track-width omitted the
            # callers pass the fab-tier floor here (impedance_width_floor), so
            # only an explicit --track-width or a genuinely unmanufacturable
            # target still clamps -- and the clamp lands in clamp_report so
            # JSON_SUMMARY consumers can detect it.
            if calculated_width < min_width:
                clamped_z = 0.0
                try:
                    _v = calculate_impedance_for_layer(pcb, layer_name, min_width,
                                                       spacing, coplanar_gap=coplanar_gap)
                    clamped_z = _v.get('zdiff' if is_differential else 'z0', 0) or 0.0
                except Exception:
                    pass
                note = (f" -- this trace will be ~{clamped_z:.0f} ohm, not "
                        f"{target_z0:g}") if clamped_z > 0 else ""
                print(f"  WARNING: {layer_name} calculated width {calculated_width:.4f}mm "
                      f"< min {min_width:.4f}mm, using min{note}")
                print(f"           (the width floor is {floor_desc})")
                if clamp_report is not None:
                    clamp_report[layer_name] = [round(calculated_width, 4),
                                                round(min_width, 4)]
                calculated_width = min_width

            layer_widths[layer_name] = calculated_width

    return layer_widths


def print_impedance_routing_plan(pcb: PCBData, layers: List[str], target_z0: float,
                                 spacing: float = 0.0, is_differential: bool = False,
                                 min_width: float = 0.0, coplanar_gap: float = 0.0):
    """
    Print the impedance-controlled routing plan showing width per layer.

    Args:
        pcb: Parsed PCB data
        layers: List of layers that will be used for routing
        target_z0: Target impedance in ohms
        spacing: Differential pair spacing in mm
        is_differential: Whether this is differential impedance
        min_width: Minimum allowed track width (for warning display)
        coplanar_gap: Design side gap to same-layer ground pour in mm (#486)
    """
    imp_type = "differential" if is_differential else "single-ended"
    print(f"\nImpedance-Controlled Routing Plan: {target_z0}Ω {imp_type}")
    if is_differential:
        print(f"Differential pair spacing: {spacing}mm ({spacing * 39.3701:.2f} mil)")
    if coplanar_gap and coplanar_gap > 0:
        print(f"Coplanar ground gap: {coplanar_gap}mm "
              f"({coplanar_gap * 39.3701:.2f} mil) -- outer layers use the "
              f"CPW-over-ground model")
        print(f"  NOTE: pour the plane layers with a matching zone clearance "
              f"(route_planes --zone-clearance {coplanar_gap}), then verify "
              f"with check_impedance.py --coplanar-gap {coplanar_gap}")
    if IMPEDANCE_WIDTH_SCALE != 1.0:
        print(f"Width scaling factor: {IMPEDANCE_WIDTH_SCALE:.0%}")
    print("=" * 80)
    print(f"{'Layer':<12} {'Type':<12} {'Width(mm)':<12} {'Width(mil)':<12} {'Verified Z':<15}")
    print("-" * 80)

    for layer_name in layers:
        result = calculate_width_for_impedance(
            pcb, layer_name, target_z0,
            spacing=spacing, is_differential=is_differential,
            coplanar_gap=coplanar_gap
        )

        if 'error' in result:
            print(f"{layer_name:<12} {'ERROR':<12} {'-':<12} {'-':<12} {result['error']}")
        else:
            params = result.get('params')
            if result.get('is_coplanar'):
                layer_type = "CPW-on-GND"
            elif params and params.is_outer_layer:
                layer_type = "Microstrip"
            else:
                layer_type = "Stripline"
            raw_width = result.get('calculated_width_mm', 0)
            width_mm = raw_width * IMPEDANCE_WIDTH_SCALE
            width_mil = width_mm * 39.3701
            z_unit = "Ω diff" if is_differential else "Ω"

            # Calculate verified impedance at scaled width
            if width_mm > 0:
                verify = calculate_impedance_for_layer(pcb, layer_name, width_mm, spacing,
                                                       coplanar_gap=coplanar_gap)
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


def get_layer_epsilon_eff(pcb: PCBData, layer_name: str,
                          width: float = 0.0, side_gap: float = 0.0) -> float:
    """
    Get effective dielectric constant for propagation delay calculation.

    For microstrip (outer layers): epsilon_eff = (Er + 1) / 2
    For stripline (inner layers): epsilon_eff = Er

    Args:
        pcb: Parsed PCB data with stackup
        layer_name: Copper layer name (e.g., "F.Cu", "In1.Cu")
        width: Optional trace width in mm. When given, outer layers use the
            width-dependent Hammerstad er_eff instead of the flat (Er + 1) / 2
            (#486: the flat form is a lower bound -- a real trace always has
            more field in the dielectric than it implies, so delays came out
            short). Inner layers are unaffected: stripline really is er.
        side_gap: Optional measured side gap to coplanar ground in mm. When
            given together with `width` on an outer layer, and the gap is
            close enough to matter (see cpwg_applies), the CPWG er_eff is
            used. It lands BETWEEN the two other forms: above the flat
            (Er + 1) / 2, but below the equivalent microstrip's, because the
            coplanar gaps are air-filled and hold part of the field. So a
            coplanar trace is slightly FASTER than a microstrip of the same
            width, and tightening the gap speeds it up further.

    Returns:
        Effective dielectric constant (defaults to 4.0 for FR4 if layer not found)

    Passing no width reproduces the pre-#486 result exactly, which is what
    every existing length/time-matching call site does.
    """
    params = get_layer_impedance_params(pcb, layer_name)
    if params is None:
        return 4.0  # Default FR4

    er = params.dielectric_constant
    if not params.is_outer_layer:
        # Stripline: field entirely in dielectric
        return er

    if width and width > 0 and params.dielectric_height > 0:
        h = params.dielectric_height
        t = params.copper_thickness
        if side_gap and side_gap > 0 and cpwg_applies(side_gap, h):
            ee = cpwg_epsilon_eff(width, side_gap, h, t, er)
            if ee > 0:
                return ee
        # Width-dependent microstrip er_eff (Hammerstad), the same expression
        # microstrip_z0 uses internally.
        u = width / h
        return (er + 1) / 2 + (er - 1) / 2 * (1 / math.sqrt(1 + 12 / u))

    # Microstrip, width unknown: field partially in air, partially in dielectric
    return (er + 1) / 2


#: A nominal FR4 stackup, for the ONE question that can be answered without a
#: real one: "is this target impedance achievable on a board like this at all?"
#: (#909). These are KiCad's own defaults for a 1.6 mm two-layer board -- see
#: `kicad_files/flat_hierarchy.kicad_pcb`, which stores exactly 0.035 mm copper
#: over a 1.51 mm er-4.5 core plus 0.01 mm masks. Nothing here WRITES a stackup;
#: the board's author owns that (`/recommend-stackup`), and a written default
#: would only make the width solver print a number and clamp it straight back.
NOMINAL_BOARD_THICKNESS_MM = 1.6
NOMINAL_COPPER_THICKNESS_MM = 0.035
NOMINAL_MASK_THICKNESS_MM = 0.01
NOMINAL_EPSILON_R = 4.5


def nominal_stackup(n_copper_layers: int = 2,
                    board_thickness: float = NOMINAL_BOARD_THICKNESS_MM):
    """A synthetic `List[StackupLayer]` for a board that declares none (#909).

    Copper of `NOMINAL_COPPER_THICKNESS_MM`, masks of
    `NOMINAL_MASK_THICKNESS_MM`, and the remaining height split evenly into
    er-`NOMINAL_EPSILON_R` dielectric between the copper layers. It is an
    ASSUMPTION and every caller must print it as one.
    """
    from kicad_parser import StackupLayer
    n = max(2, int(n_copper_layers))
    cu = NOMINAL_COPPER_THICKNESS_MM
    msk = NOMINAL_MASK_THICKNESS_MM
    diel_total = max(0.05, board_thickness - n * cu - 2 * msk)
    each = diel_total / (n - 1)
    names = (['F.Cu'] + [f'In{i}.Cu' for i in range(1, n - 1)] + ['B.Cu'])
    out = [StackupLayer('F.Mask', 'solder_mask', msk, 3.3, 0.0, 'mask')]
    for i, nm in enumerate(names):
        out.append(StackupLayer(nm, 'copper', cu, 0.0, 0.0, 'copper'))
        if i < n - 1:
            out.append(StackupLayer(f'dielectric {i + 1}',
                                    'core' if n == 2 else 'prepreg',
                                    each, NOMINAL_EPSILON_R, 0.02, 'FR4'))
    out.append(StackupLayer('B.Mask', 'solder_mask', msk, 3.3, 0.0, 'mask'))
    return out


def _impedance_scope_net_ids(pcb: PCBData, net_names) -> list:
    """Net ids the caller's `net_names` selects, globs included (#909).

    The scope is resolved here rather than reused from the routing loop
    because this note is printed BEFORE the loop resolves anything, and an
    empty/None selection means "every net", which is what the CLIs mean by it.
    """
    import fnmatch as _fn
    nets = getattr(pcb, 'nets', None) or {}
    if not net_names:
        return [nid for nid in nets if nid]
    out = []
    for nid, net in nets.items():
        if not nid:
            continue
        nm = getattr(net, 'name', '') or ''
        if any(nm == pat or _fn.fnmatch(nm, pat) for pat in net_names):
            out.append(nid)
    return out


def tightest_pin_gap(pcb: PCBData, net_ids) -> float:
    """Smallest edge-to-edge gap a trace of these nets has to leave through.

    For every pad of `net_ids`, the gap to the NEAREST OTHER PAD OF THE SAME
    FOOTPRINT -- i.e. the channel between the pins of the part it escapes
    from. That is the number the impedance width has to be compared against
    (#909): a 90 ohm leg is meaningless if it is three times the pin gap,
    because it necks down at both ends anyway. Returns 0.0 when it cannot be
    computed.
    """
    want = {int(n) for n in (net_ids or []) if n}
    if not want or not getattr(pcb, 'footprints', None):
        return 0.0
    best = 0.0
    for fp in pcb.footprints.values():
        pads = list(getattr(fp, 'pads', None) or [])
        if len(pads) < 2:
            continue
        mine = [p for p in pads if p.net_id in want]
        for a in mine:
            for b in pads:
                if b is a:
                    continue
                dx = abs(a.global_x - b.global_x) - (a.size_x + b.size_x) / 2
                dy = abs(a.global_y - b.global_y) - (a.size_y + b.size_y) / 2
                gap = max(dx, dy)
                if gap > 0 and (best == 0.0 or gap < best):
                    best = gap
    return best


def achievability_note(pcb: PCBData, layer_name: str, target_z0: float, *,
                       is_differential: bool = False, spacing: float = 0.0,
                       min_pitch_gap: float = 0.0,
                       board_thickness: float = NOMINAL_BOARD_THICKNESS_MM):
    """"Is `target_z0` reachable on a board like this?" -- with NUMBERS (#909).

    A board with no `(stackup ...)` gets one line today: "No stackup found in
    PCB file. Using fixed track width." True, and it tells the reader nothing
    about whether authoring a stackup would have helped. The repo's own
    solvers can answer that in one call against a NOMINAL stackup, and the
    answer is usually decisive: on a 2-layer 1.6 mm FR4 board a 90 ohm
    differential leg is 1.13 mm wide, against the 0.325 mm gap between the
    0.65 mm-pitch pins it has to leave from -- not achievable, and on a run
    that short not needed either.

    Returns `(text, detail_dict)`; `('', None)` when the board HAS a stackup
    (then the real solver runs and this has nothing to add) or when the
    solver cannot answer.
    """
    import copy as _copy
    if pcb is None or not target_z0:
        return '', None
    bi = getattr(pcb, 'board_info', None)
    if bi is None or getattr(bi, 'stackup', None):
        return '', None            # a real stackup: the real path runs
    n_cu = len(getattr(bi, 'copper_layers', None) or []) or 2
    probe = _copy.copy(pcb)
    probe_bi = _copy.copy(bi)
    probe_bi.stackup = nominal_stackup(n_cu, board_thickness)
    probe.board_info = probe_bi
    try:
        res = calculate_width_for_impedance(
            probe, layer_name, target_z0, spacing=spacing,
            is_differential=is_differential)
    except Exception:
        return '', None
    width = (res or {}).get('calculated_width_mm')
    if not width:
        return '', None
    kind = 'differential leg' if is_differential else 'trace'
    channel = (2 * width + spacing) if is_differential else width
    text = (f"No stackup declared, so no impedance pass runs. On a NOMINAL "
            f"{n_cu}-layer {board_thickness:g}mm FR4 stack (er "
            f"{NOMINAL_EPSILON_R:g}, {NOMINAL_COPPER_THICKNESS_MM * 1000:.0f}um "
            f"Cu) {target_z0:g} ohm on {layer_name} would need a "
            f"{width:.3f}mm {kind}"
            + (f" ({channel:.3f}mm for the pair)" if is_differential else ""))
    detail = {'assumed': 'nominal', 'copper_layers': n_cu,
              'board_thickness_mm': board_thickness,
              'epsilon_r': NOMINAL_EPSILON_R, 'target_ohms': float(target_z0),
              'layer': layer_name, 'width_mm': round(float(width), 4),
              'channel_mm': round(float(channel), 4),
              'differential': bool(is_differential)}
    if min_pitch_gap and min_pitch_gap > 0:
        ratio = channel / min_pitch_gap
        detail['pin_gap_mm'] = round(float(min_pitch_gap), 4)
        detail['channel_over_pin_gap'] = round(ratio, 2)
        text += (f", against a {min_pitch_gap:.3f}mm gap between the pins it "
                 f"leaves from -- {ratio:.1f}x. Not achievable here")
    text += ". Author a stackup (/recommend-stackup) if you need it graded."
    return text, detail


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

#!/usr/bin/env python3
"""
Post-route impedance / return-path analysis for a KiCad PCB (#486).

The pre-route impedance model resolves a static ``{layer: width}`` dict before
routing starts, so it structurally cannot see two things that are properties of
where a trace actually ENDS UP:

  A. **Side coupling.** A trace running through a ground pour on its own layer
     is a coplanar waveguide over ground, not a microstrip. The side ground
     pulls Z0 down, so the pre-route width comes out too wide.
  B. **Reference-plane continuity.** ``is_outer_layer`` is a pure stackup-index
     test; nothing verifies that the adjacent layer actually has copper UNDER
     the trace's path. A controlled-impedance trace crossing a gap in its
     reference plane gets the ideal-plane number anyway. That is the classic
     return-path discontinuity: it inflates Z0, wrecks the return current path
     and radiates. It is the more serious of the two, and it currently ships
     silent.

This module measures both on the ROUTED board and reports them. It is
diagnostic only -- it never modifies the board.

Usage:
    python3 check_impedance.py board.kicad_pcb
    python3 check_impedance.py board.kicad_pcb --nets "/DDR*" "/USB_*"
    python3 check_impedance.py board.kicad_pcb --json report.json
    python3 check_impedance.py board.kicad_pcb --verbose      # per-crossing list

Exit code is 1 when any reference-plane crossing is found (so it can gate a
build the way check_drc.py does), 0 otherwise. ``--exit-zero`` suppresses that.
"""

from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from impedance import (                                   # noqa: E402
    CPWG_GAP_RATIO_LIMIT,
    cpwg_applies,
    cpwg_z0,
    get_layer_impedance_params,
    is_asymmetric_stripline,
    microstrip_z0,
    stripline_z0,
    stripline_z0_asymmetric,
)

# Default distance between samples along a trace, in mm. The zone fill models
# rasterize at 0.04-0.08mm, so anything below ~0.1mm buys resolution the
# underlying occupancy grid does not have.
DEFAULT_SAMPLE_STEP = 0.15

# A void run shorter than this is quantization noise around a via antipad or a
# pad clearance opening, not a plane slot worth reporting.
DEFAULT_MIN_VOID_LENGTH = 0.30

# How far a measured coplanar gap may stray from the declared one before the
# declaration counts as broken. The pour rasterizes at 0.04-0.08mm and KiCad's
# filler rounds its own outline, so a few hundredths is measurement noise.
DEFAULT_GAP_TOLERANCE = 0.05


# =============================================================================
# Result structures
# =============================================================================

@dataclass
class RefCrossing:
    """One stretch of trace whose reference plane is missing or changes."""
    net_id: int
    net_name: str
    layer: str                    # layer the SIGNAL is on
    ref_layer: str                # layer that should be carrying the return
    kind: str                     # 'void' (no reference copper) | 'split'
    x: float                      # start of the run (mm)
    y: float
    x_end: float
    y_end: float
    length_mm: float

    def describe(self) -> str:
        if self.kind == 'void':
            what = f"no reference copper on {self.ref_layer}"
        else:
            what = f"reference plane SPLIT on {self.ref_layer}"
        return (f"{self.net_name} [{self.layer}] {what}: "
                f"{self.length_mm:.3f}mm from ({self.x:.3f}, {self.y:.3f}) "
                f"to ({self.x_end:.3f}, {self.y_end:.3f})")


@dataclass
class NetImpedance:
    """Per-net measured geometry and the Z0 it implies."""
    net_id: int
    net_name: str
    routed_length_mm: float = 0.0
    # --- reference-plane continuity (B) ---
    length_over_void_mm: float = 0.0
    segments_over_void: int = 0
    crossings: List[RefCrossing] = field(default_factory=list)
    # --- side gap / CPW (A) ---
    gap_samples: List[float] = field(default_factory=list)   # mm, coplanar gap
    tight_gap_length_mm: float = 0.0    # length where gap <= ratio_limit * h
    z0_nominal: float = 0.0             # what the pre-route model assumed
    z0_measured: float = 0.0            # length-weighted, side-gap aware
    z0_error_pct: float = 0.0
    # --- coplanar declaration audit (only when --coplanar-gap was given) ---
    declared_gap: float = 0.0           # what the route call promised
    outer_length_mm: float = 0.0        # length on outer layers (where CPW applies)
    length_no_ground_mm: float = 0.0    # outer length with NO pour beside it
    length_gap_wrong_mm: float = 0.0    # outer length whose gap misses the promise

    @property
    def void_fraction(self) -> float:
        if self.routed_length_mm <= 0:
            return 0.0
        return self.length_over_void_mm / self.routed_length_mm


@dataclass
class BoardImpedanceReport:
    board: str
    nets: List[NetImpedance] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    sample_step: float = DEFAULT_SAMPLE_STEP
    ratio_limit: float = CPWG_GAP_RATIO_LIMIT
    declared_gap: float = 0.0
    gap_tolerance: float = 0.0

    # --- board-level rollups (the acceptance-criteria numbers) ---
    @property
    def total_crossings(self) -> int:
        return sum(len(n.crossings) for n in self.nets)

    @property
    def segments_over_void(self) -> int:
        return sum(n.segments_over_void for n in self.nets)

    @property
    def length_over_void_mm(self) -> float:
        return sum(n.length_over_void_mm for n in self.nets)

    @property
    def routed_length_mm(self) -> float:
        return sum(n.routed_length_mm for n in self.nets)

    @property
    def all_gaps(self) -> List[float]:
        out: List[float] = []
        for n in self.nets:
            out.extend(n.gap_samples)
        return out


# =============================================================================
# Stackup: who references whom
# =============================================================================

def copper_layers_in_order(pcb) -> List[str]:
    """Copper layers top to bottom. Stackup order is authoritative; fall back
    to board_info.copper_layers on boards with no stackup section."""
    stackup = getattr(pcb.board_info, 'stackup', None) or []
    ordered = [l.name for l in stackup if l.layer_type == 'copper']
    if ordered:
        return ordered
    return list(getattr(pcb.board_info, 'copper_layers', None) or [])


def reference_layers_for(pcb) -> Dict[str, List[str]]:
    """Map each copper layer to the adjacent copper layer(s) that should carry
    its return current.

    Outer layers reference the one layer beneath them; inner layers sit between
    two and reference both. This mirrors get_layer_impedance_params' microstrip
    / stripline split -- deliberately, since the point of the check is to test
    the assumption that split is making.
    """
    order = copper_layers_in_order(pcb)
    refs: Dict[str, List[str]] = {}
    for i, name in enumerate(order):
        adjacent = []
        if i > 0:
            adjacent.append(order[i - 1])
        if i < len(order) - 1:
            adjacent.append(order[i + 1])
        refs[name] = adjacent
    return refs


# =============================================================================
# Copper occupancy on one layer, from the zone fill models
# =============================================================================

class LayerCopper:
    """Filled-pour occupancy for one copper layer.

    Backed by ZoneFillModel, which the fill-fidelity work made faithful to
    KiCad's actual filler (shape-true pad stamps, min-thickness opening,
    per-net-class clearances). Component ids matter as much as presence: a
    trace that stays over copper but moves from one fill COMPONENT to another
    has still crossed a slot, and its return current still has to detour.
    """

    def __init__(self, pcb, layer: str):
        self.layer = layer
        self.entries: List[Tuple[int, int, object]] = []   # (idx, net_id, model)
        # Rasterization pitch of the coarsest model on this layer. Occupancy is
        # tested at CELL CENTRES, so a distance probe first reports copper half
        # a cell late on average; callers de-bias with this.
        self.cell: float = 0.0
        zones = getattr(pcb, 'zones', None) or []
        try:
            from plane_fill_model import get_zone_model
        except Exception:
            return
        for idx, zone in enumerate(zones):
            if zone.layer != layer or not zone.polygon:
                continue
            if not zone.net_id:
                continue        # unassigned pour is not a reference
            try:
                model = get_zone_model(pcb, zone)
            except Exception:
                model = None
            if model is not None:
                self.entries.append((idx, zone.net_id, model))
                self.cell = max(self.cell, getattr(model, 'cell', 0.0) or 0.0)

    def __bool__(self) -> bool:
        return bool(self.entries)

    def key_at(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """(zone index, fill component) of the pour covering (x, y), else None.

        Deterministic: zones are probed in file order and the first hit wins,
        so two runs over the same board classify a point identically.
        """
        for idx, _net_id, model in self.entries:
            comp = model.query_component(x, y)
            if comp:
                return (idx, comp)
        return None

    def foreign_key_at(self, x: float, y: float,
                       exclude_net: int) -> Optional[Tuple[int, int]]:
        """Same as key_at but ignoring pours belonging to `exclude_net` -- used
        for the coplanar-ground probe, where the trace's own net's pour is not
        a coplanar ground, it is the trace."""
        for idx, net_id, model in self.entries:
            if net_id == exclude_net:
                continue
            if model.query_component(x, y):
                return (idx, net_id)
        return None


# =============================================================================
# Measurement
# =============================================================================

def _sample_points(x1: float, y1: float, x2: float, y2: float,
                   step: float) -> List[Tuple[float, float, float]]:
    """Points along a segment as (x, y, arc-length-from-start)."""
    length = math.hypot(x2 - x1, y2 - y1)
    if length <= 0:
        return [(x1, y1, 0.0)]
    n = max(1, int(math.ceil(length / step)))
    out = []
    for i in range(n + 1):
        t = i / n
        out.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t, t * length))
    return out


def measure_side_gap(copper: LayerCopper, x: float, y: float,
                     dx: float, dy: float, half_width: float,
                     max_probe: float, exclude_net: int,
                     probe_step: float = 0.05) -> Optional[float]:
    """Edge-to-edge distance from the trace to same-layer foreign pour copper.

    Marches perpendicular to the trace on BOTH sides and returns the smaller
    gap (the tighter side dominates the coupling). None means no coplanar
    ground within `max_probe` -- i.e. microstrip governs and there is no CPW
    correction to make.
    """
    if not copper:
        return None
    # Unit normal to the trace direction.
    norm = math.hypot(dx, dy)
    if norm <= 0:
        return None
    nx, ny = -dy / norm, dx / norm

    best: Optional[float] = None
    steps = int(max_probe / probe_step) + 1
    for sign in (1.0, -1.0):
        d = half_width
        for _ in range(steps):
            d += probe_step
            if d - half_width > max_probe:
                break
            if copper.foreign_key_at(x + nx * sign * d, y + ny * sign * d,
                                     exclude_net):
                gap = d - half_width
                if best is None or gap < best:
                    best = gap
                break
    if best is None:
        return None
    # De-bias the rasterization: the pour is detected at the first CELL CENTRE
    # that carries fill, which sits on average half a cell beyond the true
    # copper edge (plus half a probe step of search granularity). Without this
    # every measured gap reads ~one cell wide -- a 0.20mm declared gap
    # measured 0.25mm and the audit called a correct pour off-target.
    bias = copper.cell / 2.0 + probe_step / 2.0
    return max(0.0, best - bias)


def _nominal_z0(pcb, layer: str, width: float, coplanar_gap: float = 0.0) -> float:
    """Z0 the ROUTE CALL assumed for this layer/width.

    With no coplanar declaration that is the ideal-continuous-plane microstrip
    or stripline answer. With one, it is the CPWG answer at the DECLARED gap --
    the thing the routed width was actually derived from, and therefore the
    right baseline to score the measured geometry against.
    """
    params = get_layer_impedance_params(pcb, layer)
    if params is None or width <= 0:
        return 0.0
    if params.is_outer_layer:
        if coplanar_gap > 0 and params.dielectric_height > 0:
            return cpwg_z0(width, coplanar_gap, params.dielectric_height,
                           params.copper_thickness, params.dielectric_constant)
        return microstrip_z0(width, params.dielectric_height,
                             params.copper_thickness, params.dielectric_constant)
    if is_asymmetric_stripline(params):
        return stripline_z0_asymmetric(width, params.height_above,
                                       params.height_below,
                                       params.copper_thickness,
                                       params.dielectric_constant)
    return stripline_z0(width, params.dielectric_height,
                        params.copper_thickness, params.dielectric_constant)


def analyze_impedance(pcb, net_patterns: Optional[List[str]] = None,
                      sample_step: float = DEFAULT_SAMPLE_STEP,
                      min_void_length: float = DEFAULT_MIN_VOID_LENGTH,
                      ratio_limit: float = CPWG_GAP_RATIO_LIMIT,
                      include_plane_nets: bool = False,
                      coplanar_gap: float = 0.0,
                      gap_tolerance: float = DEFAULT_GAP_TOLERANCE,
                      board_name: str = "",
                      net_declared_gaps: Optional[Dict[int, float]] = None) -> BoardImpedanceReport:
    """Walk every routed segment and measure what the pre-route model assumed.

    Args:
        pcb: Parsed PCBData (post-route, pours filled).
        net_patterns: fnmatch patterns limiting which nets are analyzed.
            None/empty analyzes every routed signal net.
        sample_step: Distance between samples along a trace, mm.
        min_void_length: Ignore void runs shorter than this (via antipads).
        ratio_limit: Side gap <= ratio_limit * h counts as coplanar-coupled.
        include_plane_nets: Also analyze nets that own a pour (off by default;
            a plane net's own traces are not controlled-impedance signals).
        coplanar_gap: The gap that was DECLARED at route time (route.py
            --coplanar-gap). > 0 turns this into an audit of that promise:
            every outer-layer sample is checked for a pour at that distance,
            and the shortfall is reported per net.
        gap_tolerance: How far the measured gap may stray from the declared
            one before it counts as broken, in mm.
        board_name: Label for the report.
        net_declared_gaps: Per-net declared coplanar gaps (net_id -> mm),
            auto-read from the sibling .kicad_pro's net_impedance records
            (#521). A net with an entry audits at ITS declared gap (0 = that
            net was declared non-coplanar); nets without one fall back to
            ``coplanar_gap``.

    Returns:
        BoardImpedanceReport. Diagnostic only; pcb is not modified.
    """
    report = BoardImpedanceReport(board=board_name, sample_step=sample_step,
                                  ratio_limit=ratio_limit,
                                  declared_gap=coplanar_gap,
                                  gap_tolerance=gap_tolerance)

    refs_by_layer = reference_layers_for(pcb)
    if not refs_by_layer:
        report.notes.append("Board has no copper layer list; nothing to check.")
        return report

    stackup = getattr(pcb.board_info, 'stackup', None) or []
    if not stackup:
        report.notes.append(
            "Board has NO STACKUP section: dielectric heights and Er are "
            "unknown, so Z0 numbers are omitted. Reference-plane continuity "
            "is still checked (it only needs the layer order).")

    # Which nets?
    from net_queries import matches_net_filter
    segments_by_net: Dict[int, List] = {}
    for seg in pcb.segments:
        if not seg.net_id:
            continue
        segments_by_net.setdefault(seg.net_id, []).append(seg)

    # Nets that own a pour are planes, not controlled-impedance signals: a GND
    # or VCC trace does not have (or need) a reference plane under it, and
    # including them buries the real signal offenders under plane-net noise.
    # (#486 open question "which nets count as controlled-impedance": with no
    # per-net-class impedance notion on the board, the honest default is every
    # routed SIGNAL net, narrowed with --nets.)
    pour_nets = {z.net_id for z in (getattr(pcb, 'zones', None) or [])
                 if z.net_id and z.polygon}

    selected = []
    skipped_plane_nets = 0
    for net_id in sorted(segments_by_net):
        net = pcb.nets.get(net_id)
        name = net.name if net else f"<net {net_id}>"
        if net_patterns and not matches_net_filter(name, net_patterns):
            continue
        if net_id in pour_nets and not include_plane_nets:
            skipped_plane_nets += 1
            continue
        selected.append((net_id, name))

    if skipped_plane_nets:
        report.notes.append(
            f"Skipped {skipped_plane_nets} plane net(s) that own a pour "
            f"(not controlled-impedance signals). Use include_plane_nets/"
            f"--include-plane-nets to analyze them anyway.")

    if not selected:
        report.notes.append("No routed nets matched the filter.")
        return report

    # Lazily-built copper occupancy, one model set per layer (expensive to
    # rasterize, so shared across every net that references it).
    copper_cache: Dict[str, LayerCopper] = {}

    def copper_for(layer: str) -> LayerCopper:
        got = copper_cache.get(layer)
        if got is None:
            got = LayerCopper(pcb, layer)
            copper_cache[layer] = got
        return got

    # Board-level sanity: a reference layer with no pour at all is not a
    # per-segment defect, it is a stackup/flow problem. Say it once.
    for layer in sorted(set(refs_by_layer)):
        for ref in refs_by_layer[layer]:
            if not copper_for(ref):
                note = (f"Reference layer {ref} carries NO filled pour: every "
                        f"trace on {layer} referencing it is unreferenced.")
                if note not in report.notes:
                    report.notes.append(note)

    for net_id, net_name in selected:
        # #521: a net's own recorded declaration outranks the call-level gap.
        net_gap = (net_declared_gaps or {}).get(net_id, coplanar_gap)
        result = NetImpedance(net_id=net_id, net_name=net_name,
                              declared_gap=net_gap)
        z0_weighted = 0.0
        z0_nom_weighted = 0.0
        weight = 0.0

        for seg in segments_by_net[net_id]:
            layer = seg.layer
            seg_len = math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
            if seg_len <= 0:
                continue
            result.routed_length_mm += seg_len

            params = get_layer_impedance_params(pcb, layer)
            h = params.dielectric_height if params else 0.0
            ref_layers = refs_by_layer.get(layer, [])

            samples = _sample_points(seg.start_x, seg.start_y,
                                     seg.end_x, seg.end_y, sample_step)
            # _sample_points returns n+1 points spanning n intervals. Each
            # sample stands for ONE interval, so only the first n carry weight
            # -- weighting all n+1 made every accumulated length overshoot by
            # (n+1)/n and produced "128% of net" rows.
            n_int = max(1, len(samples) - 1)
            per_sample = seg_len / n_int

            # ---- B: reference-plane continuity -------------------------
            seg_flagged = False
            # A sample is "unreferenced" if ANY of its required reference
            # layers has no copper under it (a stripline needs both). Tracked
            # as a union across ref layers so the reported length can never
            # exceed the trace's own length -- summing per ref layer
            # double-counted inner-layer traces and produced >100% figures.
            void_any = [False] * len(samples)
            for ref_layer in ref_layers:
                ref_copper = copper_for(ref_layer)
                keys = [ref_copper.key_at(x, y) for x, y, _ in samples]

                run_start: Optional[int] = None
                for i, key in enumerate(keys + [object()]):   # sentinel closes
                    is_void = (i < len(keys)) and (key is None)
                    if is_void and run_start is None:
                        run_start = i
                    elif not is_void and run_start is not None:
                        run_len = (i - run_start) * per_sample
                        if run_len >= min_void_length:
                            x0, y0, _ = samples[run_start]
                            x1, y1, _ = samples[min(i, len(samples) - 1)]
                            result.crossings.append(RefCrossing(
                                net_id=net_id, net_name=net_name, layer=layer,
                                ref_layer=ref_layer, kind='void',
                                x=x0, y=y0, x_end=x1, y_end=y1,
                                length_mm=run_len))
                            for j in range(run_start, min(i, len(samples))):
                                void_any[j] = True
                            seg_flagged = True
                        run_start = None

                # Reference SPLIT: still over copper, but a different fill
                # component -- the return current cannot follow the trace.
                # Only ADJACENT samples count: a change across an intervening
                # void is already reported as that void, and counting it twice
                # would double-charge one defect.
                for i in range(1, len(keys)):
                    prev, key = keys[i - 1], keys[i]
                    if prev is not None and key is not None and key != prev:
                        x0, y0, _ = samples[i]
                        result.crossings.append(RefCrossing(
                            net_id=net_id, net_name=net_name, layer=layer,
                            ref_layer=ref_layer, kind='split',
                            x=x0, y=y0, x_end=x0, y_end=y0,
                            length_mm=0.0))
                        seg_flagged = True

            result.length_over_void_mm += sum(void_any[:n_int]) * per_sample
            if seg_flagged:
                result.segments_over_void += 1

            # ---- A: coplanar side gap + implied Z0 ---------------------
            if params is None or h <= 0 or not params.is_outer_layer:
                # CPWG models one plane BELOW the trace, so it is an
                # outer-layer model. Inner-layer side coupling is real but
                # needs a coupled-stripline map we do not have; excluded
                # rather than guessed at.
                continue

            result.outer_length_mm += seg_len
            same_layer = copper_for(layer)

            # The declared gap is the design intent; the trace was ROUTED at a
            # width derived from it, so that width is what the nominal Z0 must
            # be scored at either way.
            z0_nom = _nominal_z0(pcb, layer, seg.width, coplanar_gap=net_gap)

            if not same_layer:
                # No pour at all on this layer. If the route call declared a
                # coplanar gap, that promise is broken over this whole segment.
                if net_gap > 0:
                    result.length_no_ground_mm += seg_len
                continue

            # Probe far enough to tell "gap is wider than promised" from "no
            # ground at all", and far enough to characterize the distribution
            # either side of the ratio_limit * h threshold.
            max_probe = max(ratio_limit * h * 1.5, 0.5)
            if net_gap > 0:
                max_probe = max(max_probe, net_gap * 3.0)
            dx, dy = seg.end_x - seg.start_x, seg.end_y - seg.start_y
            half_w = (seg.width or 0.0) / 2.0

            for (sx, sy, _) in samples[:n_int]:
                gap = measure_side_gap(same_layer, sx, sy, dx, dy, half_w,
                                       max_probe, net_id)
                if gap is None:
                    z0_here = z0_nom
                    if net_gap > 0:
                        result.length_no_ground_mm += per_sample
                else:
                    result.gap_samples.append(gap)
                    if net_gap > 0 and \
                            abs(gap - net_gap) > gap_tolerance:
                        result.length_gap_wrong_mm += per_sample
                    if cpwg_applies(gap, h, ratio_limit):
                        result.tight_gap_length_mm += per_sample
                        z0_here = cpwg_z0(seg.width, gap, h,
                                          params.copper_thickness,
                                          params.dielectric_constant)
                    else:
                        z0_here = z0_nom
                if z0_here > 0 and z0_nom > 0:
                    z0_weighted += z0_here * per_sample
                    z0_nom_weighted += z0_nom * per_sample
                    weight += per_sample

        if weight > 0:
            result.z0_measured = z0_weighted / weight
            result.z0_nominal = z0_nom_weighted / weight
            if result.z0_nominal > 0:
                result.z0_error_pct = 100.0 * (
                    result.z0_measured - result.z0_nominal) / result.z0_nominal

        if result.routed_length_mm > 0:
            report.nets.append(result)

    return report


# =============================================================================
# Reporting
# =============================================================================

def _percentiles(values: List[float], pcts=(5, 25, 50, 75, 95)) -> Dict[str, float]:
    if not values:
        return {}
    ordered = sorted(values)
    out = {}
    for p in pcts:
        idx = min(len(ordered) - 1, max(0, int(round((p / 100.0) * (len(ordered) - 1)))))
        out[f"p{p}"] = ordered[idx]
    return out


def format_report(report: BoardImpedanceReport, verbose: bool = False,
                  top: int = 20) -> str:
    lines: List[str] = []
    lines.append("=" * 78)
    lines.append(f"Impedance / return-path analysis: {report.board}")
    lines.append("=" * 78)

    for note in report.notes:
        lines.append(f"  NOTE: {note}")
    if report.notes:
        lines.append("")

    if not report.nets:
        lines.append("  No routed nets analyzed.")
        return "\n".join(lines)

    offenders = [n for n in report.nets if n.crossings]
    lines.append(f"Nets analyzed:            {len(report.nets)}")
    lines.append(f"Routed length:            {report.routed_length_mm:.1f} mm")
    lines.append("")
    lines.append("-- Reference-plane continuity (return path) --")
    lines.append(f"  Nets with a crossing:   {len(offenders)}")
    lines.append(f"  Segments over a void:   {report.segments_over_void}")
    lines.append(f"  Total crossings:        {report.total_crossings}")
    lines.append(f"  Length over void:       {report.length_over_void_mm:.3f} mm")

    if offenders:
        lines.append("")
        lines.append(f"  {'net':<28} {'over void mm':>12} {'% of net':>9} {'crossings':>10}")
        ranked = sorted(offenders,
                        key=lambda n: (-n.length_over_void_mm, n.net_name))
        for n in ranked[:top]:
            lines.append(f"  {n.net_name[:28]:<28} {n.length_over_void_mm:>12.3f} "
                         f"{100 * n.void_fraction:>8.1f}% {len(n.crossings):>10}")
        if len(ranked) > top:
            lines.append(f"  ... and {len(ranked) - top} more nets")

    if verbose:
        lines.append("")
        lines.append("  Crossings:")
        for n in sorted(offenders, key=lambda n: n.net_name):
            for c in n.crossings:
                lines.append(f"    {c.describe()}")

    # --- coplanar declaration audit ---
    if report.declared_gap > 0:
        lines.append("")
        lines.append(f"-- Coplanar declaration audit (--coplanar-gap "
                     f"{report.declared_gap:g}mm, tolerance "
                     f"±{report.gap_tolerance:g}mm) --")
        outer = sum(n.outer_length_mm for n in report.nets)
        no_gnd = sum(n.length_no_ground_mm for n in report.nets)
        wrong = sum(n.length_gap_wrong_mm for n in report.nets)
        ok = max(0.0, outer - no_gnd - wrong)
        lines.append(f"  Outer-layer length declared coplanar: {outer:.1f} mm")
        if outer > 0:
            lines.append(f"    gap as declared:  {ok:>9.1f} mm  "
                         f"({100 * ok / outer:.1f}%)")
            lines.append(f"    gap off-target:   {wrong:>9.1f} mm  "
                         f"({100 * wrong / outer:.1f}%)")
            lines.append(f"    NO ground beside: {no_gnd:>9.1f} mm  "
                         f"({100 * no_gnd / outer:.1f}%)")
        broken = [n for n in report.nets
                  if (n.length_no_ground_mm + n.length_gap_wrong_mm) > 0.01]
        if broken:
            lines.append("")
            lines.append("  Nets whose declared gap was NOT achieved "
                         "(their routed width assumes it was):")
            lines.append(f"  {'net':<28} {'no gnd mm':>10} {'off-target mm':>14} {'% bad':>7}")
            ranked = sorted(broken,
                            key=lambda n: -(n.length_no_ground_mm
                                            + n.length_gap_wrong_mm))
            for n in ranked[:top]:
                bad = n.length_no_ground_mm + n.length_gap_wrong_mm
                frac = 100 * bad / n.outer_length_mm if n.outer_length_mm else 0.0
                lines.append(f"  {n.net_name[:28]:<28} {n.length_no_ground_mm:>10.2f} "
                             f"{n.length_gap_wrong_mm:>14.2f} {frac:>6.1f}%")
            if len(ranked) > top:
                lines.append(f"  ... and {len(ranked) - top} more nets")
        else:
            lines.append("  All declared-coplanar copper achieved its gap.")

    # --- side gap / CPW ---
    gaps = report.all_gaps
    lines.append("")
    lines.append(f"-- Coplanar side gap (CPW-over-ground, threshold {report.ratio_limit:g}x h) --")
    if not gaps:
        lines.append("  No same-layer foreign pour found beside any trace: "
                     "microstrip governs everywhere, no CPW correction needed.")
    else:
        tight = sum(n.tight_gap_length_mm for n in report.nets)
        pct = _percentiles(gaps)
        lines.append(f"  Samples with coplanar ground nearby: {len(gaps)}")
        lines.append(f"  Length coplanar-coupled (<= {report.ratio_limit:g}x h): {tight:.1f} mm")
        lines.append("  Gap distribution (mm): " +
                     "  ".join(f"{k}={v:.3f}" for k, v in pct.items()))
        shifted = [n for n in report.nets if abs(n.z0_error_pct) >= 1.0]
        if shifted:
            lines.append("")
            lines.append(f"  {'net':<28} {'assumed Z0':>11} {'measured':>10} {'error':>8}")
            for n in sorted(shifted, key=lambda n: n.z0_error_pct)[:top]:
                lines.append(f"  {n.net_name[:28]:<28} {n.z0_nominal:>10.1f}Ω "
                             f"{n.z0_measured:>9.1f}Ω {n.z0_error_pct:>+7.1f}%")
            if len(shifted) > top:
                lines.append(f"  ... and {len(shifted) - top} more nets")
        else:
            lines.append("  No net's implied Z0 shifts by >=1% from the "
                         "microstrip assumption.")

    lines.append("=" * 78)
    return "\n".join(lines)


def report_to_dict(report: BoardImpedanceReport) -> dict:
    """JSON-serializable form, for corpus aggregation."""
    gaps = report.all_gaps
    return {
        'board': report.board,
        'sample_step_mm': report.sample_step,
        'ratio_limit': report.ratio_limit,
        'notes': report.notes,
        'totals': {
            'nets_analyzed': len(report.nets),
            'routed_length_mm': round(report.routed_length_mm, 4),
            'nets_with_crossing': sum(1 for n in report.nets if n.crossings),
            'segments_over_void': report.segments_over_void,
            'crossings': report.total_crossings,
            'length_over_void_mm': round(report.length_over_void_mm, 4),
            'gap_samples': len(gaps),
            'gap_percentiles_mm': {k: round(v, 4)
                                   for k, v in _percentiles(gaps).items()},
            'tight_gap_length_mm': round(
                sum(n.tight_gap_length_mm for n in report.nets), 4),
            'declared_gap_mm': report.declared_gap,
            'outer_length_mm': round(
                sum(n.outer_length_mm for n in report.nets), 4),
            'length_no_ground_mm': round(
                sum(n.length_no_ground_mm for n in report.nets), 4),
            'length_gap_wrong_mm': round(
                sum(n.length_gap_wrong_mm for n in report.nets), 4),
        },
        'nets': [
            {
                'net_id': n.net_id,
                'net_name': n.net_name,
                'routed_length_mm': round(n.routed_length_mm, 4),
                'length_over_void_mm': round(n.length_over_void_mm, 4),
                'segments_over_void': n.segments_over_void,
                'crossings': len(n.crossings),
                'void_fraction': round(n.void_fraction, 5),
                'tight_gap_length_mm': round(n.tight_gap_length_mm, 4),
                'outer_length_mm': round(n.outer_length_mm, 4),
                'length_no_ground_mm': round(n.length_no_ground_mm, 4),
                'length_gap_wrong_mm': round(n.length_gap_wrong_mm, 4),
                'z0_nominal': round(n.z0_nominal, 3),
                'z0_measured': round(n.z0_measured, 3),
                'z0_error_pct': round(n.z0_error_pct, 3),
                'crossing_detail': [
                    {'kind': c.kind, 'layer': c.layer, 'ref_layer': c.ref_layer,
                     'x': round(c.x, 4), 'y': round(c.y, 4),
                     'length_mm': round(c.length_mm, 4)}
                    for c in n.crossings
                ],
            }
            for n in sorted(report.nets,
                            key=lambda n: (-n.length_over_void_mm, n.net_name))
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Post-route impedance / return-path analysis (#486)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('input', help='Routed .kicad_pcb file')
    parser.add_argument('--nets', nargs='+', metavar='PATTERN',
                        help='Only analyze these nets (fnmatch patterns). '
                             'Default: every routed net.')
    parser.add_argument('--sample-step', type=float, default=DEFAULT_SAMPLE_STEP,
                        help=f'Sample spacing along traces in mm '
                             f'(default {DEFAULT_SAMPLE_STEP})')
    parser.add_argument('--min-void-length', type=float,
                        default=DEFAULT_MIN_VOID_LENGTH,
                        help=f'Ignore void runs shorter than this in mm '
                             f'(default {DEFAULT_MIN_VOID_LENGTH}; filters via '
                             f'antipads)')
    parser.add_argument('--gap-ratio-limit', type=float,
                        default=CPWG_GAP_RATIO_LIMIT,
                        help=f'Side gap <= this multiple of the dielectric '
                             f'height counts as coplanar-coupled '
                             f'(default {CPWG_GAP_RATIO_LIMIT})')
    parser.add_argument('--coplanar-gap', type=float, default=0.0,
                        help='Audit a coplanar declaration: the --coplanar-gap '
                             'that route.py was run with. Every outer-layer '
                             'sample is checked for same-layer pour at that '
                             'distance, and the shortfall is reported per net. '
                             'Nets with a recorded declaration in the sibling '
                             '.kicad_pro (#521, written by --impedance steps) '
                             'audit at THEIR OWN gap automatically; this flag '
                             'is the fallback for nets without one.')
    parser.add_argument('--gap-tolerance', type=float, default=DEFAULT_GAP_TOLERANCE,
                        help=f'How far a measured gap may stray from the '
                             f'declared one before it counts as broken, in mm '
                             f'(default {DEFAULT_GAP_TOLERANCE})')
    parser.add_argument('--include-plane-nets', action='store_true',
                        help='Also analyze nets that own a pour. Off by '
                             'default: a GND/VCC trace is not a controlled-'
                             'impedance signal and swamps the real offenders.')
    parser.add_argument('--json', metavar='PATH',
                        help='Write the full report as JSON')
    parser.add_argument('--verbose', action='store_true',
                        help='List every individual crossing')
    parser.add_argument('--top', type=int, default=20,
                        help='Rows per table (default 20)')
    parser.add_argument('--exit-zero', action='store_true',
                        help='Always exit 0, even when crossings are found')
    args = parser.parse_args()

    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(args.input)

    # #521: route steps run with --impedance record each net's declaration
    # (ohms + coplanar gap) in the sibling .kicad_pro. Auto-read them so the
    # audit grades every net against ITS OWN recorded promise -- a stored
    # entry (even gap 0 = declared non-coplanar) outranks --coplanar-gap,
    # which remains the fallback for nets/boards without records.
    net_declared_gaps = {}
    try:
        from protected_nets import read_impedance_specs, pro_path_for_board
        _specs = read_impedance_specs(pro_path_for_board(args.input))
        if _specs:
            # #906: a declaration recorded `applied: false` is a statement of
            # INTENT whose width was never solved -- the router fell back to
            # the plain track width, so the copper is not impedance-controlled
            # and grading it against the declared ohms reports a promise the
            # run never made. The record is kept so a later step with a usable
            # stackup can recompute it; this audit skips it. A record written
            # before #906 has no `applied` key at all and is treated as applied,
            # which is what it meant then.
            _unapplied = sorted(nm for nm, sp in _specs.items()
                                if sp.get('applied') is False)
            _specs = {nm: sp for nm, sp in _specs.items()
                      if sp.get('applied') is not False}
            _n2i = {n.name: nid for nid, n in pcb.nets.items() if n.name}
            net_declared_gaps = {
                _n2i[nm]: float(sp.get('coplanar_gap', 0) or 0)
                for nm, sp in _specs.items() if nm in _n2i}
            _cop = sum(1 for g in net_declared_gaps.values() if g > 0)
            print(f"Auto-read {len(net_declared_gaps)} net impedance "
                  f"declaration(s) from the sibling .kicad_pro "
                  f"({_cop} coplanar) -- per-net gaps outrank --coplanar-gap")
            if _unapplied:
                print(f"  {len(_unapplied)} declaration(s) recorded as NOT "
                      f"APPLIED are skipped -- no width was solved for them, so "
                      f"their copper is not impedance-controlled: "
                      f"{', '.join(_unapplied[:6])}"
                      f"{' ...' if len(_unapplied) > 6 else ''}")
    except Exception:
        pass

    report = analyze_impedance(
        pcb,
        net_patterns=args.nets,
        sample_step=args.sample_step,
        min_void_length=args.min_void_length,
        ratio_limit=args.gap_ratio_limit,
        include_plane_nets=args.include_plane_nets,
        coplanar_gap=args.coplanar_gap,
        gap_tolerance=args.gap_tolerance,
        board_name=os.path.basename(args.input),
        net_declared_gaps=net_declared_gaps)

    print(format_report(report, verbose=args.verbose, top=args.top))

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(report_to_dict(report), fh, indent=2)
        print(f"\nJSON written to {args.json}")

    if args.exit_zero:
        return 0
    broken_promise = any((n.length_no_ground_mm + n.length_gap_wrong_mm) > 0.01
                         for n in report.nets)
    return 1 if (report.total_crossings or broken_promise) else 0


if __name__ == '__main__':
    sys.exit(main())

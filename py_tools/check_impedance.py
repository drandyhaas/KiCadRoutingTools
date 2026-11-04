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

Exit code is 1 when any reference-plane crossing is found, or when a net with
a declared coplanar gap (``--coplanar-gap``, or its own declaration in the
sibling .kicad_pro) misses it -- no ground beside it, or a gap off the
declared one, over more than 0.01 mm of its outer-layer length; the coplanar
audit section names those nets. Samples over the net's own pad, or within the
pour's clearance of it, are reported apart and never counted (#1582). Exit 0
otherwise, so it can gate a build the way check_drc.py does. ``--exit-zero``
suppresses that.

The coplanar gap is read on KiCad's own fill at exact distance (#1624): the
(filled_polygon ...) KiCad stored in the board, else -- when a declared gap
is audited -- a KiCad refill, else the fill model at the project's net-class
clearances. The report's ``gap_source`` names which.
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

# A void run shorter than this is quantization noise around a pad clearance
# opening, not a plane slot worth reporting. The net's OWN via antipads are
# exempted by geometry instead (#1203): a segment ending on its via runs over
# ~via radius + clearance of antipad, which no length filter can separate.
DEFAULT_MIN_VOID_LENGTH = 0.30

# How far a measured coplanar gap may stray from the declared one before the
# declaration counts as broken. The fill model rasterizes at 0.04-0.08mm and KiCad's
# filler rounds its own outline, so a few hundredths is measurement noise.
DEFAULT_GAP_TOLERANCE = 0.05

# How far a declared impedance may sit off the model's reading of the shipped
# copper before the audit names the net (#1630), in percent: a fab's usual
# controlled-impedance tolerance. Disclosure only -- the exit code ignores it.
DECLARED_OHMS_TOLERANCE_PCT = 10.0

# A diff pair's P and N count as coupled where their copper is within this
# multiple of the declared pair gap: a breakout or a fan-out apart from its
# partner is not a coupled line, and the formula does not apply there.
COUPLED_GAP_FACTOR = 2.0

# A net's coplanar promise is broken when more than this length of it has no
# ground beside it or misses the declared gap (mm). board_score reads the same
# predicate off the JSON rows (#1389).
BROKEN_PROMISE_MM = 0.01


def promise_broken(length_no_ground_mm: float, length_gap_wrong_mm: float) -> bool:
    """The one rule for a broken coplanar promise (#1389, #1582)."""
    return ((length_no_ground_mm or 0.0) + (length_gap_wrong_mm or 0.0)
            > BROKEN_PROMISE_MM)


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
    # Void runs inside the antipad of the net's OWN via, or its P/N
    # partner's (#1203): a layer change, not a plane slot. Counted here,
    # never as a crossing.
    own_via_antipad_runs: int = 0
    length_over_own_antipad_mm: float = 0.0
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
    # Outer length whose gap misses the promise only because the net's OWN
    # pad is there (#1582): over a pad wider than the track the conductor is
    # the pad, and just past it the pour still keeps its clearance from the
    # pad. Like the own-via antipad (#1203), reported here and never counted
    # in the two above.
    length_at_own_pad_mm: float = 0.0
    # --- declared impedance (#1630, disclosure): the ohms a route step was
    # asked for (its record in the sibling .kicad_pro), against the model's
    # reading of the shipped width -- and, for a pair, the shipped P<->N gap.
    declared_ohms: float = 0.0
    declared_differential: bool = False
    shipped_ohms: float = 0.0           # length-weighted over `ohms_length_mm`
    ohms_length_mm: float = 0.0         # length the reading covers (a pair: coupled)

    @property
    def ohms_off_pct(self) -> Optional[float]:
        """How far the shipped impedance is off the declared one, %; None
        when there is no declaration or no length to read it on."""
        if self.declared_ohms <= 0 or self.ohms_length_mm <= 0:
            return None
        return 100.0 * (self.shipped_ohms - self.declared_ohms) / self.declared_ohms

    @property
    def promise_broken(self) -> bool:
        """Did any of this net's declared-coplanar length miss its gap?"""
        return promise_broken(self.length_no_ground_mm, self.length_gap_wrong_mm)

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
    # #1624: whose fill the coplanar gaps were read on -- 'stored' (the
    # (filled_polygon ...) KiCad wrote into the board), 'refill' (a KiCad
    # refill of it), 'model' (the fill model alone) or '' (no gap measured).
    gap_source: str = ''

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

def _overlapping_components(ma, mb) -> List[Tuple[int, int]]:
    """(component of `ma`, component of `mb`) for every pair of fill
    components of two pour models that share copper. Sampled at the cell
    centres of the finer of the two grids, over their common extent -- the
    same cell-centre reading `ZoneFillModel.query_component` gives a probe."""
    import numpy as np
    la_all = getattr(ma, 'labels', None)
    lb_all = getattr(mb, 'labels', None)
    if la_all is None or lb_all is None:
        return []
    swap = mb.cell < ma.cell
    f, c = (mb, ma) if swap else (ma, mb)          # f: the finer grid
    lo_x, hi_x = max(f.x0, c.x0), min(f.x0 + f.nx * f.cell, c.x0 + c.nx * c.cell)
    lo_y, hi_y = max(f.y0, c.y0), min(f.y0 + f.ny * f.cell, c.y0 + c.ny * c.cell)
    if hi_x <= lo_x or hi_y <= lo_y:
        return []
    fi = np.arange(max(0, int((lo_x - f.x0) / f.cell)),
                   min(f.nx, int(math.ceil((hi_x - f.x0) / f.cell))))
    fj = np.arange(max(0, int((lo_y - f.y0) / f.cell)),
                   min(f.ny, int(math.ceil((hi_y - f.y0) / f.cell))))
    ci = np.floor((f.x0 + (fi + 0.5) * f.cell - c.x0) / c.cell).astype(np.int64)
    cj = np.floor((f.y0 + (fj + 0.5) * f.cell - c.y0) / c.cell).astype(np.int64)
    okx = (ci >= 0) & (ci < c.nx)
    oky = (cj >= 0) & (cj < c.ny)
    fi, ci, fj, cj = fi[okx], ci[okx], fj[oky], cj[oky]
    if not len(fi) or not len(fj):
        return []
    lf = f.labels[np.ix_(fi, fj)].astype(np.int64)
    lc = c.labels[np.ix_(ci, cj)].astype(np.int64)
    both = (lf > 0) & (lc > 0)
    if not both.any():
        return []
    base = int(lc.max()) + 1
    pairs = np.unique(lf[both] * base + lc[both])
    out = [(int(p // base), int(p % base)) for p in pairs]
    return [(b, a) for a, b in out] if swap else out


class ExactFill:
    """KiCad's own fill of one layer's pours, for the coplanar gap (#1624).

    Built from (filled_polygon ...) islands -- the ones KiCad stored in the
    board, or a KiCad refill's -- keyed by net. The gap is the EXACT distance
    from a point of the trace's centreline to the nearest foreign fill, less
    the half-width: a reading of KiCad's copper, not of a grid. (The fill
    model's reading moves by up to a cell with where its grid falls under the
    track, and a correct 0.20 mm gap read 0.15.) Nearest distance rather than
    a perpendicular probe: a pour cut back by a clearance is that far from
    EVERY point of the track, so the nearest fill is the clearance itself,
    also at a bend or beside the net's own via, where a perpendicular can
    pass through the copper's rounded clearance and read wide.

    A point inside the fill, or fill closer than the half-width, is fill over
    the trace: KiCad never fills there, so the stored fill predates the
    copper (or is a short). It reads 0 -- a broken promise -- and is counted
    in `overlaps`, so the report can say to refill.
    """

    def __init__(self, polys_by_net: Dict[int, List]):
        import numpy as np
        import shapely
        from shapely.geometry import Polygon
        self.nets: Dict[int, Tuple[object, object]] = {}
        self.overlaps: set = set()
        for nid, polys in polys_by_net.items():
            # buffer(0) turns KiCad's fractured outline (a hole reached by a
            # zero-width slit) into a polygon with a true hole.
            parts = [Polygon(p).buffer(0) for p in polys if len(p) >= 3]
            geom = shapely.unary_union([g for g in parts if not g.is_empty])
            if geom.is_empty:
                continue
            shapely.prepare(geom)
            segs = []
            for ring in shapely.get_parts(shapely.boundary(geom)):
                c = np.asarray(ring.coords)
                if len(c) >= 2:
                    segs.append(np.stack([c[:-1], c[1:]], axis=1))
            if not segs:
                continue
            lines = shapely.linestrings(np.concatenate(segs))
            self.nets[nid] = (geom, shapely.STRtree(lines))

    def gap(self, x: float, y: float, half_width: float, max_probe: float,
            exclude_net: int) -> Optional[float]:
        """Edge-to-edge gap from the trace at (x, y) to the nearest foreign
        fill within `max_probe`, else None."""
        import shapely
        best: Optional[float] = None
        pt = None
        for nid, (geom, tree) in self.nets.items():
            if nid == exclude_net:
                continue
            if shapely.contains_xy(geom, x, y):
                self.overlaps.add((round(x, 4), round(y, 4)))
                return 0.0
            if pt is None:
                pt = shapely.Point(x, y)
            idx, dist = tree.query_nearest(pt, max_distance=half_width + max_probe,
                                           return_distance=True)
            if len(dist):
                g = float(dist.min()) - half_width
                if g < -1e-6:
                    self.overlaps.add((round(x, 4), round(y, 4)))
                g = max(0.0, g)
                if best is None or g < best:
                    best = g
        return best


class LayerCopper:
    """Filled-pour occupancy for one copper layer.

    Backed by ZoneFillModel, which the fill-fidelity work made faithful to
    KiCad's actual filler (shape-true pad stamps, min-thickness opening,
    per-net-class clearances). Component ids matter as much as presence: a
    trace that stays over copper but moves from one fill COMPONENT to another
    has still crossed a slot, and its return current still has to detour.

    A fill component is the copper, not the zone that drew it (#1299): where
    two pours of the SAME net overlap on the layer their fills are one sheet
    of copper, so the components that overlap are one reference. Keyed by
    zone, a trace passing from one such pour's fill to the other's read as a
    split -- 64 phantom splits on a CM5 board whose GND plane was stacked on a
    same-net pour, and none once the stale pour was deleted.
    """

    def __init__(self, pcb, layer: str,
                 exact_polys: Optional[Dict[int, List]] = None):
        self.layer = layer
        self.entries: List[Tuple[int, int, object]] = []   # (idx, net_id, model)
        # Rasterization pitch of the coarsest model on this layer: how far a
        # model's fill edge can sit from the true one (half a cell either way).
        self.cell: float = 0.0
        # #1624: KiCad's OWN fill of the pours of these nets on this layer
        # (stored or refilled polygons; an empty list is a pour KiCad filled
        # with nothing). The side gap to them is measured at exact distance,
        # and the model is not asked about them for it.
        self.exact_nets = set(exact_polys or ())
        self.exact: Optional[ExactFill] = (
            ExactFill(exact_polys) if any((exact_polys or {}).values())
            else None)
        # (zone index, component) -> the key of the copper it belongs to; only
        # components merged with another pour's are listed.
        self._same_copper: Dict[Tuple[int, int], Tuple[int, int]] = {}
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
        self._merge_same_net_overlaps()

    def _merge_same_net_overlaps(self) -> None:
        """Union every pair of same-net fill components that overlap, keyed to
        the smallest (zone index, component) of each union so the key does not
        depend on which pour a probe hits first."""
        by_net: Dict[int, List[Tuple[int, object]]] = {}
        for idx, net_id, model in self.entries:
            by_net.setdefault(net_id, []).append((idx, model))
        parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

        def find(k):
            while parent.get(k, k) != k:
                parent[k] = parent.get(parent[k], parent[k])
                k = parent[k]
            return k

        for pours in by_net.values():
            for a in range(len(pours)):
                for b in range(a + 1, len(pours)):
                    ia, ma = pours[a]
                    ib, mb = pours[b]
                    for la, lb in _overlapping_components(ma, mb):
                        ra, rb = find((ia, la)), find((ib, lb))
                        if ra != rb:
                            lo, hi = min(ra, rb), max(ra, rb)
                            parent[hi] = lo
        self._same_copper = {k: find(k) for k in parent}

    def __bool__(self) -> bool:
        return bool(self.entries) or bool(self.exact_nets)

    def model_only_nets(self) -> set:
        """Nets with a pour here whose side gap the fill model answers."""
        return {n for _i, n, _m in self.entries} - self.exact_nets

    def key_at(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """(zone index, fill component) of the copper covering (x, y), else
        None. Overlapping same-net fills share one key (see the class note).

        Deterministic: zones are probed in file order and the first hit wins,
        so two runs over the same board classify a point identically.
        """
        for idx, _net_id, model in self.entries:
            comp = model.query_component(x, y)
            if comp:
                key = (idx, comp)
                return self._same_copper.get(key, key)
        return None

    def foreign_key_at(self, x: float, y: float,
                       exclude_net: int) -> Optional[Tuple[int, int]]:
        """Same as key_at but ignoring pours belonging to `exclude_net` -- used
        for the coplanar-ground probe, where the trace's own net's pour is not
        a coplanar ground, it is the trace. Pours whose KiCad fill is known
        (`exact_nets`) are measured by `ExactFill` instead and skipped here."""
        for idx, net_id, model in self.entries:
            if net_id == exclude_net or net_id in self.exact_nets:
                continue
            if model.query_component(x, y):
                return (idx, net_id)
        return None


# =============================================================================
# Measurement
# =============================================================================

def _pair_partners(pcb) -> Dict[int, int]:
    """net_id -> its diff-pair partner's net_id, by `extract_diff_pair_base`
    (same base, same suffix style, opposite polarity)."""
    from net_queries import extract_diff_pair_base
    halves: Dict[Tuple[str, str, bool], int] = {}
    for nid, net in pcb.nets.items():
        got = extract_diff_pair_base(getattr(net, 'name', '') or '')
        if got:
            base, pos, style = got
            halves[(base, style, pos)] = nid
    return {nid: halves[(b, st, not pos)]
            for (b, st, pos), nid in halves.items() if (b, st, not pos) in halves}


def _via_spans(via, layer: str, order: List[str]) -> bool:
    """Does `via`'s barrel pass through copper `layer`?"""
    layers = [l for l in (getattr(via, 'layers', None) or ()) if l in order]
    if len(layers) < 2 or layer not in order:
        return True             # unknown span: treat as through
    lo, hi = sorted(order.index(l) for l in layers[:2])
    return lo <= order.index(layer) <= hi


def own_antipad_radius(pcb, ref_layer: str, via, cell: float) -> float:
    """How far the reference fill on `ref_layer` stays off `via`: its radius
    plus the clearance the fill model carves it at -- max(zone clearance, the
    plane net's class, the via net's class), as `ZoneFillModel` stamps a
    foreign via -- plus one model cell for its cell-centre quantisation."""
    return via.size / 2.0 + _pour_clearance(pcb, ref_layer, via.net_id) + cell


def _pour_clearance(pcb, layer: str, net_id: int) -> float:
    """The clearance the fill model carves the pours on `layer` at around an
    item of `net_id`: max(zone clearance, the plane net's class, the item
    net's class), as `ZoneFillModel` stamps foreign copper."""
    try:
        from plane_fill_model import _NC_ATTR
        import routing_defaults as _defaults
        nc = getattr(pcb, _NC_ATTR, None) or {}
        default_zc = _defaults.PLANE_ZONE_CLEARANCE
    except Exception:
        nc, default_zc = {}, 0.0
    clr = 0.0
    for z in getattr(pcb, 'zones', None) or ():
        if z.layer != layer or not z.net_id or not z.polygon:
            continue
        zc = z.clearance if z.clearance is not None else default_zc
        clr = max(clr, zc, nc.get(z.net_id, 0.0), nc.get(net_id, 0.0))
    return clr


def own_pad_halo(pcb, layer: str, pad, cell: float) -> float:
    """How far past the net's own `pad` the pour on `layer` is still shaped by
    it (#1582): the clearance the fill keeps from the pad (its own override
    included) plus one model cell for the cell-centre quantisation, as
    :func:`own_antipad_radius` sizes a via's."""
    return max(_pour_clearance(pcb, layer, pad.net_id),
               getattr(pad, 'local_clearance', 0.0) or 0.0) + cell


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

    Pours whose KiCad fill is known (`copper.exact`) are measured exactly
    (#1624, see `ExactFill`); the rest are probed on the fill model, and the
    smaller of the two is the gap.
    """
    if not copper:
        return None
    best: Optional[float] = None
    if copper.exact is not None:
        best = copper.exact.gap(x, y, half_width, max_probe, exclude_net)
    if not copper.model_only_nets() - {exclude_net}:
        return best             # every foreign pour here was measured exactly
    # Unit normal to the trace direction.
    norm = math.hypot(dx, dy)
    if norm <= 0:
        return best
    nx, ny = -dy / norm, dx / norm

    steps = int(max_probe / probe_step) + 1
    for sign in (1.0, -1.0):
        d = half_width
        for _ in range(steps):
            d += probe_step
            if d - half_width > max_probe:
                break
            if copper.foreign_key_at(x + nx * sign * d, y + ny * sign * d,
                                     exclude_net):
                # #1624: find the model's fill edge between the last miss and
                # this hit, rather than de-biasing the step. The model's edge
                # is unbiased (a cell is fill when its centre is, so its edge
                # sits within half a cell either side of the true one:
                # 0.176-0.221 for a true 0.200 at a 0.05 cell); the old
                # `cell/2 + step/2` de-bias read a correct 0.20 as 0.15 or
                # 0.20 by where the grid fell, the 0.15 a tolerance miss.
                lo = max(half_width, d - probe_step)
                hi = d
                for _b in range(6):
                    mid = (lo + hi) / 2.0
                    if copper.foreign_key_at(x + nx * sign * mid,
                                             y + ny * sign * mid, exclude_net):
                        hi = mid
                    else:
                        lo = mid
                gap = (lo + hi) / 2.0 - half_width
                if best is None or gap < best:
                    best = gap
                break
    return None if best is None else max(0.0, best)


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


def _point_seg_dist(x: float, y: float, s) -> float:
    ex, ey = s.end_x - s.start_x, s.end_y - s.start_y
    l2 = ex * ex + ey * ey
    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((x - s.start_x) * ex
                                               + (y - s.start_y) * ey) / l2))
    return math.hypot(x - s.start_x - t * ex, y - s.start_y - t * ey)


def shipped_impedance(pcb, segs, partner_segs, spec: dict,
                      sample_step: float = DEFAULT_SAMPLE_STEP
                      ) -> Tuple[float, float]:
    """(ohms, length mm): the model's impedance of a net's SHIPPED copper,
    length-weighted, for its declaration `spec` (#1630). Single-ended: Z0 at
    each segment's width. Differential: Zdiff at each sample's width and its
    measured edge-to-edge gap to the partner on the same layer, over the
    length where the two are coupled (within COUPLED_GAP_FACTOR x the declared
    pair gap). A declared coplanar gap selects the CPW model, as the route
    step solved with. (0, 0) when nothing could be read."""
    from impedance import calculate_impedance_for_layer
    diff = bool(spec.get('differential'))
    cg = float(spec.get('coplanar_gap') or 0.0)
    pg = float(spec.get('pair_gap') or 0.0)
    limit = COUPLED_GAP_FACTOR * pg if pg > 0 else 0.5
    cache: Dict[Tuple[str, float, float], float] = {}

    def z_at(layer, width, gap):
        key = (layer, round(width, 5), round(gap, 4))
        if key not in cache:
            r = calculate_impedance_for_layer(pcb, layer, width, gap,
                                              coplanar_gap=cg)
            cache[key] = float(r.get('zdiff' if diff else 'z0') or 0.0)
        return cache[key]

    total = length = 0.0
    for seg in segs:
        seg_len = math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
        if seg_len <= 0 or not seg.width:
            continue
        samples = _sample_points(seg.start_x, seg.start_y, seg.end_x, seg.end_y,
                                 sample_step)
        n_int = max(1, len(samples) - 1)
        per = seg_len / n_int
        mates = [p for p in partner_segs if p.layer == seg.layer and p.width]
        for sx, sy, _ in samples[:n_int]:
            gap = 0.0
            if diff:
                if not mates:
                    continue
                gap = min(_point_seg_dist(sx, sy, p) - p.width / 2.0
                          for p in mates) - seg.width / 2.0
                if gap <= 0 or gap > limit:
                    continue
            z = z_at(seg.layer, seg.width, gap)
            if z > 0:
                total += z * per
                length += per
    return (total / length if length > 0 else 0.0), length


def publish_class_clearances(pcb) -> None:
    """Give the fill model the clearances KiCad fills at (#1624).

    KiCad fills a pour at max(zone clearance, the net class of each item);
    the fill model stamps at the zone's own clearance unless the board's
    class map is published, which only the plane steps did. A class at 0.2
    with the pour's zone at 0.12 then read 0.10 where KiCad leaves 0.20.
    Published from the sibling project, a Default-only net at the Default
    class; a map a caller already published is kept."""
    try:
        from plane_fill_model import _NC_ATTR, set_board_net_clearances
        from list_nets import (board_default_netclass_clearance,
                               net_clearance_map_by_id)
    except Exception:                                           # noqa: BLE001
        return
    path = getattr(pcb, 'source_path', '') or ''
    if getattr(pcb, _NC_ATTR, None) or not path or not os.path.isfile(path):
        return
    names = {nid: n.name for nid, n in (pcb.nets or {}).items()
             if nid and getattr(n, 'name', '')}
    try:
        nc = dict(net_clearance_map_by_id(path, names) or {})
        default = board_default_netclass_clearance(path)
    except Exception:                                           # noqa: BLE001
        return
    if default:
        for nid in names:
            nc.setdefault(nid, float(default))
    set_board_net_clearances(pcb, nc)


#: How `analyze_impedance` finds KiCad's fill for the coplanar gap (#1624).
FILL_SOURCES = ('auto', 'stored', 'model')


def kicad_fill_islands(pcb, fill_source: str = 'auto', refill: bool = True
                       ) -> Tuple[Optional[Dict[Tuple[str, str], List]], str, str]:
    """(islands, source, why) -- KiCad's fill of the board, for the gap.

    'stored' is the (filled_polygon ...) KiCad wrote into the board file: what
    KiCad shows, plots and grades without a refill. With none stored and
    `refill`, KiCad refills the board ('refill': the PCBData's own provider,
    else a pcbnew subprocess at the project's rules). Otherwise None and
    'model', with `why` naming the reason. `fill_source='stored'` never
    refills; 'model' reads no fill at all."""
    if fill_source == 'model':
        return None, 'model', 'fill_source=model'
    path = getattr(pcb, 'source_path', '') or ''
    if path and os.path.isfile(path):
        try:
            from kicad_parser import read_board_text
            from kicad_exact_fill import parse_filled_islands
            text = read_board_text(path)
            if '(filled_polygon' in text:
                got = parse_filled_islands(text)
                if got:
                    return got, 'stored', ''
        except Exception:                                       # noqa: BLE001
            pass
    if fill_source == 'stored' or not refill:
        return None, 'model', 'the board stores no fill'
    provider = getattr(pcb, 'exact_fill_provider', None)
    if callable(provider):
        try:
            got = provider()
        except Exception:                                       # noqa: BLE001
            got = None
        if got is not None:
            return got, 'refill', ''
    if not path or not os.path.isfile(path):
        return None, 'model', 'the board stores no fill and has no file to refill'
    try:
        from kicad_exact_fill import refill_islands_ex
        got, st = refill_islands_ex(path)
    except Exception as e:                                      # noqa: BLE001
        return None, 'model', f'the KiCad refill could not be attempted ({e})'
    if st.ok and got is not None:
        return got, 'refill', ''
    return None, 'model', st.why()


def _exact_polys_by_layer(pcb, islands, source: str
                          ) -> Dict[str, Dict[int, List]]:
    """{layer: {net_id: [polygon, ...]}} for every pour KiCad's fill covers.
    A refill fills every pour, so a pour it left out poured nothing (an
    empty list); a stored fill covers only the pours it carries."""
    if not islands:
        return {}
    ids: Dict[str, int] = {}
    for nid, n in (pcb.nets or {}).items():
        if nid and getattr(n, 'name', ''):
            ids.setdefault(n.name, nid)
    out: Dict[str, Dict[int, List]] = {}
    for (name, layer), polys in islands.items():
        nid = ids.get(name)
        if nid:
            out.setdefault(layer, {}).setdefault(nid, []).extend(polys)
    if source == 'refill':
        for z in getattr(pcb, 'zones', None) or ():
            if z.net_id and z.polygon:
                out.setdefault(z.layer, {}).setdefault(z.net_id, [])
    return out


def analyze_impedance(pcb, net_patterns: Optional[List[str]] = None,
                      sample_step: float = DEFAULT_SAMPLE_STEP,
                      min_void_length: float = DEFAULT_MIN_VOID_LENGTH,
                      ratio_limit: float = CPWG_GAP_RATIO_LIMIT,
                      include_plane_nets: bool = False,
                      coplanar_gap: float = 0.0,
                      gap_tolerance: float = DEFAULT_GAP_TOLERANCE,
                      board_name: str = "",
                      net_declared_gaps: Optional[Dict[int, float]] = None,
                      fill_source: str = 'auto',
                      net_declared_ohms: Optional[Dict[int, dict]] = None
                      ) -> BoardImpedanceReport:
    """Walk every routed segment and measure what the pre-route model assumed.

    Args:
        pcb: Parsed PCBData (post-route, pours filled).
        net_patterns: fnmatch patterns limiting which nets are analyzed.
            None/empty analyzes every routed signal net.
        sample_step: Distance between samples along a trace, mm.
        min_void_length: Ignore void runs shorter than this (fill
            quantisation). A run inside the antipad of the net's own via or
            plated through-hole pad, or its P/N partner's, is never a
            crossing, whatever its length (#1203, #1498).
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
        fill_source: Whose fill the coplanar gap is read on (#1624).
            'auto': the fill KiCad stored in the board, else -- when a
            declared gap is being audited -- a KiCad refill, else the fill
            model. 'stored': never refill. 'model': the fill model only.
            KiCad's fill is read at exact distance; the model to within half
            a cell. The reference-plane continuity check reads the model
            either way.
        net_declared_ohms: Per-net impedance declarations (net_id -> the
            sibling .kicad_pro record: ohms, differential, pair_gap,
            coplanar_gap), auto-read like the gaps. Each such net reports the
            model's impedance of its shipped copper against the declared one
            (#1630, `shipped_impedance`); disclosure only.

    Returns:
        BoardImpedanceReport. Diagnostic only; pcb's copper is not modified
        (the project's class clearances are published to its fill model,
        `publish_class_clearances`).
    """
    if fill_source not in FILL_SOURCES:
        raise ValueError(f"fill_source must be one of {FILL_SOURCES}, "
                         f"not {fill_source!r}")
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
    from kicad_parser import pour_zones
    # A teardropped SIGNAL net is still a signal net (#1164).
    pour_nets = {z.net_id for z in pour_zones(getattr(pcb, 'zones', None))
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

    # #1624: the model stamps at the clearances KiCad fills at, and the
    # coplanar gap is read on KiCad's own fill wherever it can be had. A
    # refill is a pcbnew run, so it is asked for only when a declared gap is
    # being audited.
    publish_class_clearances(pcb)
    auditing = any((net_declared_gaps or {}).get(nid, coplanar_gap) > 0
                   for nid, _nm in selected)
    islands, source, why = kicad_fill_islands(pcb, fill_source,
                                              refill=auditing)
    exact_by_layer = _exact_polys_by_layer(pcb, islands, source)
    report.gap_source = source
    if source == 'model' and auditing:
        report.notes.append(
            f"Coplanar gaps read on the fill MODEL, not KiCad's fill ({why}): "
            f"within half a model cell of KiCad's, and the model does not "
            f"read .kicad_dru rules. Fill the board (fill_for_delivery.py) "
            f"to grade on KiCad's own fill.")
    elif source in ('stored', 'refill'):
        report.notes.append(
            "Coplanar gaps read at exact distance on " +
            ("the fill KiCad stored in the board." if source == 'stored'
             else "a KiCad refill of the board."))

    # Lazily-built copper occupancy, one model set per layer (expensive to
    # rasterize, so shared across every net that references it).
    copper_cache: Dict[str, LayerCopper] = {}

    def copper_for(layer: str) -> LayerCopper:
        got = copper_cache.get(layer)
        if got is None:
            got = LayerCopper(pcb, layer, exact_by_layer.get(layer))
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

    # #1203: a trace that ends on its own via runs over that via's antipad on
    # the reference layer for about via radius + clearance (0.425 mm at a 0.45
    # via and 0.2 clearance), which no void-length filter separates from a
    # slot. Those samples are a layer change, not a missing plane.
    partners = _pair_partners(pcb)
    layer_order = copper_layers_in_order(pcb)
    vias_by_net: Dict[int, List] = {}
    for v in pcb.vias:
        if v.net_id:
            vias_by_net.setdefault(v.net_id, []).append(v)
    # #1498: a plated through-hole PAD of the net (or its partner) is the same
    # layer change: its barrel passes every reference layer, carving an
    # antipad its trace runs over, which no routing can remove. Sized as a
    # via of the pad's larger side, the way the fill model carves it.
    from types import SimpleNamespace
    try:
        from kicad_parser import pad_is_plated_through
    except Exception:                                        # noqa: BLE001
        pad_is_plated_through = None
    tht_by_net: Dict[int, List] = {}
    if pad_is_plated_through is not None:
        for fp in (pcb.footprints or {}).values():
            for pd in fp.pads:
                if pd.net_id and pad_is_plated_through(pd):
                    tht_by_net.setdefault(pd.net_id, []).append(SimpleNamespace(
                        x=pd.global_x, y=pd.global_y,
                        size=max(pd.size_x, pd.size_y), net_id=pd.net_id))

    # #1582: every copper pad of a net, by layer, for the coplanar audit. A
    # line landing on a pad wider than itself runs over the pad -- where the
    # conductor is the pad -- and then through the pour's clearance step
    # around it, and the gap read from the track's half-width misses the
    # declared one on both, which no routing can change.
    from net_queries import expand_pad_layers
    from routing_utils import point_to_pad_rect_dist
    copper_pads_by_net: Dict[int, List] = {}
    for fp in (pcb.footprints or {}).values():
        for pd in fp.pads:
            if pd.net_id and getattr(pd, 'pad_type', '') != 'np_thru_hole':
                copper_pads_by_net.setdefault(pd.net_id, []).append(pd)

    for net_id, net_name in selected:
        own_vias = list(vias_by_net.get(net_id, ()))
        own_pads = list(tht_by_net.get(net_id, ()))
        pads_on: Dict[str, List] = {}
        for pd in copper_pads_by_net.get(net_id, ()):
            for ly in expand_pad_layers(list(getattr(pd, 'layers', None) or ()),
                                        layer_order):
                pads_on.setdefault(ly, []).append(pd)
        if partners.get(net_id) is not None:
            own_vias += vias_by_net.get(partners[net_id], [])
            own_pads += tht_by_net.get(partners[net_id], [])
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
            antipad_any = [False] * len(samples)
            for ref_layer in ref_layers:
                ref_copper = copper_for(ref_layer)
                keys = [ref_copper.key_at(x, y) for x, y, _ in samples]
                # Samples inside the antipad of an own / partner via that
                # passes through this reference layer.
                discs = [(v.x, v.y, own_antipad_radius(pcb, ref_layer, v,
                                                       ref_copper.cell))
                         for v in own_vias
                         if _via_spans(v, ref_layer, layer_order)]
                # #1498: a through-hole pad spans every copper layer
                discs += [(pd.x, pd.y, own_antipad_radius(pcb, ref_layer, pd,
                                                          ref_copper.cell))
                          for pd in own_pads]
                lo_x = min(seg.start_x, seg.end_x)
                hi_x = max(seg.start_x, seg.end_x)
                lo_y = min(seg.start_y, seg.end_y)
                hi_y = max(seg.start_y, seg.end_y)
                discs = [d for d in discs
                         if lo_x - d[2] <= d[0] <= hi_x + d[2]
                         and lo_y - d[2] <= d[1] <= hi_y + d[2]]
                in_antipad = [key is None and any(
                                  math.hypot(x - dx, y - dy) <= dr
                                  for dx, dy, dr in discs)
                              for (x, y, _), key in zip(samples, keys)]
                for i, hit in enumerate(in_antipad):
                    if hit:
                        antipad_any[i] = True
                        if i == 0 or not in_antipad[i - 1]:
                            result.own_via_antipad_runs += 1

                run_start: Optional[int] = None
                for i, key in enumerate(keys + [object()]):   # sentinel closes
                    is_void = (i < len(keys)) and (key is None) \
                        and not in_antipad[i]
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
            result.length_over_own_antipad_mm += (
                sum(a and not v for a, v in zip(antipad_any[:n_int],
                                                void_any[:n_int])) * per_sample)
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
            # #1582: the net's own pads on this layer, each with how far past
            # it the pour is still shaped by it.
            halos = []
            if net_gap > 0:
                for pd in pads_on.get(layer, ()):
                    m = own_pad_halo(pcb, layer, pd, same_layer.cell)
                    reach = math.hypot(pd.size_x, pd.size_y) / 2.0 + m
                    if (min(seg.start_x, seg.end_x) - reach <= pd.global_x
                            <= max(seg.start_x, seg.end_x) + reach
                            and min(seg.start_y, seg.end_y) - reach <= pd.global_y
                            <= max(seg.start_y, seg.end_y) + reach):
                        halos.append((pd, m, reach))

            for (sx, sy, _) in samples[:n_int]:
                gap = measure_side_gap(same_layer, sx, sy, dx, dy, half_w,
                                       max_probe, net_id)
                # Over the net's own pad, or within its pour clearance of it.
                at_pad = [reach for pd, m, reach in halos
                          if point_to_pad_rect_dist(sx, sy, pd) <= m]
                if gap is None:
                    z0_here = z0_nom
                    if net_gap > 0:
                        # Beside a wide pad the pour can sit past the probe:
                        # probe past the pad before calling it no ground. No
                        # pour there either is a real break, and it counts.
                        if at_pad and measure_side_gap(
                                same_layer, sx, sy, dx, dy, half_w,
                                max_probe + max(at_pad), net_id) is not None:
                            result.length_at_own_pad_mm += per_sample
                        else:
                            result.length_no_ground_mm += per_sample
                else:
                    result.gap_samples.append(gap)
                    if net_gap > 0 and \
                            abs(gap - net_gap) > gap_tolerance:
                        if at_pad:
                            result.length_at_own_pad_mm += per_sample
                        else:
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

        # #1630: the declared ohms against the model's reading of the copper
        spec = (net_declared_ohms or {}).get(net_id)
        if spec and float(spec.get('ohms') or 0) > 0:
            result.declared_ohms = float(spec['ohms'])
            result.declared_differential = bool(spec.get('differential'))
            mates = (segments_by_net.get(partners.get(net_id), [])
                     if result.declared_differential else [])
            result.shipped_ohms, result.ohms_length_mm = shipped_impedance(
                pcb, segments_by_net[net_id], mates, spec, sample_step)

        if result.routed_length_mm > 0:
            report.nets.append(result)

    over = sum(len(c.exact.overlaps) for c in copper_cache.values()
               if c.exact is not None)
    if over:
        report.notes.append(
            f"{over} sample(s) of a trace lie under KiCad's {source} fill of "
            f"another net, which KiCad never fills: the fill predates the "
            f"copper (or the board is shorted). Read as gap 0; refill "
            f"(fill_for_delivery.py) and grade again.")

    runs = sum(n.own_via_antipad_runs for n in report.nets)
    if runs:
        report.notes.append(
            f"{runs} void run(s) inside an antipad of the net's own via or "
            f"through-hole pin (or its P/N partner's) are a layer change, not a "
            f"plane slot: counted as own_via_antipad_runs, never as a crossing "
            f"(#1203, #1498).")
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
    # #1582: whenever ANY net carries a declared gap -- a per-net declaration
    # read from the project counts as much as --coplanar-gap. It used to print
    # for the call-level flag only, so a project-declared board exited 1 on a
    # broken promise with no line saying why.
    declared = [n for n in report.nets if n.declared_gap > 0]
    if declared:
        gaps = sorted({round(n.declared_gap, 6) for n in declared})
        src = (f"--coplanar-gap {report.declared_gap:g}mm"
               if report.declared_gap > 0 and gaps == [round(report.declared_gap, 6)]
               else "declared gap " + ", ".join(f"{g:g}" for g in gaps) + "mm")
        lines.append("")
        # ASCII: this line now prints on every project-declared board, and a
        # parent reading it in another code page lost the whole stdout to one
        # undecodable '±' (Windows, test_meander_demo_boards).
        lines.append(f"-- Coplanar declaration audit ({src}, tolerance "
                     f"+/-{report.gap_tolerance:g}mm) --")
        outer = sum(n.outer_length_mm for n in declared)
        no_gnd = sum(n.length_no_ground_mm for n in declared)
        wrong = sum(n.length_gap_wrong_mm for n in declared)
        at_pad = sum(n.length_at_own_pad_mm for n in declared)
        ok = max(0.0, outer - no_gnd - wrong - at_pad)
        lines.append(f"  Outer-layer length declared coplanar: {outer:.1f} mm")
        if outer > 0:
            lines.append(f"    gap as declared:  {ok:>9.1f} mm  "
                         f"({100 * ok / outer:.1f}%)")
            lines.append(f"    gap off-target:   {wrong:>9.1f} mm  "
                         f"({100 * wrong / outer:.1f}%)")
            lines.append(f"    NO ground beside: {no_gnd:>9.1f} mm  "
                         f"({100 * no_gnd / outer:.1f}%)")
            if at_pad > 0:
                lines.append(f"    at its own pad:   {at_pad:>9.1f} mm  "
                             f"({100 * at_pad / outer:.1f}%; the pad and the "
                             f"pour's clearance step past it -- not counted)")
        broken = [n for n in declared if n.promise_broken]
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

    # --- declared impedance (#1630) ---
    held = [n for n in report.nets if n.declared_ohms > 0]
    if held:
        lines.append("")
        lines.append(f"-- Declared impedance vs the shipped copper (the model's "
                     f"reading; disclosure, tolerance +/-"
                     f"{DECLARED_OHMS_TOLERANCE_PCT:g}%) --")
        lines.append(f"  {'net':<28} {'declared':>9} {'shipped':>9} {'off':>8} "
                     f"{'read over mm':>13}")
        off = 0
        for n in sorted(held, key=lambda n: (-(abs(n.ohms_off_pct)
                                               if n.ohms_off_pct is not None
                                               else 1e9), n.net_name))[:top]:
            pct = n.ohms_off_pct
            unit = 'diff' if n.declared_differential else ''
            if pct is None:
                lines.append(f"  {n.net_name[:28]:<28} {n.declared_ohms:>7.1f}{unit[:1]:<2}"
                             f" {'-':>9} {'-':>8} {0.0:>13.1f}  (nothing to read: "
                             f"{'no coupled length' if n.declared_differential else 'no stackup'})")
                off += 1
                continue
            flag = '  OFF TARGET' if abs(pct) > DECLARED_OHMS_TOLERANCE_PCT else ''
            off += bool(flag)
            lines.append(f"  {n.net_name[:28]:<28} {n.declared_ohms:>7.1f}{unit[:1]:<2}"
                         f" {n.shipped_ohms:>9.1f} {pct:>+7.1f}% {n.ohms_length_mm:>13.1f}{flag}")
        lines.append(f"  {off} of {len(held)} declared net(s) off target or unread "
                     f"(not counted in the exit code).")

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
        'gap_source': report.gap_source,
        'notes': report.notes,
        'totals': {
            'nets_analyzed': len(report.nets),
            'routed_length_mm': round(report.routed_length_mm, 4),
            'nets_with_crossing': sum(1 for n in report.nets if n.crossings),
            'segments_over_void': report.segments_over_void,
            'crossings': report.total_crossings,
            'length_over_void_mm': round(report.length_over_void_mm, 4),
            'own_via_antipad_runs': sum(n.own_via_antipad_runs
                                        for n in report.nets),
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
            'length_at_own_pad_mm': round(
                sum(n.length_at_own_pad_mm for n in report.nets), 4),
            'nets_broken_promise': sorted(n.net_name for n in report.nets
                                          if n.promise_broken),
            # #1630: disclosure, not in the exit code
            'declared_ohms_tolerance_pct': DECLARED_OHMS_TOLERANCE_PCT,
            'nets_ohms_off_target': sorted(
                n.net_name for n in report.nets if n.declared_ohms > 0 and (
                    n.ohms_off_pct is None
                    or abs(n.ohms_off_pct) > DECLARED_OHMS_TOLERANCE_PCT)),
        },
        'nets': [
            {
                'net_id': n.net_id,
                'net_name': n.net_name,
                'routed_length_mm': round(n.routed_length_mm, 4),
                'length_over_void_mm': round(n.length_over_void_mm, 4),
                'segments_over_void': n.segments_over_void,
                'crossings': len(n.crossings),
                'own_via_antipad_runs': n.own_via_antipad_runs,
                'length_over_own_antipad_mm': round(
                    n.length_over_own_antipad_mm, 4),
                'void_fraction': round(n.void_fraction, 5),
                'tight_gap_length_mm': round(n.tight_gap_length_mm, 4),
                'outer_length_mm': round(n.outer_length_mm, 4),
                'length_no_ground_mm': round(n.length_no_ground_mm, 4),
                'length_gap_wrong_mm': round(n.length_gap_wrong_mm, 4),
                'length_at_own_pad_mm': round(n.length_at_own_pad_mm, 4),
                'declared_gap_mm': n.declared_gap,
                'promise_broken': n.promise_broken,
                'z0_nominal': round(n.z0_nominal, 3),
                'z0_measured': round(n.z0_measured, 3),
                'z0_error_pct': round(n.z0_error_pct, 3),
                'declared_ohms': n.declared_ohms,
                'declared_differential': n.declared_differential,
                'shipped_ohms': round(n.shipped_ohms, 3),
                'ohms_length_mm': round(n.ohms_length_mm, 4),
                'ohms_off_pct': (None if n.ohms_off_pct is None
                                 else round(n.ohms_off_pct, 3)),
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
    parser.add_argument('--nets', nargs='+', action='extend', metavar='PATTERN',
                        help='Only analyze these nets (fnmatch patterns). '
                             'Default: every routed net.')
    parser.add_argument('--sample-step', type=float, default=DEFAULT_SAMPLE_STEP,
                        help=f'Sample spacing along traces in mm '
                             f'(default {DEFAULT_SAMPLE_STEP})')
    parser.add_argument('--min-void-length', type=float,
                        default=DEFAULT_MIN_VOID_LENGTH,
                        help=f'Ignore void runs shorter than this in mm '
                             f'(default {DEFAULT_MIN_VOID_LENGTH}; filters fill '
                             f'quantisation. The net\'s own via and '
                             f'through-hole pin antipads, and its P/N '
                             f'partner\'s, are exempted whatever the length)')
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
    net_declared_ohms = {}
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
            net_declared_ohms = {_n2i[nm]: sp for nm, sp in _specs.items()
                                 if nm in _n2i}
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
        net_declared_gaps=net_declared_gaps,
        net_declared_ohms=net_declared_ohms)

    print(format_report(report, verbose=args.verbose, top=args.top))

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(report_to_dict(report), fh, indent=2)
        print(f"\nJSON written to {args.json}")

    if args.exit_zero:
        return 0
    broken_promise = any(n.promise_broken for n in report.nets)
    return 1 if (report.total_crossings or broken_promise) else 0


if __name__ == '__main__':
    sys.exit(main())

"""
Connectivity analysis utilities for PCB routing.

Functions for finding endpoints, analyzing stubs, computing MST paths,
and tracking segment connectivity.
"""
from __future__ import annotations

from collections import OrderedDict

import math
from typing import List, Optional, Tuple, Dict, Set

import numpy as np

from kicad_parser import PCBData, Segment, Via, Pad, Zone
from routing_config import GridRouteConfig, GridCoord
from routing_utils import pos_key, segment_length, POSITION_DECIMALS, build_layer_map
from geometry_utils import UnionFind
from routing_constants import BGA_DEFAULT_EDGE_TOLERANCE, BGA_EDGE_DETECTION_TOLERANCE


class _EndpointStub:
    """Lightweight stand-in for a Pad object at a stub free-end position.

    Used by check_multipoint_net when a stub endpoint (not an actual pad)
    needs to participate in zone-connectivity checks that access .layers etc.
    """
    __slots__ = ('x', 'y', 'global_x', 'global_y', 'layer', 'layers',
                 'drill', 'size_x', 'size_y', 'pad_number', 'net_id')

    def __init__(self, x: float, y: float, layer: str, net_id: int = 0):
        self.x = x
        self.y = y
        self.global_x = x
        self.global_y = y
        self.layer = layer
        self.layers = [layer]
        self.drill = 0
        self.size_x = 0
        self.size_y = 0
        self.pad_number = ''
        # #545 F3: carry the REAL net id. A hardcoded 0 made
        # _pad_all_layer_reach short-circuit False for every stub terminal,
        # so a stub free end sitting exactly on a same-net through via was
        # never offered on all layers (Phase 1 pre-island-union, Phase 3
        # targets) -- the via right under the terminal was invisible.
        self.net_id = net_id


def _make_endpoint_stub(x: float, y: float, layer: str,
                        net_id: int = 0) -> '_EndpointStub':
    """Create a pad-like object for a stub free-end position."""
    return _EndpointStub(x, y, layer, net_id)


def _pad_touches_copper_group(pad, group, net_vias, routing_layers) -> bool:
    """Pad centre on/near any segment of the group (mid-segment too),
    layer-gated and via-in-pad aware.

    Hoisted from the multipoint path (#545 F8) so get_net_endpoints Case 2
    can use the same test: its old check compared the pad centre to segment
    ENDPOINTS on ANY layer within 0.05mm -- a B.Cu pad 0.05mm from an In1.Cu
    endpoint with no via graded "connected", Case 2 skipped it, and Case 4
    then declared the net already routed without ever verifying.

    Conservative: claims connection only when group copper provably reaches
    well inside the pad area (segment half-width plus a quarter of the pad's
    smaller dimension). Under-claiming just adds a redundant endpoint the
    router connects in a few iterations; over-claiming recreates the
    phantom-success bug.
    """
    from net_queries import expand_pad_layers
    px, py = pad.global_x, pad.global_y
    pad_layers = expand_pad_layers(pad.layers, routing_layers)
    reach_pad = min(pad.size_x, pad.size_y) / 4 if (pad.size_x and pad.size_y) else 0.05
    # A via-in-pad fans an SMD pad out to every copper layer, so the
    # group's copper may legitimately reach the pad on a layer the pad
    # does not itself live on (e.g. an F.Cu pad escaped to B.Cu). Treat
    # such a pad like a through-hole pad and skip the layer gate; else it
    # is mis-classed as unconnected and the net is falsely promoted to a
    # multi-point route that re-runs the already-routed leg. The test is
    # deliberately a genuine via-IN-pad (via centre inside the pad
    # rectangle), NOT mere annulus overlap: a same-net via merely grazing
    # the pad edge is some neighbour's escape, and crediting it would
    # collapse endpoints the router still needs, perturbing dense
    # multi-drop nets.
    half_x = (pad.size_x or 0) / 2
    half_y = (pad.size_y or 0) / 2
    reaches_all_layers = getattr(pad, 'drill', 0) > 0 or any(
        abs(v.x - px) <= half_x and abs(v.y - py) <= half_y
        for v in net_vias
    )
    for seg in group:
        if seg.layer not in pad_layers and not reaches_all_layers:
            continue
        dx = seg.end_x - seg.start_x
        dy = seg.end_y - seg.start_y
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-8:
            cx, cy = seg.start_x, seg.start_y
        else:
            t = ((px - seg.start_x) * dx + (py - seg.start_y) * dy) / seg_len_sq
            t = max(0.0, min(1.0, t))
            cx = seg.start_x + t * dx
            cy = seg.start_y + t * dy
        dist = math.sqrt((px - cx) ** 2 + (py - cy) ** 2)
        if dist <= seg.width / 2 + reach_pad:
            return True
    return False


def _get_pad_coords(p) -> Tuple[float, float]:
    """Get x, y coordinates from Pad object or dict."""
    if hasattr(p, 'global_x'):
        return p.global_x, p.global_y
    elif isinstance(p, dict):
        return p['x'], p['y']
    else:
        raise ValueError(f"Unknown pad type: {type(p)}")


def calculate_stub_length(pcb_data, net_id: int) -> float:
    """
    Calculate the total length of existing stub segments for a net.

    Stubs are the pre-existing track segments that connect from pads to the
    routing endpoints. For accurate length matching, we need to include these
    in the total pad-to-pad length.

    Args:
        pcb_data: PCB data containing existing segments
        net_id: Net ID to calculate stub length for

    Returns:
        Total stub length in mm
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    total = 0.0
    for seg in net_segments:
        total += segment_length(seg)
    return total


def is_edge_stub(pad_x: float, pad_y: float, bga_zones: List) -> bool:
    """Check if a pad is on the outer row/column of any BGA zone.

    Args:
        pad_x, pad_y: Pad position to check (not stub endpoint)
        bga_zones: List of BGA zones, either:
                   - 4-tuple: (min_x, min_y, max_x, max_y) - uses default tolerance
                   - 5-tuple: (min_x, min_y, max_x, max_y, edge_tolerance)

    Returns:
        True if pad is inside a BGA zone AND in the outer row/column
    """
    for zone in bga_zones:
        min_x, min_y, max_x, max_y = zone[:4]
        # Use per-zone tolerance if available, else default (0.5mm margin + 1mm pitch * 1.1)
        edge_tolerance = zone[4] if len(zone) > 4 else BGA_DEFAULT_EDGE_TOLERANCE
        # Check if pad is INSIDE this zone (with small tolerance for floating point)
        inside_tolerance = BGA_EDGE_DETECTION_TOLERANCE
        if (pad_x >= min_x - inside_tolerance and pad_x <= max_x + inside_tolerance and
            pad_y >= min_y - inside_tolerance and pad_y <= max_y + inside_tolerance):
            # Check if it's on an edge (within tolerance of min/max)
            on_left = abs(pad_x - min_x) <= edge_tolerance
            on_right = abs(pad_x - max_x) <= edge_tolerance
            on_bottom = abs(pad_y - min_y) <= edge_tolerance
            on_top = abs(pad_y - max_y) <= edge_tolerance
            if on_left or on_right or on_bottom or on_top:
                return True
    return False


# THE strict endpoint-coincidence tolerance (#320). Every coincidence-based
# consumer (segment grouping, stub free ends, cycle-prune port clustering,
# dead-end chaining, and the cleanup gates' width-clamped strict graph) uses
# this ONE constant: 0.02mm = 2 x check_drc's _SOFT_JOINT_MIN_GAP, i.e. two
# endpoints may each carry quantization-level slack (<=10um) and still count
# as coincident, while anything a soft joint could flag does not. The
# PHYSICAL (overlap) definition in check_net_connectivity is deliberately
# untouched: it grades electrical reality; this constant grades geometric
# intent. See issue #320 for the role assignment.
COINCIDENCE_TOL = 0.02


def endpoint_reaches_pad(x, y, radius, layers, pad) -> set:
    """Which of `layers` a disc of copper -- centre (x, y), radius `radius` --
    both SHARES with `pad`'s copper and physically OVERLAPS. Empty set = no
    contact.

    THE one predicate for "does this copper reach this pad". A track
    endpoint's round cap and a via barrel are the SAME question with a
    different radius; answering them separately is how #695 and #722 shipped
    four lines apart, in four copies across three modules.

    GEOMETRY mirrors check_connected's endpoint-in-pad rule exactly --
    ``_m = max(ewidth / 2 - 1e-6, tolerance)`` -- and its via twin,
    ``_m = max(vsize / 2 - 1e-6, tolerance)``, whose `tolerance` default IS
    COINCIDENCE_TOL. `margin` inflates the EXACT pad outline (custom-pad
    polygons, roundrect corners and rect_rotation all hold), so this reads
    "the copper overlaps the pad copper", not "the centre is inside it".
    COINCIDENCE_TOL is a FLOOR under the credit, never a replacement for it,
    exactly as it is there.

    Note the role split this file documents above: COINCIDENCE_TOL grades
    geometric INTENT. Asking a copper-reaches-copper question with it is a
    category error -- it made these checkers contradict check_net_connectivity
    on copper KiCad grades joined, and their exit codes are chain-blocking.

    LAYERS come from `net_queries.expand_pad_layers`, the SAME expansion
    check_connected uses, rather than a local reading of `pad.layers`:

      * `*.Cu` means every copper layer; `*.Mask` and `*.Paste` mean NO copper
        layer. A local `any('*' in L)` test reads a mask wildcard as an
        all-layer copper pad -- and 641 pads in kicad_files/ carry `*.Mask`.
      * a drilled pad occupies the layers it DECLARES. `drill > 0 -> every
        layer` over-credits a PTH pad declared ("F.Cu" "B.Cu") on a 4-layer
        board, which the authority grades as not reaching In1.Cu at all.
      * an NPTH pad is a hole with no copper whatever its layer list says
        (#328). `pad_is_plated_through` is the spelling for the neighbouring
        question ("does this barrel tie copper layers"); here the pad simply
        has no copper to land on.

    Crediting the cap WITHOUT the layer guard trades one contradiction for
    another: B.Cu ends near an F.Cu-only pad are genuinely split, and a
    layer-blind widening silences that -- the direction that ships broken
    copper.
    """
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return set()                        # a hole, not copper (#328)
    # Local import: net_queries imports this module, so a module-level import
    # would cycle -- the same reason check_net_connectivity is imported below.
    from net_queries import expand_pad_layers
    want = list(layers)
    on = set(want) & set(expand_pad_layers(list(getattr(pad, 'layers', None)
                                                or ()), want))
    if not on:
        return set()                        # cheap test first; geometry is the cost
    from check_connected import _point_in_pad
    if _point_in_pad(x, y, pad, margin=max(radius - 1e-6, COINCIDENCE_TOL)):
        return on
    return set()


def via_copper_layers(via, copper_layers=None) -> set:
    """The copper layers `via`'s barrel actually occupies.

    check_connected's rule: a via listing both F.Cu and B.Cu (or listing
    nothing) spans every copper layer; otherwise it spans what it declares.
    When `copper_layers` is supplied, a blind/buried via also gets the layers
    BETWEEN its endpoints -- a buried F.Cu-In2.Cu via touches In1.Cu, and
    treating the span as the two endpoints alone manufactured phantom
    unsupported-via findings (check_weird._via_span, same rule).
    """
    declared = [L for L in (getattr(via, 'layers', None) or ()) if L.endswith('.Cu')]
    if not declared or ('F.Cu' in declared and 'B.Cu' in declared):
        return set(copper_layers or declared or ())
    if copper_layers:
        order = list(copper_layers)
        idx = [order.index(L) for L in declared if L in order]
        if len(idx) >= 2:
            return set(order[min(idx):max(idx) + 1])
    return set(declared)


def endpoint_reaches_via(x, y, radius, via, layers, copper_layers=None) -> bool:
    """Does a disc of copper -- centre (x, y), radius `radius`, living on
    `layers` -- overlap `via`'s barrel?

    The via twin of `endpoint_reaches_pad`, and the same mirror of the
    authority: check_connected credits a via against a segment at
    ``max((psize + seg.width) / 2 - eps, tolerance)`` -- barrel radius PLUS
    the copper's own radius -- and creates via points ONLY on the via's own
    copper layers.

    Both halves matter, and only together. The soft-joint anchors used a flat
    ``vr + 0.01``: barrel-aware but CAP-BLIND, the very defect #722 reports on
    the pad branch. Widening that to `vr + radius` WITHOUT the layer test just
    makes a layer-blind credit ~37% wider -- on a 0.6mm via and a 0.25mm track,
    0.310mm becomes 0.425mm -- so a blind F.Cu/In1.Cu via would anchor B.Cu
    track ends it carries no copper for, silencing a real open.

    A via with no declared size claims the 0.6 default, matching
    ``getattr(via, 'size', 0.6)`` in check_connected. A via whose size IS zero
    keeps zero there, so it keeps zero here.
    """
    if not (set(layers) & via_copper_layers(via, copper_layers)):
        return False
    vr = (getattr(via, 'size', 0.6) if getattr(via, 'size', None) is not None
          else 0.6) / 2.0
    return math.hypot(x - via.x, y - via.y) <= max(vr + radius - 1e-6,
                                                   COINCIDENCE_TOL)



_CLUSTER_MEMO: "OrderedDict[tuple, tuple]" = OrderedDict()
_CLUSTER_MEMO_CAP = 4096


def cluster_coincident_points(points, tol: float = COINCIDENCE_TOL):
    """Union-find clustering of (x, y, layer) points by endpoint coincidence.

    Two points join when |dx| < tol and |dy| < tol on the SAME layer (pass a
    shared sentinel layer for layer-agnostic clustering). O(n) via spatial
    hashing. Returns a list of cluster root indices, one per input point --
    the ONE shared implementation behind every coincidence consumer (#320).

    Memoized (2026-08-14 profiling: 343k calls / 54s, endpoint derivations
    re-clustering unchanged nets across rescue/escalation attempts): the
    function is pure in (points, tol), so an exact-key hit is identical by
    construction. A fresh list is returned per call (callers may mutate).
    """
    _key = (tuple(points), tol)
    _hit = _CLUSTER_MEMO.get(_key)
    if _hit is not None:
        _CLUSTER_MEMO.move_to_end(_key)
        return list(_hit)
    n = len(points)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    inv = 1.0 / tol
    buckets: Dict[Tuple[int, int], List[int]] = {}
    for i, (x, y, _layer) in enumerate(points):
        buckets.setdefault((int(x * inv), int(y * inv)), []).append(i)
    for i, (x, y, layer) in enumerate(points):
        gx, gy = int(x * inv), int(y * inv)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for j in buckets.get((gx + dx, gy + dy), ()):
                    if j <= i:
                        continue
                    ox, oy, olayer = points[j]
                    if olayer == layer and abs(x - ox) < tol and abs(y - oy) < tol:
                        union(i, j)
    roots = [find(i) for i in range(n)]
    _CLUSTER_MEMO[_key] = tuple(roots)
    while len(_CLUSTER_MEMO) > _CLUSTER_MEMO_CAP:
        _CLUSTER_MEMO.popitem(last=False)
    return roots


def find_connected_groups(segments: List[Segment], tolerance: float = COINCIDENCE_TOL,
                          layer_aware: bool = True, vias: List = None) -> List[List[Segment]]:
    """Find groups of connected segments using union-find with spatial hashing.

    Uses O(n) spatial hashing instead of O(n²) pairwise comparison.

    Args:
        segments: List of segments to group
        tolerance: Position tolerance for endpoint matching
        layer_aware: If True, only connect segments on the same layer (default).
                    If False, connect segments regardless of layer.
        vias: Optional list of vias. If provided with layer_aware=True, segments
              on different layers at the same position are connected if there's a via.

    Returns:
        List of segment groups, where each group is a list of connected segments.
    """
    if not segments:
        return []

    n = len(segments)
    uf = UnionFind()

    # Build via position set for quick lookup
    via_positions = set()
    if vias:
        for via in vias:
            via_positions.add((round(via.x, 3), round(via.y, 3)))

    # Endpoint coincidence via the ONE shared primitive (#320). Cluster
    # layer-agnostically, then apply the layer rule inside each spatial
    # cluster: same layer joins outright; different layers join only through
    # a via at that position (the original pairwise rule, unchanged).
    _AGNOSTIC = None
    points = []
    owners = []  # (segment_index, layer, x, y)
    for i, seg in enumerate(segments):
        for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
            points.append((x, y, _AGNOSTIC))
            owners.append((i, seg.layer, x, y))
    roots = cluster_coincident_points(points, tolerance)
    by_cluster: Dict[int, List[int]] = {}
    for pi, r in enumerate(roots):
        by_cluster.setdefault(r, []).append(pi)
    for members in by_cluster.values():
        if len(members) < 2:
            continue
        for a_idx in range(len(members)):
            i, la, xa, ya = owners[members[a_idx]]
            for b_idx in range(a_idx + 1, len(members)):
                j, lb, xb, yb = owners[members[b_idx]]
                if i == j:
                    continue
                if abs(xa - xb) >= tolerance or abs(ya - yb) >= tolerance:
                    continue  # same cluster by transitivity, not direct contact
                if layer_aware and la != lb:
                    pos_key = (round(xa, 3), round(ya, 3))
                    if pos_key not in via_positions:
                        continue  # Not connected without via
                uf.union(i, j)

    # Group segments by their root
    groups: Dict[int, List[Segment]] = {}
    for i in range(n):
        root = uf.find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


def _point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Check if a point (x, y) is inside a polygon using ray casting algorithm."""
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def get_copper_connected_terminal_groups(
    pcb_data: PCBData,
    net_id: int,
    pad_info: List[Tuple],
) -> Dict[int, int]:
    """Group multipoint terminals by the net's EXISTING copper connectivity,
    using the authoritative overlap-aware definition (check_net_connectivity:
    cap overlap, T-junctions, zones, pad outlines and via-in-pad all count).

    Soft-jointed copper -- endpoints not coincident but caps overlapping -- is
    ONE group here and must never be re-tapped (issue #317). This is what the
    0.02mm endpoint-coincidence grouping of get_zone_connected_pad_groups got
    wrong for MST seeding: it called overlap-joined escapes disconnected, so
    the multipoint MST routed redundant loops between already-connected copper
    while the genuinely missing edge could still fail.

    pad_info rows are (gx, gy, layer_idx, orig_x, orig_y, endpoint_obj) as
    returned by get_multipoint_net_pads; endpoint_obj is a real Pad or an
    _EndpointStub sitting on a copper free end.

    Returns {pad_info index -> component id}. Terminals that cannot be tied
    to any copper/pad point each get a unique (negative) component.
    """
    components, _copper, _segs, _vias = get_terminal_component_info(
        pcb_data, net_id, pad_info)
    return components


def get_terminal_component_info(
    pcb_data: PCBData,
    net_id: int,
    pad_info: List[Tuple],
) -> Tuple[Dict[int, int], Dict[int, int], Dict[int, List[Segment]],
           Dict[int, List]]:
    """get_copper_connected_terminal_groups plus per-component copper (#348).

    Returns (components, copper_count, segs_by_component, vias_by_component):
      components        -- {pad_info index -> component id} (see the wrapper)
      copper_count      -- {component id -> number of existing net segments}
      segs_by_component -- {component id -> [existing net Segment, ...]}
      vias_by_component -- {component id -> [existing net Via, ...]} (#545 F9)
    The phase-1-exhausted fallback uses copper_count to pick the copper-richest
    base island and segs_by_component to seed tap sources from exactly that
    island's copper. Stub terminals resolve through the same nearest-endpoint
    matching as the wrapper, so islands whose terminal is a free end (not a
    real pad) participate fully.

    vias_by_component comes from the connectivity graph's via_index_repr --
    the AUTHORITATIVE membership (tolerance-clustered, mid-segment T-taps
    included), replacing the callers' hand-rolled rounded-endpoint-key /
    proximity-box matches, which missed a via 5um across a rounding-bucket
    boundary and every via tapped into a segment's middle (#545 F9).
    Position-sorted for GUI/CLI order-independence.
    """
    # Local import: check_connected -> net_queries -> connectivity would cycle.
    from check_connected import check_net_connectivity

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
    net_zones = [z for z in pcb_data.zones if z.net_id == net_id]
    # ALL net pads (not just terminals): a non-terminal pad can be the copper
    # that bridges two islands (e.g. both escapes land in one connector pad).
    net_pads = pcb_data.pads_by_net.get(net_id, [])

    res = check_net_connectivity(net_id, net_segments, net_vias, net_pads,
                                 net_zones, return_graph=True,
                                 pcb_data=pcb_data)
    graph = res.get('graph') or {}
    uf = UnionFind()
    for a, b in graph.get('edges', []):
        uf.union(a, b)

    # Map real-Pad terminals through the pad's representative point id; the
    # terminal objects come from pads_by_net, so identity holds (objects are
    # alive in net_pads, so id() keying is safe here).
    pad_obj_repr = {id(net_pads[idx]): rep
                    for idx, rep in graph.get('pad_index_repr', {}).items()}

    # Stub terminals sit on a segment free end; segment i's endpoints are
    # point ids 2i / 2i+1 in the connectivity graph.
    endpoint_points = []  # (x, y, layer, point_id)
    for si, seg in enumerate(net_segments):
        endpoint_points.append((seg.start_x, seg.start_y, seg.layer, 2 * si))
        endpoint_points.append((seg.end_x, seg.end_y, seg.layer, 2 * si + 1))

    components: Dict[int, int] = {}
    next_unique = -1
    for i, info in enumerate(pad_info):
        obj = info[5] if len(info) > 5 else None
        rep = None
        if obj is not None and id(obj) in pad_obj_repr:
            rep = pad_obj_repr[id(obj)]
        elif obj is not None:
            # Stub (or unmatched pad-like) terminal: nearest same-layer
            # segment endpoint within the coincidence tolerance that created
            # the stub. Terminal coords are info[3]/info[4] (orig x/y).
            tx, ty = info[3], info[4]
            tlayer = getattr(obj, 'layer', None)
            best = None
            for ex, ey, elayer, pid in endpoint_points:
                if tlayer is not None and elayer != tlayer:
                    continue
                d = abs(ex - tx) + abs(ey - ty)
                if d < 0.01 and (best is None or d < best[0]):
                    best = (d, pid)
            if best is not None:
                rep = best[1]
        if rep is None:
            components[i] = next_unique
            next_unique -= 1
        else:
            components[i] = uf.find(rep)

    copper_count: Dict[int, int] = {}
    segs_by_component: Dict[int, List[Segment]] = {}
    for si, seg in enumerate(net_segments):
        root = uf.find(2 * si)
        copper_count[root] = copper_count.get(root, 0) + 1
        segs_by_component.setdefault(root, []).append(seg)

    # #545 F9: per-component vias straight from the graph's membership.
    vias_by_component: Dict[int, List] = {}
    for _vidx, _vpid in (graph.get('via_index_repr') or {}).items():
        try:
            _via = net_vias[_vidx]
        except (IndexError, TypeError):
            continue
        vias_by_component.setdefault(uf.find(_vpid), []).append(_via)
    for _vlist in vias_by_component.values():
        _vlist.sort(key=lambda v: (v.x, v.y))

    # CANONICAL component LABELS. uf.find() returns an arbitrary representative
    # decided by the union ORDER, which follows the order segments sit in the
    # board -- and the GUI and CLI hold identical copper in different order. The
    # partition is the same either way, but the LABELS are not, and downstream
    # code sorts on them: compute_component_mst_edges does
    # `comp_ids = sorted(comp_terminals)` and starts its tree from comp_ids[0],
    # so the whole MST (and the "longest edge" the multipoint router picks
    # first) depended on which copper happened to be written first. Measured on
    # eth_tap: both fronts found a 2.10mm longest edge but different pad pairs
    # (0-3 vs 3-1).
    #
    # Relabel by the component's own content: the smallest terminal index it
    # owns, else the smallest segment anchor. That is a property of the net, so
    # both fronts agree, and terminal-bearing components keep low ids (their
    # natural pad order), which preserves the previous "start from the first
    # terminal's component" behaviour on boards where order never differed.
    _first_term: Dict[int, int] = {}
    for i, r in components.items():
        if r not in _first_term or i < _first_term[r]:
            _first_term[r] = i
    _anchor: Dict[int, tuple] = {}
    for r, segs in segs_by_component.items():
        a = None
        for _s in segs:
            k = (min((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)),
                 max((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)), _s.layer)
            if a is None or k < a:
                a = k
        _anchor[r] = a

    _roots = set(components.values()) | set(copper_count) | set(segs_by_component)
    _BIG = len(pad_info) + 1

    def _canon_key(r):
        return (_first_term.get(r, _BIG), _anchor.get(r) is None, _anchor.get(r) or ())

    remap = {r: i for i, r in enumerate(sorted(_roots, key=_canon_key))}
    components = {i: remap[r] for i, r in components.items()}
    copper_count = {remap[r]: c for r, c in copper_count.items()}
    segs_by_component = {remap[r]: v for r, v in segs_by_component.items()}
    vias_by_component = {remap[r]: v for r, v in vias_by_component.items()
                         if r in remap}
    return components, copper_count, segs_by_component, vias_by_component


def compute_component_mst_edges(
    positions: List[Tuple[float, float]],
    components: Dict[int, int],
) -> List[Tuple[int, int, float]]:
    """Minimum spanning tree over connected COMPONENTS of terminals (#317).

    Nodes are component ids; the distance between two components is the
    minimum Manhattan distance over their terminal pairs, and the chosen MST
    edge is realized by that closest pair. Joining N components takes exactly
    N-1 routed connections; intra-component edges never appear, so copper the
    authoritative checker already grades connected is never re-routed.

    Args:
        positions: (x, y) per terminal, indexed like pad_info
        components: {terminal index -> component id} (missing -> own index)

    Returns [(idx_a, idx_b, dist)] with indices into `positions`; idx_a is on
    the already-spanned side of the tree when the edge is added.
    """
    n = len(positions)
    comp_terminals: Dict[int, List[int]] = {}
    for i in range(n):
        comp_terminals.setdefault(components.get(i, i), []).append(i)
    comp_ids = sorted(comp_terminals)
    num_comps = len(comp_ids)
    if num_comps <= 1:
        return []

    # Classic O(C^2) Prim keyed on components, each candidate edge kept as its
    # realizing closest terminal pair. best[c] = (dist, term_in_tree, term_in_c)
    # is the cheapest connection from the current tree to component c. Growing
    # the tree relaxes each outside component against ONLY the newly added
    # component's terminals, so total terminal-pair work is sum(|A|*|B|) <=
    # N^2/2 -- same order as the old pad-position MST (a fresh power net is all
    # singleton components), never O(C^3) re-scans of tree-vs-outside pairs.
    # Deterministic: sorted component ids, strict-< relaxation, id tie-break.
    INF = float('inf')
    best: Dict[int, Tuple[float, int, int]] = {c: (INF, -1, -1) for c in comp_ids}

    def relax(from_comp: int) -> None:
        for a in comp_terminals[from_comp]:
            ax, ay = positions[a]
            for c, entry in best.items():
                for b in comp_terminals[c]:
                    bx, by = positions[b]
                    d = abs(ax - bx) + abs(ay - by)
                    if d < entry[0]:
                        entry = (d, a, b)
                best[c] = entry

    start = comp_ids[0]
    del best[start]
    relax(start)
    edges: List[Tuple[int, int, float]] = []
    while best:
        next_comp = min(best, key=lambda c: (best[c][0], c))
        d, a, b = best.pop(next_comp)
        edges.append((a, b, d))
        relax(next_comp)
    return edges


def find_stub_free_ends(segments: List[Segment], pads: List[Pad],
                        tolerance: float = 0.05,
                        coincidence_tol: float = COINCIDENCE_TOL) -> List[Tuple[float, float, str]]:
    """
    Find the free ends of a segment group (endpoints not connected to other segments or pads).

    Args:
        segments: Connected group of segments
        pads: Pads that might be connected to the segments
        tolerance: PAD-attach tolerance -- an endpoint this close to a pad
            center is a pad attach, not a free end. This is a pad-geometry
            heuristic, deliberately NOT the copper-coincidence tolerance.
        coincidence_tol: endpoint-to-endpoint coincidence (#320, the one
            shared COINCIDENCE_TOL; was exact 1um rounding before, so
            quantization-slop joins now correctly count as connected).

    Returns:
        List of (x, y, layer) tuples for free endpoints
    """
    if not segments:
        return []

    # Cluster endpoints with the ONE shared coincidence primitive (#320), then
    # count SEGMENT INCIDENCES per cluster -- not raw endpoint multiplicity. A
    # segment SHORTER than the tolerance contributes BOTH endpoints to one
    # cluster; raw counting made such a stub tip read "not alone" and the
    # whole stub lost its free end (lumenpnp PD7: fanout sub-grid jog, tip
    # 15um from its junction -> stub invisible to routing -> MPS reorder ->
    # board-wide re-roll). Degree of a cluster = segments with exactly ONE
    # endpoint in it; degree 1 = a chain tip (report the member farthest from
    # the incident segment's far end -- the true tip); degree 2+ = a joint.
    points = []
    for seg in segments:
        points.append((seg.start_x, seg.start_y, seg.layer))
        points.append((seg.end_x, seg.end_y, seg.layer))
    roots = cluster_coincident_points(points, coincidence_tol)
    from collections import defaultdict
    members: Dict[int, List[int]] = defaultdict(list)
    for pi, r in enumerate(roots):
        members[r].append(pi)

    pad_positions = [(p.global_x, p.global_y) for p in pads]

    def _near_pad(x, y):
        return any(abs(x - px) < tolerance and abs(y - py) < tolerance
                   for px, py in pad_positions)

    free_ends = []
    for r, pis in members.items():
        seg_hits: Dict[int, int] = defaultdict(int)
        for pi in pis:
            seg_hits[pi // 2] += 1
        open_segs = [si for si, c in seg_hits.items() if c == 1]
        if len(open_segs) == 1:
            si = open_segs[0]
            seg = segments[si]
            in_cluster = next(pi for pi in pis if pi // 2 == si)
            far = ((seg.end_x, seg.end_y) if in_cluster % 2 == 0
                   else (seg.start_x, seg.start_y))
            # Deterministic tie-break: max() returns the FIRST maximal item,
            # so two equidistant members were resolved by iteration order --
            # i.e. by the order segments happen to sit in the board. Fold the
            # point itself into the key so the winner is geometric.
            x, y, layer = max((points[pi] for pi in pis),
                              key=lambda p: ((p[0]-far[0])**2 + (p[1]-far[1])**2,
                                             p[0], p[1], p[2]))
            if not _near_pad(x, y):
                free_ends.append((round(x, POSITION_DECIMALS),
                                  round(y, POSITION_DECIMALS), layer))
        elif not open_segs and len(members) == 1:
            # The whole group is one sub-tolerance blob: report its raw
            # endpoints (old semantics) rather than nothing.
            seen = set()
            for pi in pis:
                x, y, layer = points[pi]
                key = (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS), layer)
                if key not in seen and not _near_pad(x, y):
                    seen.add(key)
                    free_ends.append(key)

    # Sort: clusters are walked in `members` insertion order, which follows the
    # order segments sit in the board, so the free-end LIST order was
    # board-order dependent. Consumers take [0] as the representative endpoint
    # (see get_net_endpoints), so that leaked straight into routing decisions.
    return sorted(free_ends)


def get_stub_direction(segments: List[Segment], stub_x: float, stub_y: float, stub_layer: str,
                       tolerance: float = 0.05) -> Tuple[float, float]:
    """
    Find the direction a stub is pointing (from pad toward free end).

    Args:
        segments: List of segments for the net
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        (dx, dy): Normalized direction vector pointing in the stub direction
    """
    # Find the segment that has this stub endpoint
    for seg in segments:
        if seg.layer != stub_layer:
            continue
        if getattr(seg, 'graphic', False):
            continue  # board art never defines a stub's direction (#583)

        # Check if start matches stub position
        if abs(seg.start_x - stub_x) < tolerance and abs(seg.start_y - stub_y) < tolerance:
            # Direction from end to start (toward the free end)
            dx = seg.start_x - seg.end_x
            dy = seg.start_y - seg.end_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

        # Check if end matches stub position
        if abs(seg.end_x - stub_x) < tolerance and abs(seg.end_y - stub_y) < tolerance:
            # Direction from start to end (toward the free end)
            dx = seg.end_x - seg.start_x
            dy = seg.end_y - seg.start_y
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                return (dx / length, dy / length)
            return (0, 0)

    # Fallback - no matching segment found
    return (0, 0)


def get_stub_segments(pcb_data: PCBData, net_id: int, stub_x: float, stub_y: float,
                      stub_layer: str, tolerance: float = 0.05) -> List[Segment]:
    """
    Get all segments that form a stub (from free end back to pad).

    Walks backwards from the stub free end through connected segments until
    reaching a pad or the segment chain ends.

    Args:
        pcb_data: PCB data containing segments and pads
        net_id: Net ID of the stub
        stub_x, stub_y: Position of the stub free end
        stub_layer: Layer of the stub
        tolerance: Distance tolerance for matching endpoints

    Returns:
        List of segments forming the stub, ordered from free end to pad
    """
    # Copper GRAPHICS are excluded: they never conduct (#513) and are written
    # back from the original file with their original layer, so a stub walk
    # that picks one up hands immovable board copper to the layer-swap
    # machinery -- which then relayers it in pcb_data only, blinding the
    # obstacle map to the real copper (#583: apple1_aci_card tracks routed
    # through graphic clearance zones after a multipoint collapse "moved"
    # 96 F.Cu graphics to B.Cu).
    net_segments = [s for s in pcb_data.segments
                    if s.net_id == net_id and s.layer == stub_layer
                    and not getattr(s, 'graphic', False)]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    pad_positions = [(p.global_x, p.global_y) for p in net_pads]

    result = []
    visited = set()
    current_x, current_y = stub_x, stub_y

    while True:
        # Find the connected segment at the current position. Among ALL segments
        # whose endpoint is within tolerance, take the CLOSEST -- not the first.
        # Picking the first let the walk "jump across" a short diagonal jog whose
        # far end sits right at the per-axis tolerance (a 0.05mm-per-axis jog == the
        # 0.05mm tolerance): the neighbour beyond the jog matched and the jog segment
        # was skipped, dropping it from the stub. A later layer-swap then relocated
        # only part of the stub and left a ~71um gap that severed the pad
        # (fpga_sdram /D0-). Closest-wins keeps the chain intact -- the true next
        # segment (shared endpoint, distance ~0) always beats a boundary-distance
        # neighbour, while the per-axis gate preserves the original match set.
        found = None
        best_d = None
        for seg in net_segments:
            if id(seg) in visited:
                continue
            if abs(seg.start_x - current_x) < tolerance and abs(seg.start_y - current_y) < tolerance:
                d = math.hypot(seg.start_x - current_x, seg.start_y - current_y)
                if best_d is None or d < best_d:
                    found, best_d, next_x, next_y = seg, d, seg.end_x, seg.end_y
            if abs(seg.end_x - current_x) < tolerance and abs(seg.end_y - current_y) < tolerance:
                d = math.hypot(seg.end_x - current_x, seg.end_y - current_y)
                if best_d is None or d < best_d:
                    found, best_d, next_x, next_y = seg, d, seg.start_x, seg.start_y

        if found is None:
            break

        result.append(found)
        visited.add(id(found))

        # Check if next position is at a pad (we've reached the pad end)
        at_pad = False
        for px, py in pad_positions:
            if abs(next_x - px) < tolerance and abs(next_y - py) < tolerance:
                at_pad = True
                break

        if at_pad:
            break

        current_x, current_y = next_x, next_y

    return result


def get_stub_vias(pcb_data: PCBData, net_id: int, stub_segments: List[Segment],
                  tolerance: float = 0.05) -> List:
    """
    Get vias that are part of a stub (e.g., pad vias from layer switching).

    Finds vias that are located at segment endpoints along the stub path.

    Args:
        pcb_data: PCB data containing vias
        net_id: Net ID of the stub
        stub_segments: List of segments forming the stub (from get_stub_segments)
        tolerance: Distance tolerance for matching positions

    Returns:
        List of Via objects that are part of the stub
    """
    if not stub_segments:
        return []

    # Collect all unique positions along the stub segments only
    # Don't add all pad positions - only positions actually on this stub path
    positions = set()
    for seg in stub_segments:
        positions.add((seg.start_x, seg.start_y))
        positions.add((seg.end_x, seg.end_y))

    # Find vias at these positions
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
    stub_vias = []
    for via in net_vias:
        for px, py in positions:
            if abs(via.x - px) < tolerance and abs(via.y - py) < tolerance:
                stub_vias.append(via)
                break

    return stub_vias


def calculate_stub_via_barrel_length(stub_vias: List, stub_layer: str, pcb_data) -> float:
    """
    Calculate via barrel length for stub vias, using the stub layer as one endpoint.

    The barrel length is from the stub's layer to the pad's layer (where the via
    connects), not the via's full span.

    Args:
        stub_vias: List of Via objects in the stub
        stub_layer: Layer the stub segments are on
        pcb_data: PCBData with stackup info and pads

    Returns:
        Total via barrel length in mm
    """
    if not stub_vias or not pcb_data or not hasattr(pcb_data, 'get_via_barrel_length'):
        return 0.0

    total = 0.0
    for via in stub_vias:
        if not via.layers or len(via.layers) < 2:
            continue

        # Find pad at this via position to determine the other layer
        pad_layer = None
        net_pads = pcb_data.pads_by_net.get(via.net_id, [])
        for pad in net_pads:
            if abs(pad.global_x - via.x) < 0.05 and abs(pad.global_y - via.y) < 0.05:
                # Find copper layer from pad's layers (filter out mask/paste layers and wildcards)
                for layer in pad.layers:
                    if layer.endswith('.Cu') and not layer.startswith('*'):
                        pad_layer = layer
                        break
                break

        if pad_layer and pad_layer != stub_layer:
            # Calculate barrel from stub layer to pad layer
            total += pcb_data.get_via_barrel_length(stub_layer, pad_layer)
        elif stub_layer in via.layers:
            # No pad found or same layer - use via's other layer
            other_layer = via.layers[1] if via.layers[0] == stub_layer else via.layers[0]
            total += pcb_data.get_via_barrel_length(stub_layer, other_layer)

    return total


def drop_off_board_pads(pcb_data: PCBData, pads: List[Pad]) -> List[Pad]:
    """Drop pads whose centre is clearly outside the Edge.Cuts outline (#291).

    Such pads are unreachable -- the edge keep-out blocks every legal cell
    around the outline -- but the keep-out's blocked band only extends a few
    grid cells past the outline bbox, so an edge between TWO off-board pads
    routes entirely in the unblocked space beyond it and ships as board-edge
    DRC (framework_dock: TX/VBUS multipoint copper 20-50mm below the board).
    Filtering them out of the endpoint set means no copper is drawn toward
    them and no search is wasted; the pre-route warning in batch_route names
    them, and the final connectivity reconciliation still reports them as
    failed pads.
    """
    from check_drc import make_off_board_test
    off_board = make_off_board_test(pcb_data.board_info)
    if off_board is None:
        return pads
    return [p for p in pads if not off_board(p.global_x, p.global_y)]


def get_net_endpoints_anchor_split(pcb_data: PCBData, net_id: int,
                                   config: GridRouteConfig,
                                   anchor_a: Tuple[float, float],
                                   anchor_b: Tuple[float, float],
                                   island_points: Optional[set] = None
                                   ) -> Tuple[List, List, Optional[str]]:
    """Endpoint derivation for a KNOWN gap (the scoped net_rescue window).

    get_net_endpoints keeps only the two LARGEST copper groups as the
    route's two sides -- correct on a whole board, wrong inside a cropped
    rescue window: the crop can sever the main trunk into two large
    fragments that then win both slots, silently dropping the small island
    the rescue was aimed at (USB_D_P: the window cut the trunk at the USB
    connector, the A* tried to re-join trunk-to-trunk with its target
    parked in the fence ring, and the BGA ball island was never a target).

    Here the caller knows the gap: every copper endpoint, pad, and via of
    the net is assigned to the nearer of the two anchor points (the gap's
    ends, board mm). Trunk fragments land together on the trunk side no
    matter where the crop cut them; the rescued island lands opposite.
    Rows are get_net_endpoints-shaped: (gx, gy, layer_idx, orig_x, orig_y).

    island_points: when the caller knows the actual copper PARTITION (the
    rescue's union-find components), pass the island side's points as a set
    of (round(x,3), round(y,3)) -- assignment is then by MEMBERSHIP, never
    proximity: island copper -> side B, everything else -> side A. Without
    it, proximity mis-assigns whenever the gap is short and trunk copper
    passes nearer anchor B than the island does (dilemma M_COL4_L: a
    2.48mm gap put trunk cells on BOTH sides, the route joined
    trunk-to-trunk 4.5mm from the gap, and the rescue's verify undid it
    every retry -- the net shipped open though the window route existed).
    """
    from net_queries import expand_pad_layers

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)
    ax, ay = anchor_a
    bx, by = anchor_b
    side_a: List = []
    side_b: List = []

    def _add(x, y, layer_idx, gx, gy):
        if island_points is not None:
            on_island = (round(x, 3), round(y, 3)) in island_points
            (side_b if on_island else side_a).append((gx, gy, layer_idx, x, y))
            return
        da = (x - ax) ** 2 + (y - ay) ** 2
        db = (x - bx) ** 2 + (y - by) ** 2
        (side_a if da <= db else side_b).append((gx, gy, layer_idx, x, y))

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
        _add(seg.start_x, seg.start_y, layer_idx, gx1, gy1)
        if (gx1, gy1) != (gx2, gy2):
            _add(seg.end_x, seg.end_y, layer_idx, gx2, gy2)
    for pad in drop_off_board_pads(pcb_data, pcb_data.pads_by_net.get(net_id, [])):
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        for layer in expand_pad_layers(pad.layers, config.layers):
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                _add(pad.global_x, pad.global_y, layer_idx, gx, gy)
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for layer in config.layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                _add(via.x, via.y, layer_idx, gx, gy)

    if not side_a or not side_b:
        return side_a, side_b, "anchor split produced an empty side"
    return side_a, side_b, None


def get_net_endpoints(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                      use_stub_free_ends: bool = False) -> Tuple[List, List, str]:
    """Deterministic wrapper: sort the endpoint lists by POSITION.

    The candidate lists are built by walking pcb_data.segments / pads, so their
    ORDER follows the order copper happens to sit in the board. Consumers take
    `sources[0]` / `targets[0]` as the net's representative endpoint -- notably
    layer_swap_optimization, which derives each net's src/tgt LAYER from them --
    so a different segment order silently flipped which end is "source".

    That is a real cross-front divergence, not a theoretical one. The GUI adds
    copper to the live board in its own order while the CLI writer emits it in
    another, so the same board (identical segment SET, 1159 of 1159) yields
    different segment ORDER. Measured on eth_tap step 11:

        /Thru Path/ETH0_MDC   run A: src=In1.Cu tgt=F.Cu
                              run B: src=F.Cu  tgt=In1.Cu   (flipped)

    which made the greedy swap-pair test `src1 == tgt2 and tgt1 == src2` match
    for two extra pairs (7 applied vs 5), changed MPS crossing conflicts
    (346 vs 348) and hence the routing order, and left 27 of 295 nets routed
    differently -- 3486 segments vs the CLI's 3428. Every earlier comparison
    used SETS, which is why the order difference stayed invisible.

    Sorting by the endpoint tuple (grid x, grid y, layer index, x, y) makes the
    representative a function of GEOMETRY rather than of board order, so both
    fronts agree regardless of how the copper got there. This is a behaviour
    change for the CLI too (its own order was equally arbitrary) and wants a
    corpus A/B before being treated as settled.
    """
    sources, targets, err = _get_net_endpoints_ordered(
        pcb_data, net_id, config, use_stub_free_ends)
    if sources:
        sources = sorted(sources)
    if targets:
        targets = sorted(targets)
    # Role assignment is left to the derivation below -- see the group-ordering
    # note in _get_net_endpoints_ordered. A blanket "smallest coordinate wins"
    # swap was tried TWICE and regressed both times (CLI 3538 vs GUI 3526/3516):
    # it is deterministic but routing-hostile, because it makes "source" an
    # arbitrary geometric corner and discards the meaning the derivation encodes
    # (route FROM established copper TO the unconnected end).
    return sources, targets, err


def _get_net_endpoints_ordered(pcb_data: PCBData, net_id: int, config: GridRouteConfig,
                               use_stub_free_ends: bool = False) -> Tuple[List, List, str]:
    """
    Find source and target endpoints for a net, considering segments, pads, and existing vias.

    Args:
        pcb_data: PCB data
        net_id: Net ID to find endpoints for
        config: Grid routing configuration
        use_stub_free_ends: If True, use only stub free ends (for diff pairs).
                           If False, use all segment endpoints (for single-ended).

    Returns:
        (sources, targets, error_message)
        - sources: List of (gx, gy, layer_idx, orig_x, orig_y)
        - targets: List of (gx, gy, layer_idx, orig_x, orig_y)
        - error_message: None if successful, otherwise describes why routing can't proceed
    """
    # Import expand_pad_layers locally to avoid circular import
    from net_queries import expand_pad_layers

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = drop_off_board_pads(pcb_data, pcb_data.pads_by_net.get(net_id, []))
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: Multiple segment groups
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) >= 2:
            groups.sort(key=len, reverse=True)

            # If the net also has pads connected to none of the copper groups,
            # those pads are genuine terminals and must not be dropped. A freshly
            # fanned-out stub can fragment into several co-located groups (e.g. a
            # layer swap splitting it across a via); treating two such fragments
            # as the net's two terminals discards the real far pad and collapses
            # the span to ~0. So when unconnected pads exist, use ALL groups'
            # free ends as one side and the unconnected pads as the other -
            # the same shape as the single-group Case 2 below, generalized.
            if use_stub_free_ends:
                all_seg_points = set()
                for g in groups:
                    for seg in g:
                        all_seg_points.add((round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)))
                        all_seg_points.add((round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS)))
                unconnected_pads = []
                for pad in net_pads:
                    px, py = round(pad.global_x, POSITION_DECIMALS), round(pad.global_y, POSITION_DECIMALS)
                    if not any(abs(px - sx) < 0.05 and abs(py - sy) < 0.05 for sx, sy in all_seg_points):
                        unconnected_pads.append(pad)
                if unconnected_pads:
                    sources = []
                    for g in groups:
                        for x, y, layer in find_stub_free_ends(g, net_pads):
                            layer_idx = layer_map.get(layer)
                            if layer_idx is not None:
                                gx, gy = coord.to_grid(x, y)
                                sources.append((gx, gy, layer_idx, x, y))
                    targets = []
                    for pad in unconnected_pads:
                        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                        for layer in expand_pad_layers(pad.layers, config.layers):
                            layer_idx = layer_map.get(layer)
                            if layer_idx is not None:
                                targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))
                    if sources and targets:
                        return sources, targets, None

            # Which group is the SOURCE must not depend on board order.
            # find_connected_groups walks net_segments in order, so groups[0]
            # was simply whichever group owned the earliest segment in the
            # board -- and the GUI and CLI emit copper in different orders.
            # Measured on eth_tap: identical copper, identical endpoint
            # coordinates, but /Thru Path/ETH0_MDC read src=In1.Cu/tgt=F.Cu in
            # one front and src=F.Cu/tgt=In1.Cu in the other. That flip fed the
            # greedy swap-pair test and cascaded into a board-wide re-route.
            #
            # Rank by COPPER, not by coordinates. Routing extends from the
            # established trunk toward the smaller stub: the bigger group offers
            # more anchor points for the search to launch from, and the smaller
            # one is a tighter, better-defined goal. Coordinates enter only as a
            # last-resort tie-break, so the rule stays geometric but is never
            # decided by an arbitrary corner (that variant was tried twice and
            # regressed both fronts).
            def _group_rank(g):
                total = 0.0
                anchor = None
                for _sg in g:
                    total += math.hypot(_sg.end_x - _sg.start_x,
                                        _sg.end_y - _sg.start_y)
                    a = (min((_sg.start_x, _sg.start_y), (_sg.end_x, _sg.end_y)),
                         max((_sg.start_x, _sg.start_y), (_sg.end_x, _sg.end_y)),
                         _sg.layer)
                    if anchor is None or a < anchor:
                        anchor = a
                # more segments first, then more copper, then geometry
                return (-len(g), -total, anchor)

            _ranked = sorted(groups, key=_group_rank)
            source_segs = _ranked[0]
            target_segs = _ranked[1]

            if use_stub_free_ends:
                # For diff pairs: use only stub free ends (tips not connected to pads)
                source_free_ends = find_stub_free_ends(source_segs, net_pads)
                target_free_ends = find_stub_free_ends(target_segs, net_pads)

                sources = []
                for x, y, layer in source_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        sources.append((gx, gy, layer_idx, x, y))

                targets = []
                for x, y, layer in target_free_ends:
                    layer_idx = layer_map.get(layer)
                    if layer_idx is not None:
                        gx, gy = coord.to_grid(x, y)
                        targets.append((gx, gy, layer_idx, x, y))

                # #545 F5: a group with NO usable stub tip (both tips on pads,
                # or a closed loop) left its side EMPTY, the guard below
                # failed, and the pair fell through every later case to
                # "Cannot determine endpoints" -- unroutable even though the
                # group has perfectly good copper. Case 2 has always had this
                # fallback ("Single-ended (or no free tip found): use all
                # segment endpoints"); Case 1 was missing the else limb.
                # Per-side, so a healthy tip on the other side keeps its
                # tight diff-pair semantics.
                for _side, _segs_fb in ((sources, source_segs),
                                        (targets, target_segs)):
                    if _side:
                        continue
                    for seg in _segs_fb:
                        layer_idx = layer_map.get(seg.layer)
                        if layer_idx is not None:
                            gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                            gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                            _side.append((gx1, gy1, layer_idx,
                                          seg.start_x, seg.start_y))
                            if (gx1, gy1) != (gx2, gy2):
                                _side.append((gx2, gy2, layer_idx,
                                              seg.end_x, seg.end_y))
            else:
                # For single-ended: use all segment endpoints
                sources = []
                for seg in source_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for seg in target_segs:
                    layer_idx = layer_map.get(seg.layer)
                    if layer_idx is not None:
                        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                        targets.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                        if (gx1, gy1) != (gx2, gy2):
                            targets.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                # Each group's VIAS are route terminals on every routing layer
                # (PR #534): a via spans the stack, and Case 2 below has
                # exposed vias this way since the original import -- Case 1
                # never did, an oversight, not a decision. The gap bites when
                # a group's segments all sit on layers this run can't route
                # (an F.Cu escape stub + via from a previous fanout step,
                # routed --layers In*): the group's only terminals are on the
                # FORBIDDEN full-stack-appended layer, and the net survives
                # only through the stub-swap rescue ladder (measured on the
                # repro: 18 segments + 2 via-in-pads vs a direct 3-segment
                # route through the existing vias once vias are terminals).
                # Membership mirrors find_connected_groups exactly: grouping
                # unions across layers only where a via coincides with a
                # segment ENDPOINT, so endpoint proximity IS the group's via
                # set (same 0.05 box as Case 2's pad test). Sorted by
                # position: net_vias arrives in board order, which differs
                # between the text parser and pcbnew (the 6515b1c GUI/CLI
                # order-independence class).
                def _group_via_terminals(g):
                    pts = set()
                    for _sg in g:
                        pts.add((_sg.start_x, _sg.start_y))
                        pts.add((_sg.end_x, _sg.end_y))
                    vlist = [v for v in net_vias
                             if any(abs(v.x - px) < 0.05 and abs(v.y - py) < 0.05
                                    for px, py in pts)]
                    vlist.sort(key=lambda v: (round(v.x, POSITION_DECIMALS),
                                              round(v.y, POSITION_DECIMALS)))
                    return vlist
                for _vlist, _side in ((_group_via_terminals(source_segs), sources),
                                      (_group_via_terminals(target_segs), targets)):
                    for _via in _vlist:
                        gx, gy = coord.to_grid(_via.x, _via.y)
                        for _vlayer in config.layers:
                            layer_idx = layer_map.get(_vlayer)
                            if layer_idx is not None:
                                _side.append((gx, gy, layer_idx, _via.x, _via.y))

            if sources and targets:
                return sources, targets, None

    # Case 2: One segment group + unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) == 1:
            # Check if any pad is NOT connected to the segment group.
            # #545 F8: the shared layer-gated, mid-segment-aware, via-in-pad-
            # aware test (the multipoint path's _pad_on_group). The old check
            # compared the pad centre to segment ENDPOINTS on ANY layer within
            # 0.05mm: a B.Cu pad near an In1.Cu endpoint with no via graded
            # "connected", Case 2 skipped it, and Case 4 below then declared
            # the net "already routed" without ever verifying -- a silently
            # dropped net (rescue usually recovered it, loudly). It also
            # missed genuine MID-SEGMENT contact, manufacturing redundant
            # route legs.
            seg_group = groups[0]
            unconnected_pads = [
                pad for pad in net_pads
                if not _pad_touches_copper_group(pad, seg_group, net_vias,
                                                 config.layers)]

            if unconnected_pads:
                # Build source endpoints from the segment group, unconnected pad(s)
                # as target.
                sources = []
                stub_free_ends = (find_stub_free_ends(seg_group, net_pads)
                                  if use_stub_free_ends else [])
                if stub_free_ends:
                    # Diff pairs: launch only from the genuine stub tip (the free
                    # end NOT coincident with a pad). Using all segment endpoints
                    # here would expose the end sitting on the BGA ball pad and the
                    # interior dogleg vertices as candidates - both inside the
                    # keepout - and the closest-pair selector would pick the ball
                    # pad (tightly pitched, perfectly aligned P||N) over the
                    # escaped tip outside the zone, placing the launch inside the
                    # blocked region.
                    for x, y, layer in stub_free_ends:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            gx, gy = coord.to_grid(x, y)
                            sources.append((gx, gy, layer_idx, x, y))
                else:
                    # Single-ended (or no free tip found): use all segment endpoints.
                    for seg in seg_group:
                        layer_idx = layer_map.get(seg.layer)
                        if layer_idx is not None:
                            gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
                            gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
                            sources.append((gx1, gy1, layer_idx, seg.start_x, seg.start_y))
                            if (gx1, gy1) != (gx2, gy2):
                                sources.append((gx2, gy2, layer_idx, seg.end_x, seg.end_y))

                targets = []
                for pad in unconnected_pads:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    # Expand wildcard layers like "*.Cu" to actual routing layers
                    expanded_layers = expand_pad_layers(pad.layers, config.layers)
                    for layer in expanded_layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))

                # Add existing vias as endpoints - they can be routed to on any layer
                # Determine if via is near stub (add to sources) or near unconnected pads (add to targets)
                # Note: All vias are through-hole, connecting ALL routing layers
                for via in net_vias:
                    gx, gy = coord.to_grid(via.x, via.y)
                    # Check if via is near any unconnected pad
                    near_unconnected = False
                    for pad in unconnected_pads:
                        if abs(via.x - pad.global_x) < 0.1 and abs(via.y - pad.global_y) < 0.1:
                            near_unconnected = True
                            break
                    # For diff pairs, if we already have a routable stub-tip source
                    # (sources non-empty from find_stub_free_ends above), don't also
                    # expose the layer-spanning via that anchors the stub to its BGA
                    # ball pad: that via sits inside the keepout and, being a tight
                    # ball-pitch P||N pair, would beat the fanned-out escaped tips in
                    # the closest-pair selector and launch the pair from inside the
                    # blocked zone. Only keep the via when no tip was usable (e.g.
                    # the tip is on an unmapped layer) or it lands on the target pad.
                    if (use_stub_free_ends and sources and not near_unconnected):
                        continue
                    # Add via as endpoint on ALL routing layers (vias are through-hole)
                    for layer in config.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            if near_unconnected:
                                targets.append((gx, gy, layer_idx, via.x, via.y))
                            else:
                                sources.append((gx, gy, layer_idx, via.x, via.y))

                if sources and targets:
                    return sources, targets, None

    # Case 3: No segments, just pads - route between pads
    if len(net_segments) == 0 and len(net_pads) >= 2:
        # #545 F7: a via whose barrel OVERLAPS a pad (via-in-pad escape from
        # a fanout step, or rip leftovers) is genuinely connected to it and
        # spans the whole stack -- offer it as an all-layer terminal on that
        # pad's side. Without this, an SMD pad whose only escape is its
        # via-in-pad was confined to its own layer: the router dropped a
        # SECOND via next to the invisible one, or failed outright on a
        # congested pad layer. Overlap test (via radius), not a proximity
        # box: with no segments on the net, only physical overlap connects.
        # Position-sorted for GUI/CLI order-independence.
        def _pad_via_terminals(pad, side):
            svias = sorted((v for v in net_vias
                            if math.hypot(v.x - pad.global_x, v.y - pad.global_y)
                            <= (v.size or 0) / 2 + 1e-3),
                           key=lambda v: (v.x, v.y))
            for v in svias:
                vgx, vgy = coord.to_grid(v.x, v.y)
                for _layer in config.layers:
                    layer_idx = layer_map.get(_layer)
                    if layer_idx is not None:
                        side.append((vgx, vgy, layer_idx, v.x, v.y))

        # Use first pad as source, rest as targets
        sources = []
        pad = net_pads[0]
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        # Expand wildcard layers like "*.Cu" to actual routing layers
        expanded_layers = expand_pad_layers(pad.layers, config.layers)
        for layer in expanded_layers:
            layer_idx = layer_map.get(layer)
            if layer_idx is not None:
                sources.append((gx, gy, layer_idx, pad.global_x, pad.global_y))
        _pad_via_terminals(pad, sources)

        targets = []
        for pad in net_pads[1:]:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            # Expand wildcard layers like "*.Cu" to actual routing layers
            expanded_layers = expand_pad_layers(pad.layers, config.layers)
            for layer in expanded_layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    targets.append((gx, gy, layer_idx, pad.global_x, pad.global_y))
            _pad_via_terminals(pad, targets)

        if sources and targets:
            return sources, targets, None

    # Case 4: Single segment, check if it connects two pads already
    if len(net_segments) == 1 and len(net_pads) >= 2:
        # Segment already connects pads - nothing to route
        return [], [], "Net has 1 segment connecting pads - already routed"

    # Determine why we can't route
    if len(net_segments) == 0 and len(net_pads) < 2:
        return [], [], f"Net has no segments and only {len(net_pads)} pad(s) - need at least 2 endpoints"
    if len(net_segments) >= 1:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) == 1:
            return [], [], "Net segments are already connected (single group) with no unconnected pads"

    return [], [], f"Cannot determine endpoints: {len(net_segments)} segments, {len(net_pads)} pads"


def get_multipoint_net_pads(*args, **kwargs):
    """Deterministic wrapper: order multi-point terminals by POSITION.

    The terminal list is assembled by walking connected groups / pads, so its
    ORDER follows the order copper sits in the board -- and the GUI and CLI hold
    identical copper in different order. Terminal INDICES are the identity used
    downstream: get_terminal_component_info keys components by them and
    compute_component_mst_edges builds its tree over them, so a relabelling
    changes which MST edge is "pads 0 and 3" versus "pads 3 and 1" and hence
    which leg Phase 1 routes first.

    Measured on eth_tap /Thru Path/ETH0_A1V2 (5 pads): the two fronts produced
    the SAME five points and geometrically the SAME tree (edges 1.7, 2.100001,
    1.600001, 2.0) with terminals 0 and 1 swapped -- (58.7,37.5) and
    (58.7,40.5). Sorting by position makes the labelling a property of the net.
    Geometry is unchanged, so this cannot pick a worse tree; it only fixes which
    equivalent labelling both fronts agree on.
    """
    out = _get_multipoint_net_pads_unordered(*args, **kwargs)
    if not out:
        return out
    try:
        return sorted(out, key=lambda t: (t[3], t[4], t[2]))
    except (IndexError, TypeError):
        return out


def _get_multipoint_net_pads_unordered(
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig
) -> Optional[List[Tuple]]:
    """
    Check if a net has 3+ unconnected endpoints (multi-point net) and return stub endpoints.

    Multi-point nets need special handling - they can't be routed with a single
    A* path. Instead, they need incremental routing: closest pair first, then
    tap into existing track for remaining pads.

    Handles two cases:
    1. No segments, 3+ pads -> return pad positions
    2. 3+ disconnected segment groups -> return stub endpoints (free ends)

    Args:
        pcb_data: PCB data
        net_id: Net ID to check
        config: Grid routing configuration

    Returns:
        List of endpoint info if multi-point: [(gx, gy, layer_idx, orig_x, orig_y, endpoint_obj), ...]
        None if not a multi-point net
    """
    # Import expand_pad_layers locally to avoid circular import
    from net_queries import expand_pad_layers

    coord = GridCoord(config.grid_step)
    layer_map = build_layer_map(config.layers)

    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    # Unnumbered pads (pad_number == "") are paste/thermal artifacts that KiCad
    # does not netlist individually; they must not become routing targets or
    # they show up as failed pads with component_ref '?' (issue #94). They are
    # still copper, so they keep blocking via pcb_data.pads_by_net elsewhere.
    net_pads = [p for p in drop_off_board_pads(pcb_data, pcb_data.pads_by_net.get(net_id, []))
                if (p.pad_number or '').strip() != '']
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: No segments and 3+ pads
    if len(net_segments) == 0 and len(net_pads) >= 3:
        pad_info = []
        for pad in net_pads:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            # Expand wildcard layers like "*.Cu" to actual routing layers
            expanded_layers = expand_pad_layers(pad.layers, config.layers)
            # Use FIRST layer for MST calculation (one entry per physical pad)
            # The tap routing will create targets on ALL layers for through-hole pads
            for layer in expanded_layers:
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    pad_info.append((gx, gy, layer_idx, pad.global_x, pad.global_y, pad))
                    break  # Only one entry per physical pad for MST
            else:
                # Fallback: if no layers matched, use first routing layer
                if config.layers:
                    pad_info.append((gx, gy, 0, pad.global_x, pad.global_y, pad))
        return pad_info if len(pad_info) >= 3 else None

    # Cases 2+3 unified (issue #8 root cause): when the net already has copper
    # (fanout stubs, a previous pass's partial routes), the endpoint set must
    # cover EVERY copper island and EVERY pad with no copper:
    #   - one representative endpoint per connected segment group: its first
    #     free end, or - when a group terminates entirely at pads and has no
    #     dangling ends - a pad on the group / any segment endpoint, so
    #     islands are never silently skipped;
    #   - every pad not connected to any group (point-on-segment test against
    #     each group's copper, not just endpoint coincidence).
    # The old Case 2 returned only group free ends whenever there were >= 3
    # groups, dropping pads with no copper entirely: a 23-pad +3V3 net was
    # detected as "6 pads", routed, and reported fully connected while 11
    # pads had no copper at all.
    if len(net_segments) >= 1:
        groups = find_connected_groups(net_segments, vias=net_vias)

        def _pad_layers(pad):
            return expand_pad_layers(pad.layers, config.layers)

        def _pad_on_group(pad, group) -> bool:
            # Hoisted to _pad_touches_copper_group (#545 F8) so Case 2 shares
            # the same layer-gated, mid-segment-aware, via-in-pad-aware test.
            return _pad_touches_copper_group(pad, group, net_vias, config.layers)

        def _append_pad(endpoint_info, pad):
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            for layer in _pad_layers(pad):
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    endpoint_info.append((gx, gy, layer_idx, pad.global_x, pad.global_y, pad))
                    return
            if config.layers:
                endpoint_info.append((gx, gy, 0, pad.global_x, pad.global_y, pad))

        # Partition pads into per-group connected sets and the unconnected rest
        pads_on_group = [[] for _ in groups]
        unconnected_pads = []
        for pad in net_pads:
            for gi, group in enumerate(groups):
                if _pad_on_group(pad, group):
                    pads_on_group[gi].append(pad)
                    break
            else:
                unconnected_pads.append(pad)

        endpoint_info = []
        for gi, group in enumerate(groups):
            # #545 F4: prefer a VIA of the group as its representative
            # terminal. The old choice, free_ends[0], is the SORTED-SMALLEST
            # free end -- an arbitrary geometric corner pinned to one
            # segment's layer -- which skewed MST edge realization and gave
            # Phase 3 a single-layer target. A via spans the stack: with the
            # stub carrying the real net id (F3), _pad_all_layer_reach sees
            # the via under the terminal and offers it on EVERY layer.
            # Membership mirrors find_connected_groups (a via joins a group
            # at a segment endpoint); position-sorted for GUI/CLI
            # order-independence (net_vias arrives in board order).
            _gpts = set()
            for _sg in group:
                _gpts.add((_sg.start_x, _sg.start_y))
                _gpts.add((_sg.end_x, _sg.end_y))
            _gvias = sorted((v for v in net_vias
                             if any(abs(v.x - px) < 0.05 and abs(v.y - py) < 0.05
                                    for px, py in _gpts)),
                            key=lambda v: (v.x, v.y))
            if _gvias:
                _gv = _gvias[0]
                _vlayer = next(
                    (_sg.layer for _sg in group
                     if (abs(_sg.start_x - _gv.x) < 0.05
                         and abs(_sg.start_y - _gv.y) < 0.05)
                     or (abs(_sg.end_x - _gv.x) < 0.05
                         and abs(_sg.end_y - _gv.y) < 0.05)),
                    group[0].layer)
                layer_idx = layer_map.get(_vlayer)
                if layer_idx is not None:
                    gx, gy = coord.to_grid(_gv.x, _gv.y)
                    endpoint_info.append((gx, gy, layer_idx, _gv.x, _gv.y,
                                          _make_endpoint_stub(_gv.x, _gv.y,
                                                              _vlayer, net_id)))
                    continue
            free_ends = find_stub_free_ends(group, net_pads)
            if free_ends:
                x, y, layer = free_ends[0]
                layer_idx = layer_map.get(layer)
                if layer_idx is not None:
                    endpoint_info.append((
                        coord.to_grid(x, y)[0],
                        coord.to_grid(x, y)[1],
                        layer_idx,
                        x,
                        y,
                        _make_endpoint_stub(x, y, layer, net_id)
                    ))
                    continue
            # No usable free end: represent the island by one of its pads,
            # else by any segment endpoint, so it still gets tied in.
            if pads_on_group[gi]:
                _append_pad(endpoint_info, pads_on_group[gi][0])
            else:
                seg = group[0]
                layer_idx = layer_map.get(seg.layer)
                if layer_idx is not None:
                    gx, gy = coord.to_grid(seg.start_x, seg.start_y)
                    endpoint_info.append((gx, gy, layer_idx, seg.start_x, seg.start_y,
                                          _make_endpoint_stub(seg.start_x, seg.start_y,
                                                              seg.layer, net_id)))

        for pad in unconnected_pads:
            _append_pad(endpoint_info, pad)

        return endpoint_info if len(endpoint_info) >= 3 else None

    return None


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float, str]]:
    """Get free end positions of unrouted net stubs for proximity avoidance.

    Returns list of (x, y, layer) tuples - includes layer for same-layer filtering.
    """
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) < 2:
            continue
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        for group in groups:
            free_ends = find_stub_free_ends(group, net_pads)
            if free_ends:
                # Include all free ends with their layers (for same-layer filtering)
                for fe_x, fe_y, fe_layer in free_ends:
                    stubs.append((fe_x, fe_y, fe_layer))
    return stubs


def segments_intersect(a1: Tuple[float, float], a2: Tuple[float, float],
                       b1: Tuple[float, float], b2: Tuple[float, float]) -> bool:
    """Check if line segment a1-a2 intersects line segment b1-b2.

    Uses the counter-clockwise orientation test for robust intersection detection.
    """
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # Check if segments share an endpoint (not a real crossing)
    eps = 0.001
    for p1 in [a1, a2]:
        for p2 in [b1, b2]:
            if abs(p1[0] - p2[0]) < eps and abs(p1[1] - p2[1]) < eps:
                return False

    return (ccw(a1, b1, b2) != ccw(a2, b1, b2)) and (ccw(a1, a2, b1) != ccw(a1, a2, b2))


def compute_mst_edges(points: List[Tuple[float, float]], use_manhattan: bool = False) -> List[Tuple[int, int, float]]:
    """Compute minimum spanning tree edges between points using Prim's algorithm.

    Args:
        points: List of (x, y) coordinates
        use_manhattan: If True, use Manhattan distance; if False, use Euclidean

    Returns:
        List of (idx_a, idx_b, distance) tuples representing MST edges.
        The edges are in the order they were added to the tree (not sorted by length).
    """
    if len(points) < 2:
        return []
    if len(points) == 2:
        if use_manhattan:
            dist = abs(points[0][0] - points[1][0]) + abs(points[0][1] - points[1][1])
        else:
            dist = math.sqrt((points[0][0] - points[1][0])**2 + (points[0][1] - points[1][1])**2)
        return [(0, 1, dist)]

    # Prim's algorithm, vectorized with numpy. The per-step frontier update and
    # min-selection are O(n) array ops instead of Python loops, so the whole MST
    # is O(n^2) numpy work rather than O(n^2) interpreted work - a large win on
    # big nets (connectors, wide buses). argmin/strict-less-than tie-breaking
    # matches the scalar version (first-index wins).
    n = len(points)
    pts = np.asarray(points, dtype=np.float64)
    xs = pts[:, 0]
    ys = pts[:, 1]

    in_tree = np.zeros(n, dtype=bool)
    nearest = np.zeros(n, dtype=np.intp)
    edges = []

    def _dist_from(idx):
        dx = xs - xs[idx]
        dy = ys - ys[idx]
        if use_manhattan:
            return np.abs(dx) + np.abs(dy)
        return np.sqrt(dx * dx + dy * dy)

    # Start with node 0.
    in_tree[0] = True
    min_dist = _dist_from(0)
    min_dist[0] = np.inf  # node 0 is in the tree

    for _ in range(n - 1):
        # Find the non-tree node closest to the tree (in-tree nodes masked out).
        masked = np.where(in_tree, np.inf, min_dist)
        best_j = int(np.argmin(masked))
        best_dist = float(masked[best_j])
        if not np.isfinite(best_dist):
            break

        in_tree[best_j] = True
        edges.append((int(nearest[best_j]), best_j, best_dist))

        # Pull every remaining node toward the freshly added node where closer.
        d = _dist_from(best_j)
        update = (~in_tree) & (d < min_dist)
        min_dist[update] = d[update]
        nearest[update] = best_j

    return edges


def compute_mst_segments(points: List[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Compute minimum spanning tree segments between points using Prim's algorithm.

    Returns list of (point1, point2) tuples representing MST edges.
    Uses Euclidean distance.
    """
    edges = compute_mst_edges(points, use_manhattan=False)
    return [(points[e[0]], points[e[1]]) for e in edges]


def get_net_mst_segments(pcb_data: PCBData, net_id: int) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Get MST segments representing the routing path for a net.

    For 2-pad nets: returns single segment between pads.
    For 3+ pad nets: returns MST segments connecting all pads.

    Uses stub endpoints if available, otherwise pad positions.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: Has stubs - use stub free ends
    if net_segments:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) >= 2:
            # Multiple stub groups - get free end of each
            points = []
            for group in groups:
                free_ends = find_stub_free_ends(group, net_pads)
                if free_ends:
                    points.append((free_ends[0][0], free_ends[0][1]))
                else:
                    # Fallback to centroid
                    pts = []
                    for seg in group:
                        pts.append((seg.start_x, seg.start_y))
                        pts.append((seg.end_x, seg.end_y))
                    # math.fsum, not the builtin sum() (#493): Python 3.12 switched sum() to
                    # compensated summation, so a float sum differs by an ULP or two between
                    # KiCad's bundled python and the system one -- enough to flip a decision
                    # made from it and make routing depend on the interpreter.
                    cx = math.fsum(p[0] for p in pts) / len(pts)
                    cy = math.fsum(p[1] for p in pts) / len(pts)
                    points.append((cx, cy))
            return compute_mst_segments(points)

    # Case 2: No stubs, just pads - use pad positions
    if len(net_pads) >= 2:
        points = [(pad.global_x, pad.global_y) for pad in net_pads]
        return compute_mst_segments(points)

    return []


def get_net_routing_endpoints(pcb_data: PCBData, net_id: int) -> List[Tuple[float, float]]:
    """
    Get the two routing endpoints for a net, for MPS conflict detection.

    This handles multiple cases:
    1. Two stub groups -> use stub centroids
    2. One stub group + unconnected pads -> use stub centroid + pad centroid
    3. No stubs, just pads -> use pad positions

    Returns list of (x, y) positions, typically 2 for source and target.
    """
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    net_pads = pcb_data.pads_by_net.get(net_id, [])
    net_vias = [v for v in pcb_data.vias if v.net_id == net_id]

    # Case 1: Multiple stub groups - use their centroids
    if len(net_segments) >= 2:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) >= 2:
            # centroids[:2] took the first two groups in BOARD order, so a net
            # with 3+ stub groups handed MPS a different pair of representative
            # points per front. That feeds unit_distances, which orders every
            # round: measured on eth_tap, /Thru Path/ETH0_A3V3 came out
            # dist=6.075 on one front and 6.897 on the other with an IDENTICAL
            # conflict graph. Rank by COPPER (more segments, then more length,
            # geometry last) -- the same rule used for source/target selection
            # -- so "the two main stub groups" is a property of the net.
            def _grp_rank(g):
                total = 0.0
                anchor = None
                for _s in g:
                    total += math.hypot(_s.end_x - _s.start_x,
                                        _s.end_y - _s.start_y)
                    a = (min((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)),
                         max((_s.start_x, _s.start_y), (_s.end_x, _s.end_y)),
                         _s.layer)
                    if anchor is None or a < anchor:
                        anchor = a
                return (-len(g), -total, anchor)

            centroids = []
            for group in sorted(groups, key=_grp_rank):
                points = []
                for seg in group:
                    points.append((seg.start_x, seg.start_y))
                    points.append((seg.end_x, seg.end_y))
                if points:
                    cx = math.fsum(p[0] for p in points) / len(points)
                    cy = math.fsum(p[1] for p in points) / len(points)
                    centroids.append((cx, cy))
            return centroids[:2]

    # Case 2: One stub group + pads - find unconnected pads
    if len(net_segments) >= 1 and len(net_pads) >= 1:
        groups = find_connected_groups(net_segments, vias=net_vias)
        if len(groups) == 1:
            # Get stub centroid
            group = groups[0]
            seg_points = set()
            for seg in group:
                seg_points.add((round(seg.start_x, POSITION_DECIMALS), round(seg.start_y, POSITION_DECIMALS)))
                seg_points.add((round(seg.end_x, POSITION_DECIMALS), round(seg.end_y, POSITION_DECIMALS)))

            stub_pts = []
            for seg in group:
                stub_pts.append((seg.start_x, seg.start_y))
                stub_pts.append((seg.end_x, seg.end_y))
            stub_cx = math.fsum(p[0] for p in stub_pts) / len(stub_pts)
            stub_cy = math.fsum(p[1] for p in stub_pts) / len(stub_pts)

            # Find unconnected pads
            unconnected_pads = []
            for pad in net_pads:
                pad_pos = (round(pad.global_x, POSITION_DECIMALS), round(pad.global_y, POSITION_DECIMALS))
                connected = False
                for sp in seg_points:
                    if abs(pad_pos[0] - sp[0]) < 0.05 and abs(pad_pos[1] - sp[1]) < 0.05:
                        connected = True
                        break
                if not connected:
                    unconnected_pads.append(pad)

            if unconnected_pads:
                # Compute centroid of unconnected pads
                pad_cx = math.fsum(p.global_x for p in unconnected_pads) / len(unconnected_pads)
                pad_cy = math.fsum(p.global_y for p in unconnected_pads) / len(unconnected_pads)
                return [(stub_cx, stub_cy), (pad_cx, pad_cy)]

    # Case 3: No stubs, just pads - use first pad and centroid of rest
    if len(net_segments) == 0 and len(net_pads) >= 2:
        first_pad = net_pads[0]
        other_pads = net_pads[1:]
        other_cx = math.fsum(p.global_x for p in other_pads) / len(other_pads)
        other_cy = math.fsum(p.global_y for p in other_pads) / len(other_pads)
        return [(first_pad.global_x, first_pad.global_y), (other_cx, other_cy)]

    return []


def find_connected_segment_positions(pcb_data: PCBData, start_x: float, start_y: float,
                                      net_id: int, tolerance: float = None,
                                      layer: str = None) -> set:
    """
    Find all segment endpoint positions connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a set of (x, y) pos_key tuples for all endpoints in the connected chain.

    #369 A10: `tolerance` is now HONORED -- the old BFS joined endpoints by
    exact pos_key (1um) despite advertising 0.1, so a soft joint (the #320
    2-10um endpoint-gap class) stopped a polarity/target-swap relabel
    mid-trace, leaving one physical trace carrying both nets. Endpoints are
    clustered at `tolerance` (default COINCIDENCE_TOL, the engine's canonical
    coincidence radius). Cross-layer traversal now also requires a same-net
    VIA at the junction: with layer=None the old walk chained different-layer
    copper that merely shared XY, pulling co-located other-layer stubs into
    the relabel.

    Args:
        layer: Optional layer name to filter segments. When specified, only
               segments on this layer are considered.
    """
    if tolerance is None:
        tolerance = COINCIDENCE_TOL
    # Get all segments for this net (optionally filtered by layer)
    if layer:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id and s.layer == layer]
    else:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
    start_key = pos_key(start_x, start_y)
    if not net_segments:
        return {start_key}

    # Cluster endpoint positions within tolerance (spatial hash, per-axis gate)
    cell = max(tolerance, 1e-6)
    _buckets: dict = {}
    _reps: list = []

    def _cluster_of(x, y):
        cx, cy = int(math.floor(x / cell)), int(math.floor(y / cell))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for ci in _buckets.get((cx + dx, cy + dy), ()):
                    rx, ry = _reps[ci]
                    if abs(rx - x) <= tolerance and abs(ry - y) <= tolerance:
                        return ci
        _reps.append((x, y))
        _buckets.setdefault((cx, cy), []).append(len(_reps) - 1)
        return len(_reps) - 1

    # Nodes are (cluster, layer); segments connect their endpoint nodes.
    adjacency: dict = {}
    keys_at: dict = {}
    for seg in net_segments:
        a = (_cluster_of(seg.start_x, seg.start_y), seg.layer)
        b = (_cluster_of(seg.end_x, seg.end_y), seg.layer)
        adjacency.setdefault(a, []).append(b)
        adjacency.setdefault(b, []).append(a)
        keys_at.setdefault(a, set()).add(pos_key(seg.start_x, seg.start_y))
        keys_at.setdefault(b, set()).add(pos_key(seg.end_x, seg.end_y))

    # Cross-layer bridges only where a same-net via sits at the cluster
    if layer is None:
        via_clusters = {_cluster_of(v.x, v.y) for v in pcb_data.vias
                        if v.net_id == net_id}
        by_cluster: dict = {}
        for node in adjacency:
            by_cluster.setdefault(node[0], []).append(node)
        for ci, nodes in by_cluster.items():
            if ci in via_clusters and len(nodes) > 1:
                for i, na in enumerate(nodes):
                    for nb in nodes[i + 1:]:
                        adjacency[na].append(nb)
                        adjacency[nb].append(na)

    # BFS: seed every layer's node at the start cluster (the start is a pad
    # position; its own copper joins the stack there)
    start_ci = _cluster_of(start_x, start_y)
    queue = [n for n in adjacency if n[0] == start_ci]
    visited = set(queue)
    out = {start_key}
    while queue:
        node = queue.pop()
        out |= keys_at.get(node, set())
        for nb in adjacency.get(node, []):
            if nb not in visited:
                visited.add(nb)
                queue.append(nb)
    return out


def find_connected_segments(pcb_data: PCBData, start_x: float, start_y: float,
                            net_id: int) -> List:
    """
    Find all segments connected to a starting position for a given net.

    Uses BFS to traverse the segment chain from the starting position.
    Returns a list of Segment objects in the connected stub chain.

    This differs from find_connected_segment_positions in that it returns
    the actual segment objects, not just their positions. This allows
    unambiguous identification of which segments belong to which stub chain,
    even when two chains share a common endpoint position.
    """
    # Get all segments for this net
    net_segments = [s for s in pcb_data.segments if s.net_id == net_id]

    # Build adjacency: position -> list of segments touching that position
    pos_to_segments = {}
    for seg in net_segments:
        start = pos_key(seg.start_x, seg.start_y)
        end = pos_key(seg.end_x, seg.end_y)
        if start not in pos_to_segments:
            pos_to_segments[start] = []
        if end not in pos_to_segments:
            pos_to_segments[end] = []
        pos_to_segments[start].append(seg)
        pos_to_segments[end].append(seg)

    # BFS from start position, collecting segments
    start_key = pos_key(start_x, start_y)
    visited_positions = set()
    visited_segments = set()
    queue = [start_key]

    while queue:
        pos = queue.pop(0)
        if pos in visited_positions:
            continue
        visited_positions.add(pos)

        for seg in pos_to_segments.get(pos, []):
            if id(seg) in visited_segments:
                continue
            visited_segments.add(id(seg))

            # Add the other endpoint of this segment to the queue
            other_end = pos_key(seg.end_x, seg.end_y) if pos_key(seg.start_x, seg.start_y) == pos else pos_key(seg.start_x, seg.start_y)
            if other_end not in visited_positions:
                queue.append(other_end)

    # Return the actual segment objects
    return [s for s in net_segments if id(s) in visited_segments]

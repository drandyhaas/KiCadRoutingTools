"""
Connectivity Checker - Verify that tracks form fully connected routes from source to target pads.
"""
from __future__ import annotations

import sys
import os
import argparse
import math
import fnmatch
from typing import List, Dict, Set, Tuple, Optional
from collections import defaultdict
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad, PCBData, Zone
from net_queries import expand_pad_layers


# Fragment blindness: in the strict-fragment view a track-to-track joint
# only counts when the copper overlaps by at least this depth. The grading
# epsilon (1e-6) credits quantization-level lenses KiCad's exact geometry
# rejects -- run 6's VCC3V3 graded 25/27 pads connected while KiCad saw 7
# islands. 0.05mm is the writer grid quantum / find_stub_free_ends pad-attach
# tolerance: an overlap that deep cannot be an FP round-trip phantom.
# Bounds: must stay < 0.15 (the pinned soft-joint lens,
# tests/test_component_multipoint.py) and > ~0.02 (COINCIDENCE_TOL).
STRICT_JOINT_OVERLAP = 0.05

# Zone layers already reported as not-a-copper-layer-of-this-board.
# check_net_connectivity runs once PER NET, so without this the same phantom
# layer would be announced fifty-odd times on one board.
_WARNED_PHANTOM_ZONE_LAYERS: Set[str] = set()


def point_in_polygon(x: float, y: float, polygon: List[Tuple[float, float]]) -> bool:
    """Check if a point (x, y) is inside a polygon using ray casting algorithm.

    Args:
        x, y: Point coordinates
        polygon: List of (x, y) vertices defining the polygon

    Returns:
        True if point is inside the polygon
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        # Check if ray from point crosses this edge
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i

    return inside


def points_in_polygon_mask(xs, ys, polygon):
    """Vectorized point_in_polygon over coordinate arrays -- the same even-odd
    ray cast, comparison for comparison (pure compares and one division, no
    pow), so the mask is byte-identical to the scalar per point. Sweep item 7
    (#625 follow-up): the zone-credit scan ran the scalar per net point per
    zone per connectivity check."""
    import numpy as np
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    n = len(polygon)
    if n < 3 or not len(xs):
        return np.zeros(xs.shape, dtype=bool)
    P = np.asarray(polygon, dtype=np.float64)
    xi, yi = P[:, 0], P[:, 1]
    xj, yj = np.roll(P[:, 0], 1), np.roll(P[:, 1], 1)
    inside = np.zeros(xs.shape, dtype=bool)
    # Chunk the (points x vertices) broadcast to bound temporaries.
    _B = max(1, 4_000_000 // n)
    for s in range(0, len(xs), _B):
        px = xs[s:s + _B, None]
        py = ys[s:s + _B, None]
        cond = (yi[None, :] > py) != (yj[None, :] > py)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (xj - xi)[None, :] * (py - yi[None, :]) / (yj - yi)[None, :] + xi[None, :]
            crossing = cond & (px < xint)
        inside[s:s + _B] = (np.count_nonzero(crossing, axis=1) & 1).astype(bool)
    return inside


def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """Check if a net name matches any of the given patterns (fnmatch style)."""
    for pattern in patterns:
        if fnmatch.fnmatch(name, pattern):
            return True
    return False


from geometry_utils import UnionFind


def points_match(x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.02) -> bool:
    """Check if two points are within tolerance."""
    return abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance


class SpatialIndex:
    """Grid-based spatial index for fast proximity queries."""

    def __init__(self, cell_size: float = 1.0):
        self.cell_size = cell_size
        self.grid = defaultdict(list)  # (gx, gy, layer) -> list of (x, y, point_id, size)

    def _cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(x // self.cell_size), int(y // self.cell_size))

    def add(self, x: float, y: float, layer: str, point_id: int, size: float):
        gx, gy = self._cell(x, y)
        self.grid[(gx, gy, layer)].append((x, y, point_id, size))

    def query_nearby(self, x: float, y: float, layer: str, radius: float):
        """Return all points within radius on the same layer."""
        gx, gy = self._cell(x, y)
        # Check neighboring cells based on radius
        cells_to_check = int(radius / self.cell_size) + 1
        results = []
        for dx in range(-cells_to_check, cells_to_check + 1):
            for dy in range(-cells_to_check, cells_to_check + 1):
                for pt in self.grid.get((gx + dx, gy + dy, layer), []):
                    results.append(pt)
        return results


class SegmentIndex:
    """Grid-based spatial index for segments."""

    def __init__(self, cell_size: float = 1.0):
        self.cell_size = cell_size
        self.grid = defaultdict(list)  # (gx, gy, layer) -> list of (seg, seg_start_id)

    def _cells_for_segment(self, seg) -> List[Tuple[int, int]]:
        """Return all grid cells that a segment passes through."""
        x1, y1, x2, y2 = seg.start_x, seg.start_y, seg.end_x, seg.end_y
        gx1, gy1 = int(min(x1, x2) // self.cell_size), int(min(y1, y2) // self.cell_size)
        gx2, gy2 = int(max(x1, x2) // self.cell_size), int(max(y1, y2) // self.cell_size)
        cells = []
        for gx in range(gx1, gx2 + 1):
            for gy in range(gy1, gy2 + 1):
                cells.append((gx, gy))
        return cells

    def add(self, seg, seg_start_id: int):
        for gx, gy in self._cells_for_segment(seg):
            self.grid[(gx, gy, seg.layer)].append((seg, seg_start_id))

    def query_at(self, x: float, y: float, layer: str):
        """Return all segments that might contain point (x, y) on layer."""
        gx, gy = int(x // self.cell_size), int(y // self.cell_size)
        return self.grid.get((gx, gy, layer), [])

    def query_near(self, x: float, y: float, layer: str, radius: float = None):
        """Like query_at but over the cell neighbourhood, deduped - for
        proximity tests whose tolerance (a track width) can cross a cell
        edge. `radius` widens the ring: a big via's barrel credit reaches
        (via_size + track_width)/2, which exceeds one 1.0mm cell (the
        silent-miss review finding)."""
        gx, gy = int(x // self.cell_size), int(y // self.cell_size)
        r = 1
        if radius is not None and radius > self.cell_size:
            r = int(radius // self.cell_size) + 1
        seen = set()
        out = []
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for item in self.grid.get((gx + dx, gy + dy, layer), ()):
                    if id(item[0]) not in seen:
                        seen.add(id(item[0]))
                        out.append(item)
        return out


def point_on_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float, tolerance: float = 0.02) -> bool:
    """Check if point (px, py) lies on the segment from (x1, y1) to (x2, y2) within tolerance.

    This detects T-junctions where a track endpoint meets another track mid-segment.
    """
    # Quick bounding box check first (with tolerance)
    min_x, max_x = (min(x1, x2) - tolerance, max(x1, x2) + tolerance)
    min_y, max_y = (min(y1, y2) - tolerance, max(y1, y2) + tolerance)
    if not (min_x <= px <= max_x and min_y <= py <= max_y):
        return False

    # Calculate distance from point to line segment
    # Vector from segment start to end
    dx = x2 - x1
    dy = y2 - y1

    # Handle degenerate case (zero-length segment)
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-10:
        return points_match(px, py, x1, y1, tolerance)

    # Project point onto line, clamped to segment
    t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))

    # Closest point on segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    # Check if point is within tolerance of the closest point on segment
    dist_sq = (px - closest_x) ** 2 + (py - closest_y) ** 2
    return dist_sq <= tolerance * tolerance


def _pad_bounding_radius(pad) -> float:
    """Radius of a circle guaranteed to contain the pad's copper (half the
    rect diagonal). Over-approximates on purpose: used only as a cheap
    spatial prefilter before the exact shape test below."""
    return math.hypot(pad.size_x, pad.size_y) / 2


def _pad_credit_disc(pad):
    """(cx, cy, reach): the disc inside which a track counts as landing on this
    pad's copper.

    Run-7 finding, and the reason this is not just `min(size)/4` about the
    anchor: a CUSTOM pad is stored as a symmetric box around its anchor big
    enough to enclose the real primitives, and the anchor is frequently a
    corner of them rather than their centre. One MOSFET tab measured 4.57 x
    5.01mm of real copper modelled as a 9.13 x 10.01mm box whose centre sat
    3.4mm away from it -- so a 2.28mm credit disc sat in empty space, and any
    track passing near it was unioned onto the pad. Two islands of that net
    each "reached" the phantom disc, and the net graded CONNECTED while its
    copper was in two pieces. The router then skipped it as already routed.

    When the real primitive polygons are available, use their bbox. They are
    the actual copper.
    """
    polys = getattr(pad, 'polygons', None)
    if polys:
        xs = [pt[0] for poly in polys for pt in poly]
        ys = [pt[1] for poly in polys for pt in poly]
        if xs and ys:
            w, h = max(xs) - min(xs), max(ys) - min(ys)
            return ((max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0,
                    (min(w, h) / 4.0) if (w > 0 and h > 0) else 0.05)
    if pad.size_x and pad.size_y:
        return pad.global_x, pad.global_y, min(pad.size_x, pad.size_y) / 4.0
    return pad.global_x, pad.global_y, 0.05


def _pads_copper_touch(pi: Pad, pj: Pad, tolerance: float = 0.05) -> bool:
    """Shape-accurate test that two pads' copper physically touches/overlaps
    (edge-to-edge gap <= tolerance).

    The old sum-of-bounding-circle-radii test graded 1.5x0.7 rect pads at
    1.0 mm pitch (real edge gap 0.3 mm) as "connected", so a castellated
    module's spare power pads with NO copper on them at all passed the
    connectivity check — and the multipoint router, which trusts this
    checker's graph for its terminal grouping (#317), never routed them
    (issue #346). Cross-sample each pad's real perimeter against the other
    pad's exact distance function (rect/roundrect/circle/oval +
    rect_rotation + custom polygons), same geometry check_drc grades with.
    """
    from check_drc import point_to_pad_distance, _pad_perimeter_points

    def _degenerate(p):
        # _EndpointStub terminals (and any pad-like without a shape/size)
        # are points, not outlines.
        return not getattr(p, 'shape', None) or (p.size_x <= 0 and p.size_y <= 0)

    if _degenerate(pi) and _degenerate(pj):
        return math.hypot(pi.global_x - pj.global_x,
                          pi.global_y - pj.global_y) <= tolerance
    if _degenerate(pi):
        return point_to_pad_distance(pi.global_x, pi.global_y, pj) <= tolerance
    if _degenerate(pj):
        return point_to_pad_distance(pj.global_x, pj.global_y, pi) <= tolerance
    # Both directions: one pad fully inside the other still hits (the inner
    # pad's perimeter samples are inside the outer copper, distance 0).
    # Sweep item 5 (#625 follow-up): the broadcast proxy nominates candidate
    # samples (with a generous band around the tolerance, since its multiply
    # kernel rounds up to 1 ULP apart from the scalar's **2); the verdict on
    # each candidate is the scalar itself, so the boolean is byte-identical.
    # This feeds the multipoint router's terminal grouping (#317/#346), so
    # exactness matters here, not just speed.
    from check_drc import _pad_perimeter_array, _pad_dist_d2_proxy
    import numpy as np
    hi2 = (tolerance * (1 + 1e-9)) ** 2
    for a, b in ((pi, pj), (pj, pi)):
        xs, ys, pts = _pad_perimeter_array(a)
        if not len(xs):
            continue
        d2 = _pad_dist_d2_proxy(xs, ys, b)
        for i in np.nonzero(d2 <= hi2)[0]:
            if point_to_pad_distance(pts[i][0], pts[i][1], b) <= tolerance:
                return True
    return False


def _net_pads_connected_by_overlap(pads: List[Pad], copper_layers, tolerance: float = 0.05) -> bool:
    """True if every pad of the net touches the others through overlapping
    copper alone (no track needed).

    Castellated modules represent each pin as a through-hole pad plus an SMD
    pad at the same spot; such a net has pads but no segments yet is fully
    connected, so it must not be reported as unrouted (issue #92).
    """
    if len(pads) < 2:
        return True
    parent = list(range(len(pads)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def shares_copper(pi, pj):
        if pi.drill > 0 or pj.drill > 0:
            return True  # through-hole spans all copper layers
        li = set(expand_pad_layers(pi.layers, copper_layers))
        lj = set(expand_pad_layers(pj.layers, copper_layers))
        return bool(li & lj)

    for i in range(len(pads)):
        for j in range(i + 1, len(pads)):
            pi, pj = pads[i], pads[j]
            if not shares_copper(pi, pj):
                continue
            # Cheap bounding-circle prefilter, then exact shape overlap —
            # the circle alone falsely joined adjacent edge pads (#346).
            reach = _pad_bounding_radius(pi) + _pad_bounding_radius(pj) + tolerance
            dx = pi.global_x - pj.global_x
            dy = pi.global_y - pj.global_y
            if dx * dx + dy * dy <= reach * reach and \
                    _pads_copper_touch(pi, pj, tolerance):
                parent[find(i)] = find(j)
    return len({find(i) for i in range(len(pads))}) == 1


def bare_pad_nets(pcb_data, exclude_net_ids=None,
                  include_partial: bool = True) -> Dict[int, List[Pad]]:
    """Pads that would enter a pour BARE: nets with >=2 pads and ZERO copper
    (all their pads), plus -- with ``include_partial`` -- the stranded pads of
    partially-routed zone-less nets (per check_net_connectivity's
    disconnected_pads). Hardened against the false positives
    run_connectivity_check already handles: overlap-connected pad stacks
    (castellated modules, issue #92), KiCad auto-named 'unconnected-*'
    no-connects, and multi-board outlines where no single outline holds two
    of the net's pads.

    Consumer (run-6 A5): the plane scripts' pour gate. A pad that enters the
    pour unconnected has its escape channel consumed by the tap-via carpet,
    which is not rippable copper -- measured on test-board run 6, five bare
    QFN rail pads (on PARTIALLY-routed rails) oscillated 6-9 oracle joins
    across five post-pour repair attempts and never closed.
    Returns {net_id: bare pads}.
    """
    exclude = set(exclude_net_ids or ())
    pads_by_net = pcb_data.pads_by_net
    segments_by_net: Dict[int, list] = {}
    for s in pcb_data.segments:
        segments_by_net.setdefault(s.net_id, []).append(s)
    vias_by_net: Dict[int, list] = {}
    for v in pcb_data.vias:
        vias_by_net.setdefault(v.net_id, []).append(v)
    zones_by_net: Dict[int, list] = {}
    for z in (getattr(pcb_data, 'zones', None) or []):
        if getattr(z, 'net_id', None) is not None:
            zones_by_net.setdefault(z.net_id, []).append(z)
    copper_layers = pcb_data.board_info.copper_layers or ['F.Cu', 'B.Cu']
    outlines = getattr(pcb_data.board_info, 'board_outlines', None) or []
    bare: Dict[int, List[Pad]] = {}
    for net_id, net_info in pcb_data.nets.items():
        if not net_info.name or net_id in exclude:
            continue
        pads = pads_by_net.get(net_id) or []
        if len(pads) < 2:
            continue
        if net_id in zones_by_net:
            continue        # zone-owning nets are the pour's own business
        if net_info.name.lower().startswith('unconnected-'):
            continue
        has_copper = net_id in segments_by_net or net_id in vias_by_net
        if not has_copper:
            if _net_pads_connected_by_overlap(pads, copper_layers):
                continue
            if len(outlines) > 1:
                counts: Dict[int, int] = {}
                for p in pads:
                    for i, poly in enumerate(outlines):
                        if point_in_polygon(p.global_x, p.global_y, poly):
                            counts[i] = counts.get(i, 0) + 1
                            break
                if not any(c >= 2 for c in counts.values()):
                    continue
            bare[net_id] = list(pads)
            continue
        if not include_partial:
            continue
        # Partially-routed net: its STRANDED pads are just as bare to the
        # pour (the run-6 case -- rails with copper elsewhere, five pads
        # never reached).
        try:
            res = check_net_connectivity(
                net_id, segments_by_net.get(net_id, []),
                vias_by_net.get(net_id, []), pads, [], pcb_data=pcb_data)
        except Exception:
            continue
        locs = res.get('disconnected_pads') or []
        if not locs:
            continue
        stranded = []
        for loc in locs:
            lx, ly = loc[0], loc[1]
            for p in pads:
                if abs(p.global_x - lx) < 1e-3 and abs(p.global_y - ly) < 1e-3:
                    if p not in stranded:
                        stranded.append(p)
                    break
        if stranded:
            bare[net_id] = stranded
    return bare


def net_copper_fragments(net_id, segments, vias, pads, zones=None,
                         pcb_data=None, tolerance: float = 0.02) -> Dict:
    """Strict-fragment census: one strict_fragments=True graph build +
    UnionFind replay. A fragment = a connected component owning >=1 track
    segment (graphic=False) or via; pads ride along (a pad-only component is
    not a fragment -- that is `unrouted`'s domain). Consumers: the
    filter_already_routed fragment gate (#578, ported from the placement branch)
    and route.py's summary sweep -- which is how a plain --nets call finally
    SEES a net KiCad holds in pieces.

    Returns {'fragments': int, 'padless_fragments': int,
             'fragment_anchors': [(x, y)], 'zone_blob_fallback': bool}.
    """
    res = check_net_connectivity(net_id, segments, vias, pads, zones or [],
                                 tolerance=tolerance, return_graph=True,
                                 pcb_data=pcb_data, strict_fragments=True)
    graph = res.get('graph') or {}
    uf = UnionFind()
    for a, b in graph.get('edges') or []:
        uf.union(a, b)
    seg_list = list(segments)
    via_repr = graph.get('via_index_repr') or {}
    pad_repr = graph.get('pad_index_repr') or {}
    frag_anchor: Dict[int, Tuple[float, float]] = {}
    for i, seg in enumerate(seg_list):
        if getattr(seg, 'graphic', False):
            continue    # graphics never conduct (#513 item 6)
        root = uf.find(2 * i)
        if root not in frag_anchor:
            frag_anchor[root] = (seg.start_x, seg.start_y)
    for vi, pid in via_repr.items():
        root = uf.find(pid)
        if root not in frag_anchor:
            v = list(vias)[vi] if vi < len(list(vias)) else None
            frag_anchor[root] = ((v.x, v.y) if v is not None
                                 else (0.0, 0.0))
    pad_roots = {uf.find(pid) for pid in pad_repr.values()}
    padless = sum(1 for r in frag_anchor if r not in pad_roots)
    return {'fragments': len(frag_anchor),
            'padless_fragments': padless,
            'fragment_anchors': sorted(frag_anchor.values()),
            'zone_blob_fallback': bool(graph.get('zone_blob_fallback'))}


def raster_unconnected(pcb_data, net_names=None, tolerance: float = 0.02):
    """[(net, (x, y, layer, kind), (x, y, layer, kind)), ...] -- the same
    contract as ``kicad_oracle.kicad_unconnected``, computed from OUR
    fill-aware model with **no KiCad at all** (issue #648's third branch:
    exact-fill -> kicad-cli -> raster).

    The primitive already existed: ``check_net_connectivity`` builds per-net
    connected components over segments u vias u pads u zone credit -- it is
    what prints "Disconnected components: N". This packages it as a LINK
    SOURCE: one spanning link per extra component, drawn between the two
    components' true nearest approach, endpoints tagged with their layer.

    What it is FOR, stated precisely, because it is not a drop-in replacement
    for KiCad:

      * It is DETERMINISTIC and dependency-free, where kicad-cli's threaded
        connectivity is not (#490 measured 103/65/92 on identical input).
      * It is the only source at all on a machine with no KiCad -- where
        ``oracle_reconnect`` otherwise returns available=False and every
        oracle leg is a no-op rather than degraded (the cloud image, #650).
      * It CANNOT find the class #659 is about. A micro-gap that our model
        credits and KiCad rejects -- a custom pad modelled as its bounding
        rectangle, two pads 20 nm apart -- is by construction invisible to
        the model doing the crediting. Only KiCad sees those. Anything that
        needs that class must keep asking KiCad.

    `net_names` limits the scan (names, not ids); None scans every net with
    copper. Pad-less debris components are included, exactly as kicad-cli
    reports them -- the caller decides weld vs delete (classify_unconnected_link).
    """
    import math
    from geometry_utils import UnionFind
    want = set(net_names) if net_names is not None else None
    out = []
    segs_by_net, vias_by_net = {}, {}
    for s in pcb_data.segments:
        segs_by_net.setdefault(s.net_id, []).append(s)
    for v in pcb_data.vias:
        vias_by_net.setdefault(v.net_id, []).append(v)
    for nid, net in (pcb_data.nets or {}).items():
        name = getattr(net, 'name', None)
        if not name or (want is not None and name not in want):
            continue
        segs = segs_by_net.get(nid, [])
        vias = vias_by_net.get(nid, [])
        if not segs and not vias:
            continue
        pads = pcb_data.pads_by_net.get(nid, [])
        zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
                 if z.net_id == nid]
        r = check_net_connectivity(nid, segs, vias, pads, zones,
                                   tolerance=tolerance, return_graph=True,
                                   pcb_data=pcb_data)
        g = r.get('graph') or {}
        uf = UnionFind()
        for a, b in g.get('edges', []) or []:
            uf.union(a, b)
        # Points per component, tagged with layer + kind, so a link's
        # endpoints carry what the consumers key on.
        comp = {}
        for i, s in enumerate(segs):
            if getattr(s, 'graphic', False):
                continue          # art is not a routable endpoint (#337)
            root = uf.find(2 * i)
            comp.setdefault(root, []).append((s.start_x, s.start_y, s.layer, 'track'))
            comp.setdefault(root, []).append((s.end_x, s.end_y, s.layer, 'track'))
        vrep = g.get('via_index_repr') or {}
        for j, v in enumerate(vias):
            rep = vrep.get(j)
            if rep is None:
                continue
            comp.setdefault(uf.find(rep), []).append((v.x, v.y, None, 'via'))
        prep = g.get('pad_index_repr') or {}
        for k, pid in (prep.items() if isinstance(prep, dict) else []):
            pad = pads[k] if isinstance(k, int) and k < len(pads) else None
            if pad is None:
                continue
            lyr = next((L for L in (pad.layers or []) if L.endswith('.Cu')), None)
            comp.setdefault(uf.find(pid), []).append(
                (pad.global_x, pad.global_y, lyr, 'pad'))
        comp = {k: v for k, v in comp.items() if v}
        if len(comp) < 2:
            continue
        # Spanning links: each component after the first joins to whichever
        # ALREADY-JOINED component it comes nearest to -- a tree, not a
        # clique, so N components yield N-1 links exactly as a ratsnest does.
        roots = sorted(comp, key=lambda rr: (-len(comp[rr]), str(rr)))
        joined, rest = [roots[0]], roots[1:]
        for _ in range(len(rest)):
            best = None
            for rr in rest:
                for jr in joined:
                    for (ax, ay, al, ak) in comp[rr]:
                        for (bx, by, bl, bk) in comp[jr]:
                            d = math.hypot(ax - bx, ay - by)
                            if best is None or d < best[0]:
                                best = (d, rr, (ax, ay, al, ak), (bx, by, bl, bk))
            if best is None:
                break
            _d, rr, pa, pb = best
            out.append((name, pb, pa))
            joined.append(rr)
            rest.remove(rr)
    return out


def net_dead_copper(pcb_data, net_id, segments, vias, pads, zones=None):
    """The net's copper that reaches NO pad and NO zone of its own net (#659).

    Read-only twin of remove_orphan_islands' verdict: it answers "which of
    this net's copper is dead" without mutating anything, so callers deciding
    whether ROUTING can help (the #578 fragment gate, the fragment sweep)
    ask exactly the question the late sweep will act on.

    Deliberately the AUTHORITATIVE (permissive) connectivity graph, not the
    strict-fragment view. "Strictly pad-less" is a different question and
    conflating them is a real bug: a fragment can miss a pad's copper by a
    hair and still be the net's actual route -- the phantom split #578
    exists to catch -- and diverting that away from the router would ship the
    open it was built to close. Copper is dead only when the same graph that
    grades the board says nothing ties it to a pad or the net's pour.

    Graphics clusters (#337 immutable art) are never reported: they are not
    ours to delete, so they are not "dead copper" for a caller's purposes.

    Returns (dead_segments, dead_vias) as object lists.
    """
    from geometry_utils import UnionFind
    segments = list(segments)
    vias = list(vias)
    if not pads or (not segments and not vias):
        return [], []
    r = check_net_connectivity(net_id, segments, vias, pads, zones or [],
                               return_graph=True, pcb_data=pcb_data)
    g = r.get('graph') or {}
    uf = UnionFind()
    for a, b in g.get('edges', []) or []:
        uf.union(a, b)
    live = {uf.find(x) for x in (g.get('pad_index_repr') or {}).values()} | \
           {uf.find(x) for x in (g.get('zone_index_repr') or {}).values()}
    via_repr = g.get('via_index_repr') or {}
    graphic_roots = set()
    net_graphics = [s for s in segments if getattr(s, 'graphic', False)]
    for i, s in enumerate(segments):
        if getattr(s, 'graphic', False):
            graphic_roots.add(uf.find(2 * i))
    # A cluster ABUTTING the art is joined to it in copper even though the
    # graph does not conduct through it (#513). Measured on openstint: the
    # via bridging /A-'s copper to its graphic looks pad-less to us and is
    # load-bearing to KiCad -- deleting it took that board from 0 unconnected
    # items to 2. Same rule remove_orphan_islands' _touches_graphic applies.
    import math as _m

    def _abuts_art_seg(s):
        from geometry_utils import segment_to_segment_distance
        for g in net_graphics:
            if s.layer != g.layer:
                continue
            if segment_to_segment_distance(
                    s.start_x, s.start_y, s.end_x, s.end_y,
                    g.start_x, g.start_y, g.end_x, g.end_y) \
                    <= (s.width + g.width) / 2 + 1e-6:
                return True
        return False

    def _abuts_art_via(v):
        for g in net_graphics:
            dx, dy = g.end_x - g.start_x, g.end_y - g.start_y
            L2 = dx * dx + dy * dy
            tt = (max(0.0, min(1.0, ((v.x - g.start_x) * dx
                                     + (v.y - g.start_y) * dy) / L2))
                  if L2 else 0.0)
            if _m.hypot(v.x - (g.start_x + tt * dx),
                        v.y - (g.start_y + tt * dy)) \
                    <= v.size / 2.0 + g.width / 2 + 1e-6:
                return True
        return False

    if net_graphics:
        for i, s in enumerate(segments):
            if uf.find(2 * i) not in graphic_roots and _abuts_art_seg(s):
                graphic_roots.add(uf.find(2 * i))
        for j, v in enumerate(vias):
            rep = via_repr.get(j)
            if rep is not None and uf.find(rep) not in graphic_roots \
                    and _abuts_art_via(v):
                graphic_roots.add(uf.find(rep))
    dead_s = [s for i, s in enumerate(segments)
              if uf.find(2 * i) not in live
              and uf.find(2 * i) not in graphic_roots]
    dead_v = [v for j, v in enumerate(vias)
              if via_repr.get(j) is not None
              and uf.find(via_repr[j]) not in live
              and uf.find(via_repr[j]) not in graphic_roots]
    return dead_s, dead_v


def classify_unconnected_link(pcb_data, net_id, pt_a, pt_b,
                              kind_a=None, kind_b=None, radius: float = 0.35):
    """Classify the two ends of a KiCad-reported unconnected link (#659).

    KiCad reporting a link says the net is open; it does NOT say what repair
    is called for, and the three answers are different operations:

      'live'    -- the endpoint's copper cluster reaches a pad or the net's
                   own pour. A link between two live ends is a GENUINE open:
                   welding it is the fix.
      'padless' -- the cluster reaches no pad and no zone. It is rip/reroute
                   DEBRIS: it conducts nothing, so welding it adds dead metal
                   and deletion is the fix (remove_orphan_islands).
      'graphic' -- the cluster is net-tagged copper ART (#337), which is
                   immutable: neither weld nor delete applies, and the board's
                   author has to convert it to pads/tracks (#513).
      'unknown' -- no copper of this net located at the point (a shape the
                   parser models differently, e.g. a filled gr_circle whose
                   reported position is its centre).

    Measured over the recorded corpus, the KiCad-only-open links on zone-less
    signal nets split 9 padless / 36 graphic / 6 live -- so treating the whole
    class as "weld it" is wrong for 45 of 51 links.

    Returns (class_a, class_b). `kind_a`/`kind_b` are KiCad's own item kinds
    ('pad', 'zone', 'track', 'via') when known: a pad or zone item IS live by
    definition and needs no geometric search.
    """
    import math
    from geometry_utils import UnionFind
    segs = [s for s in pcb_data.segments if s.net_id == net_id]
    vias = [v for v in pcb_data.vias if v.net_id == net_id]
    pads = pcb_data.pads_by_net.get(net_id, [])
    zones = [z for z in (getattr(pcb_data, 'zones', None) or [])
             if z.net_id == net_id]
    if not pads or (not segs and not vias):
        return 'unknown', 'unknown'
    r = check_net_connectivity(net_id, segs, vias, pads, zones,
                               return_graph=True, pcb_data=pcb_data)
    g = r.get('graph') or {}
    uf = UnionFind()
    for a, b in g.get('edges', []) or []:
        uf.union(a, b)
    live = {uf.find(x) for x in (g.get('pad_index_repr') or {}).values()} | \
           {uf.find(x) for x in (g.get('zone_index_repr') or {}).values()}
    via_repr = g.get('via_index_repr') or {}
    graphic_roots = set()
    for i, s in enumerate(segs):
        if getattr(s, 'graphic', False):
            graphic_roots.add(uf.find(2 * i))

    def _one(pt, kind):
        if kind in ('pad', 'zone'):
            return 'live'
        px, py = pt[0], pt[1]
        best, bd = None, radius
        for i, s in enumerate(segs):
            dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
            L2 = dx * dx + dy * dy
            t = (max(0.0, min(1.0, ((px - s.start_x) * dx
                                    + (py - s.start_y) * dy) / L2))
                 if L2 else 0.0)
            d = math.hypot(px - (s.start_x + t * dx), py - (s.start_y + t * dy))
            if d < bd:
                best, bd = i, d
        if best is not None:
            root = uf.find(2 * best)
        else:
            # No segment: the endpoint can be a BARE via -- a failed reroute
            # keeps the barrel and drops every track around it.
            vb, vd = None, None
            for j, v in enumerate(vias):
                if via_repr.get(j) is None:
                    continue
                d = math.hypot(px - v.x, py - v.y)
                if d <= max(radius, v.size / 2.0 + 0.05) and (vd is None or d < vd):
                    vb, vd = j, d
            if vb is None:
                return 'unknown'
            root = uf.find(via_repr[vb])
        if root in graphic_roots:
            return 'graphic'
        return 'live' if root in live else 'padless'

    return _one(pt_a, kind_a), _one(pt_b, kind_b)


def _point_in_pad(px: float, py: float, pad: Pad, margin: float = 0.0) -> bool:
    """True if (px, py) lies within `pad`'s copper outline (+margin).

    EXACT shape test via check_drc.point_to_pad_distance (custom-pad
    polygons, roundrect corners, rotation). The old rectangle fallback
    treated a CUSTOM pad as its full bounding box: bitaxe Q2.5 (a 9.1x10mm
    FET tab) then phantom-connected a track endpoint 4mm from its real
    copper, and collapse_strict_redundant trusted the graph and removed the
    genuinely load-bearing bridge segment (0708d regression). A cheap bbox
    prefilter keeps the hot path fast."""
    dx = px - pad.global_x
    dy = py - pad.global_y
    reach = max(pad.size_x, pad.size_y) / 2 + margin
    if dx * dx + dy * dy > reach * reach * 2:  # bbox-diagonal prefilter
        return False
    from check_drc import point_to_pad_distance
    return point_to_pad_distance(px, py, pad) <= margin


def make_real_fill_validator(pcb_data, net_id, margin: float = 0.25,
                             shared_buckets: Optional[dict] = None):
    """Factory for a zone-credit validator: validate(x, y, layer) -> True iff
    a `margin`-radius disc at (x, y) is clear of every FOREIGN copper item on
    `layer` -- i.e. real zone fill can provably exist there.

    The zone credit in check_net_connectivity otherwise trusts the pour
    OUTLINE, which over-credits: a pad inside the outline but in a
    clearance-carved pocket grades 'plane-connected', and a REMOVAL pass
    trusting that grade deletes the pad's genuinely load-bearing tap (the
    bitaxe Q2 shredded-stub opens). Removal gates pass this validator so
    outline credit only applies where fill can actually flow. Conservative
    by construction: a False only DENIES credit, which blocks a removal.

    Foreign geometry is bucketed per layer on first use (1mm cells), so each
    validate() is O(local).

    The buckets are built at a FIXED reach (_BUCKET_REACH_MARGIN, the largest
    margin any caller uses) so validator instances with different margins can
    share them: pass the same `shared_buckets` dict to each (sweep_dead_ends
    builds two validators per net -- credit margin 0.25, anchor margin 0.02 --
    and with margin baked into the reach each one re-bucketed every foreign
    via / segment / pad on the board). A larger-reach bucket is a superset,
    so any query margin <= the build reach stays exact. The dict is scoped to
    the CALLER (never cached on pcb_data): copper mutates between sweeps and
    a stale bucket would over-permit."""
    import math as _m
    _BUCKET_REACH_MARGIN = 0.25
    assert margin <= _BUCKET_REACH_MARGIN + 1e-9, \
        f"validator margin {margin} exceeds bucket reach {_BUCKET_REACH_MARGIN}"
    _shared = shared_buckets if shared_buckets is not None else {}
    _buckets = {}

    def _build(layer):
        _ck = (net_id, layer)
        b = _shared.get(_ck)
        if b is not None:
            return b
        b = {}

        def _add(x1, y1, x2, y2, reach, obj):
            for bx in range(int(min(x1, x2) - reach) - 1,
                            int(max(x1, x2) + reach) + 2):
                for by in range(int(min(y1, y2) - reach) - 1,
                                int(max(y1, y2) + reach) + 2):
                    b.setdefault((bx, by), []).append(obj)
        for v in pcb_data.vias:
            if v.net_id != net_id:
                _add(v.x, v.y, v.x, v.y, v.size / 2 + _BUCKET_REACH_MARGIN,
                     ('c', v.x, v.y, v.size / 2))
        for s in pcb_data.segments:
            if s.net_id != net_id and s.layer == layer:
                _add(s.start_x, s.start_y, s.end_x, s.end_y,
                     s.width / 2 + _BUCKET_REACH_MARGIN, ('s', s))
        for plist in pcb_data.pads_by_net.values():
            for p in plist:
                if p.net_id == net_id:
                    continue
                if p.drill <= 0 and layer not in p.layers \
                        and '*.Cu' not in p.layers:
                    continue
                r = max(p.size_x, p.size_y) / 2
                _add(p.global_x, p.global_y, p.global_x, p.global_y,
                     r + _BUCKET_REACH_MARGIN, ('p', p))
        _shared[_ck] = b
        return b

    def validate(x, y, layer):
        b = _buckets.get(layer)
        if b is None:
            b = _buckets[layer] = _build(layer)
        from check_drc import point_to_pad_distance
        for obj in b.get((int(x), int(y)), ()):
            kind = obj[0]
            if kind == 'c':
                _, cx, cy, r = obj
                if _m.hypot(x - cx, y - cy) < r + margin:
                    return False
            elif kind == 's':
                s = obj[1]
                dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
                L2 = dx * dx + dy * dy
                t = max(0.0, min(1.0, ((x - s.start_x) * dx +
                                       (y - s.start_y) * dy) / L2)) if L2 else 0.0
                if _m.hypot(x - (s.start_x + t * dx),
                            y - (s.start_y + t * dy)) < s.width / 2 + margin:
                    return False
            else:
                if point_to_pad_distance(x, y, obj[1]) < margin:
                    return False
        return True

    return validate


def net_break_within_outlines(pcb_data, result):
    """Multi-board files (#479, len42_filter2): with >=2 outer Edge.Cuts
    outlines, a net is BROKEN only when some single outline's pads span more
    than one connected component -- pads split across outlines are joined by
    a board-to-board connector at assembly, never by copper. Returns
    (broken, disconnected_pads): on single-outline boards this is exactly
    (not result['connected'], result['disconnected_pads']); on multi-outline
    boards the pad list is filtered to outlines that are split internally.
    Shared by the grading path here and the routers' completion bookkeeping
    (filter_already_routed, route.py's authoritative reporting), so "needs
    routing" and "grades incomplete" agree on multi-board files."""
    if result.get('connected'):
        return False, []
    dps = list(result.get('disconnected_pads') or [])
    outs = getattr(pcb_data.board_info, 'board_outlines', None) or []
    if len(outs) < 2:
        return True, dps

    def _which(px, py):
        for _i, _poly in enumerate(outs):
            if point_in_polygon(px, py, _poly):
                return _i
        return None

    comps = {}
    for _loc, _comp in (result.get('pad_components') or {}).items():
        comps.setdefault(_which(_loc[0], _loc[1]), set()).add(_comp)
    split = {oi for oi, s in comps.items() if len(s) > 1}
    if not split:
        return False, []
    kept = [p for p in dps if _which(p[0], p[1]) in split]
    return True, (kept or dps)


def net_pad_pairs_within_outlines(pcb_data, result, pads):
    """Outline-aware pad-pair credit for route.py's #409 tallies. Pairs are
    counted WITHIN each outer Edge.Cuts outline -- pads split across outlines
    are joined by a board-to-board connector at assembly, never by copper
    (#479), so a cross-outline gap is neither a routable pair nor an open one.
    Returns (pairs_total, pairs_connected). Single-outline boards degenerate
    to (P - 1, P - num_components) with the clamps the tally always applied:
    num_components == 0 (no pad visible to the union-find -- the padless-copper
    answer) grades as fully connected, and pads invisible to the union-find
    can never push connected above total. The same clamps apply per outline."""
    num_pads = len(pads)
    k = result.get('num_components') or 0
    outs = getattr(pcb_data.board_info, 'board_outlines', None) or []
    if len(outs) < 2:
        total = max(0, num_pads - 1)
        conn = total if k <= 0 else min(total, max(0, num_pads - k))
        return total, conn

    def _which(px, py):
        for _i, _poly in enumerate(outs):
            if point_in_polygon(px, py, _poly):
                return _i
        return None

    # Pads per outline (a pad outside every outline is its own None bucket,
    # consistent with net_break_within_outlines), and the union-find's
    # distinct components per outline.
    pads_per: Dict[Optional[int], int] = {}
    for _p in pads:
        _oi = _which(_p.global_x, _p.global_y)
        pads_per[_oi] = pads_per.get(_oi, 0) + 1
    comps_per: Dict[Optional[int], Set[int]] = {}
    for _loc, _comp in (result.get('pad_components') or {}).items():
        comps_per.setdefault(_which(_loc[0], _loc[1]), set()).add(_comp)
    total = 0
    conn = 0
    for _oi, _n in pads_per.items():
        _t = _n - 1
        _k = len(comps_per.get(_oi, ()))
        total += _t
        conn += _t if _k <= 0 else min(_t, max(0, _n - _k))
    return total, conn


def reconcile_status_line(recon: dict) -> str:
    """One-line verdict for the KiCad-refill cross-check (#654).

    The cross-check used to speak only when it CHANGED something, so a reader
    could not tell "KiCad ran and agreed" from "KiCad never ran" -- both were
    silence, and both ended in the same `ALL NETS FULLY CONNECTED!`. That
    distinction matters most exactly when the copper-grading model is under
    suspicion of fill-model artifacts, which is the whole reason the
    cross-check exists.

    `recon` keys: state (ran | did_not_run | disabled | not_applicable |
    error), links, reclass, kicad_only, detail. Kept a pure function of that
    dict so the wording is testable without a board.
    """
    st = recon.get('state')
    if st == 'ran':
        n = recon.get('links', 0)
        parts = [f"ran, {n} KiCad link(s)"]
        if recon.get('reclass'):
            parts.append(f"reclassified {recon['reclass']} net(s) CONNECTED")
        if recon.get('kicad_only'):
            parts.append(f"flagged {recon['kicad_only']} net(s) KiCad-only "
                         "UNCONNECTED")
        if not recon.get('reclass') and not recon.get('kicad_only'):
            parts.append("agrees with copper grading")
        return "KiCad refill cross-check: " + ", ".join(parts)
    if st == 'did_not_run':
        return ("KiCad refill cross-check: DID NOT RUN "
                f"({recon.get('detail', 'unknown')}) -- zone-covered nets are "
                "graded by the copper model ALONE; KiCad has not agreed with "
                "them")
    if st == 'disabled':
        return ("KiCad refill cross-check: DISABLED "
                f"({recon.get('detail', '')}) -- copper grading only")
    if st == 'error':
        return ("KiCad refill cross-check: UNAVAILABLE "
                f"({recon.get('detail', '')}) -- copper grading only")
    return ("KiCad refill cross-check: not applicable "
            f"({recon.get('detail', 'board has no zones')})")


def check_net_connectivity(net_id: int, segments: List[Segment], vias: List[Via],
                           pads: List[Pad], zones: List[Zone] = None,
                           tolerance: float = 0.02,
                           verbose: bool = False,
                           return_graph: bool = False,
                           zone_credit_validator=None,
                           pcb_data=None,
                           strict_fragments: bool = False) -> Dict:
    """Check connectivity for a single net.

    Args:
        net_id: The net ID being checked
        segments: Track segments belonging to this net
        vias: Vias belonging to this net
        pads: Pads belonging to this net
        zones: Zones (power planes) belonging to this net
        tolerance: Connection tolerance in mm
        verbose: If True, include detailed debug info
        strict_fragments: the PLANNER's view (fragmentation blindness).
            Three credit rules tighten -- pad points lose the generic 0.4mm
            proximity radius (the EXACT pad rules #195/#89/#346/#479 stay
            live), endpoint/via caps must overlap real copper by
            STRICT_JOINT_OVERLAP instead of the grading epsilon, and zone
            credit requires the fill MODEL's word (no legacy blob where a
            model exists; no model at all keeps the blob and sets the graph's
            'zone_blob_fallback' flag). Default False preserves the grading
            semantics for every existing caller. This is deliberately a THIRD
            strictness level between the permissive grader and the
            COINCIDENCE_TOL-clamped removal twin: the removal twin would
            split the soft joints test_component_multipoint pins as one
            component.

    Returns dict with:
        - connected: bool - whether all pads are connected
        - num_components: int - number of disconnected components
        - pad_components: dict mapping pad location to component id
        - disconnected_pads: list of pad locations not connected to the main component
        - debug_info: dict with detailed component info (when verbose=True)
    """
    if zones is None:
        zones = []
    _zone_blob_fallback = False
    uf = UnionFind()
    # Record every union as an edge in parallel with uf (which still drives this
    # call's result unchanged). A caller can then re-evaluate connectivity with
    # some segments EXCLUDED -- without rebuilding the expensive spatial graph --
    # by dropping edges that touch the excluded segments' endpoint points. Segment
    # i's two endpoints are the first points created, so they are point ids 2i and
    # 2i+1 (see the segment loop below). Only PAD roots decide connectivity, so an
    # excluded segment's now-isolated points are harmless. Used by the cleanup
    # loops (prune_grazing_segments et al.) that otherwise call this per candidate
    # removal -- O(net_size x candidates) -> O(net_size + candidates) (#263).
    edges = []

    def _union(a, b):
        uf.union(a, b)
        edges.append((a, b))

    # Detect all copper layers from segments, vias, and pads
    copper_layer_set = set()
    for seg in segments:
        if seg.layer.endswith('.Cu'):
            copper_layer_set.add(seg.layer)
    for via in vias:
        if via.layers:
            for layer in via.layers:
                if layer.endswith('.Cu'):
                    copper_layer_set.add(layer)
    for pad in pads:
        for layer in pad.layers:
            # Skip wildcards like "*.Cu" - they don't represent actual layers
            if layer.endswith('.Cu') and not layer.startswith('*'):
                copper_layer_set.add(layer)
    # A zone layer is cross-checked against the board's OWN copper layers, not
    # merely tested for a '.Cu' suffix (#run11). route_planes once accepted a
    # net:layer pair as a layer name and wrote (layer "GND:B.Cu") into the zone:
    # KiCad refuses such a file outright, but "GND:B.Cu".endswith('.Cu') is
    # True, so the phantom layer joined this census, expand_pad_layers spread
    # through-hole pads onto it, and the pour was credited 70/70 when the honest
    # figure was 69/70. Every checker downstream of this function inherited that
    # lie. GUARD: only when the board declares copper layers -- a board whose
    # stackup failed to parse degrades to the old behaviour rather than having
    # every zone rejected.
    _board_copper = set(getattr(getattr(pcb_data, 'board_info', None),
                                'copper_layers', None) or ())
    for zone in zones:
        if not zone.layer.endswith('.Cu'):
            continue
        if _board_copper and zone.layer not in _board_copper:
            if zone.layer not in _WARNED_PHANTOM_ZONE_LAYERS:
                _WARNED_PHANTOM_ZONE_LAYERS.add(zone.layer)
                print(f"WARNING: zone on layer '{zone.layer}', which is NOT a "
                      f"copper layer of this board "
                      f"({', '.join(sorted(_board_copper))}). KiCad cannot open "
                      f"this file; its copper is NOT credited here. A net:layer "
                      f"pair passed to route_planes --plane-layers writes exactly "
                      f"this.")
            continue
        copper_layer_set.add(zone.layer)

    # Sort layers: F.Cu first, then In*.Cu in order, then B.Cu last
    def layer_sort_key(layer):
        if layer == 'F.Cu':
            return (0, 0)
        elif layer == 'B.Cu':
            return (2, 0)
        elif layer.startswith('In') and layer.endswith('.Cu'):
            try:
                num = int(layer[2:-3])  # Extract number from 'InX.Cu'
                return (1, num)
            except ValueError:
                return (1, 999)
        else:
            return (3, 0)

    all_copper_layers = sorted(copper_layer_set, key=layer_sort_key)
    if not all_copper_layers:
        all_copper_layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Fallback

    # Collect all points with their actual coordinates and size info
    # Each point: (x, y, layer, point_id, size, type, extra_info)
    all_points = []
    point_id = 0
    point_info = {}  # Maps point_id to descriptive info

    # Add segment endpoints
    for seg_idx, seg in enumerate(segments):
        if getattr(seg, 'graphic', False):
            # Net-tagged copper GRAPHICS (gr_line/gr_rect fingers etc.) are
            # DRC-real obstacles but NOT electrical connections: KiCad's
            # connectivity engine never credits them, so copper that only
            # joins "through" a graphic is electrically split (#513 item 6 --
            # opengammakit shipped 5 nets open while this model read them
            # connected through the card-edge finger graphics). Allocate the
            # two point ids (callers rely on "segment i -> ids 2i / 2i+1")
            # but keep the points out of the join graph entirely.
            point_id += 2
            continue
        start_id = point_id
        all_points.append((seg.start_x, seg.start_y, seg.layer, start_id, seg.width))
        point_info[start_id] = ('segment_start', seg_idx, seg.layer, seg.start_x, seg.start_y)
        point_id += 1
        end_id = point_id
        all_points.append((seg.end_x, seg.end_y, seg.layer, end_id, seg.width))
        point_info[end_id] = ('segment_end', seg_idx, seg.layer, seg.end_x, seg.end_y)
        point_id += 1
        # Connect segment's own endpoints
        _union(start_id, end_id)

    # Add vias - they connect all layers at one location
    via_repr_id = {}        # via_idx -> a representative point id (layers all unioned)
    via_copper_layers = {}  # via_idx -> set of copper layers the via spans
    for via_idx, via in enumerate(vias):
        if via.layers:
            if 'F.Cu' in via.layers and 'B.Cu' in via.layers:
                via_layers = all_copper_layers
            else:
                via_layers = via.layers
        else:
            via_layers = all_copper_layers

        via_size = getattr(via, 'size', 0.6)  # Default via size if not available
        via_ids = []
        for layer in via_layers:
            all_points.append((via.x, via.y, layer, point_id, via_size))
            point_info[point_id] = ('via', via_idx, layer, via.x, via.y)
            via_ids.append(point_id)
            point_id += 1
        # Connect all via layers together
        for vid in via_ids[1:]:
            _union(via_ids[0], vid)
        if via_ids:
            via_repr_id[via_idx] = via_ids[0]
            via_copper_layers[via_idx] = {l for l in via_layers if l.endswith('.Cu')}

    # Add pads (use a reasonable default size for pads)
    # Through-hole pads span multiple layers and should connect them (like vias)
    pad_ids = []
    pad_locations = []
    copper_layers = set(all_copper_layers)
    pad_repr_id = {}      # pad_idx -> a representative point id (its layers are all unioned)
    pad_copper_layers = {}  # pad_idx -> set of copper layers the pad actually occupies
    for pad_idx, pad in enumerate(pads):
        # Expand wildcard layers like "*.Cu" to actual copper layers
        expanded_layers = expand_pad_layers(pad.layers, all_copper_layers)
        this_pad_ids = []  # Track all layer points for this pad
        this_pad_layers = set()
        for layer in expanded_layers:
            if layer not in copper_layers:
                continue
            # Generic point-point tolerance is size/4 (below). The pad's REAL
            # size here radially over-credited large/elongated pads (a 9mm
            # custom tab pulled anything within 2.3mm of its CENTER into the
            # net -- the 0708d collapse regression); every large-pad case the
            # real size was for is now covered by the EXACT rules below
            # (#195 endpoint-in-pad, #89 via-in-pad, #346 pad-pad overlap),
            # so pad points keep only the small flat tolerance.
            # strict view: the flat proximity radius is exactly the
            # credit that merged fragments a pad never touches -- the exact
            # rules below still connect every REAL pad attachment.
            pad_size = 0.0 if strict_fragments else 0.4
            all_points.append((pad.global_x, pad.global_y, layer, point_id, pad_size))
            point_info[point_id] = ('pad', pad_idx, layer, pad.global_x, pad.global_y, pad.component_ref)
            pad_ids.append(point_id)
            pad_locations.append((pad.global_x, pad.global_y, layer, pad.component_ref))
            this_pad_ids.append(point_id)
            this_pad_layers.add(layer)
            point_id += 1
        # Connect all layers of this pad together (through-hole pads act like vias)
        for pid in this_pad_ids[1:]:
            _union(this_pad_ids[0], pid)
        if this_pad_ids:
            pad_repr_id[pad_idx] = this_pad_ids[0]
            pad_copper_layers[pad_idx] = this_pad_layers

    # Connect points through zones (power planes)
    # All points on the same layer that are inside the same zone are connected
    zone_repr_id = {}  # zone_idx -> a representative point id credited to the zone
    for zone_idx, zone in enumerate(zones):
        zone_layer = zone.layer
        # Validator-parity fill model (KiCad semantics): when the caller
        # supplies pcb_data, zone credit joins two items ONLY when they touch
        # the SAME fill COMPONENT -- the outline-blob union credited pruned
        # islands (a dogbone via's pinched ring deep in a BGA field graded
        # plane-connected while open at fab; 7 phantom balls on ottercast,
        # and step8's repair skipped them for the same reason). Points the
        # model cannot answer (no scipy, oversize zone, out of bbox) keep the
        # legacy blob credit.
        _zm = None
        try:
            if pcb_data is not None:
                from plane_fill_model import get_zone_model
                _zm = get_zone_model(pcb_data, zone)
            else:
                # No pcb_data (removal-pass call sites): a model built
                # earlier for this same zone OBJECT still applies.
                from plane_fill_model import lookup_zone_model
                _zm = lookup_zone_model(zone)
        except Exception:
            _zm = None
        # Find all points on this zone's layer
        points_on_layer = [(x, y, layer, pid, size) for x, y, layer, pid, size in all_points
                           if layer == zone_layer]

        # Find which points are inside the zone polygon (item 7: one
        # vectorized ray cast for the whole layer's points, then the model /
        # validator logic only on the inside ones -- mask byte-identical).
        points_in_zone = []   # legacy blob (no model verdict)
        points_by_comp = {}   # fill component id -> [pid]
        _mask = points_in_polygon_mask([p[0] for p in points_on_layer],
                                       [p[1] for p in points_on_layer],
                                       zone.polygon)
        for _in_poly, (x, y, layer, pid, size) in zip(_mask, points_on_layer):
            if _in_poly:
                # Removal gates pass a fill validator (#outline-over-credit,
                # bitaxe Q2): outline membership only counts where real fill
                # can provably exist. GRADING callers pass None and keep the
                # permissive outline credit.
                if zone_credit_validator is not None and \
                        not zone_credit_validator(x, y, zone.layer):
                    continue
                _c = _zm.query_component(x, y, size=size) if _zm is not None else None
                if _c is None:
                    if strict_fragments and _zm is not None:
                        # strict view: a model EXISTS but cannot vouch for
                        # this point (out of bbox) -> no credit. This is the
                        # pruned-island over-credit class.
                        continue
                    if strict_fragments:
                        # no model at all (scipy absent / oversize zone):
                        # keep the legacy blob but DISCLOSE it -- callers
                        # print a degraded-credit note off the graph flag.
                        _zone_blob_fallback = True
                    points_in_zone.append(pid)
                elif _c > 0:
                    points_by_comp.setdefault(_c, []).append(pid)
                # _c == 0: fill provably does not touch this point -> no credit

        # Connect points that share a fill component
        for _pids in points_by_comp.values():
            for pid in _pids[1:]:
                _union(_pids[0], pid)
        # Legacy blob: union together and onto the PLANE component (largest),
        # preserving old permissive semantics for unanswerable points.
        _main = _zm.largest_component() if _zm is not None else 0
        if points_in_zone:
            for pid in points_in_zone[1:]:
                _union(points_in_zone[0], pid)
            if _main in points_by_comp:
                _union(points_by_comp[_main][0], points_in_zone[0])
        # Representative point for this zone's copper component, so a graph
        # consumer (plane_component_oracle) can tell which component IS the
        # plane vs. a floating same-net island. With the fill model this is
        # the LARGEST component (the plane proper), never an island.
        if _main in points_by_comp:
            zone_repr_id[zone_idx] = points_by_comp[_main][0]
        elif points_in_zone:
            zone_repr_id[zone_idx] = points_in_zone[0]
        elif points_by_comp:
            zone_repr_id[zone_idx] = next(iter(points_by_comp.values()))[0]

    # Connect all points that are within tolerance on the same layer
    # Use spatial index for O(n) average instead of O(n²).
    # The pair tolerance below is max(size1, size2)/4 (floored at `tolerance`),
    # so the widest ring any pair can need is max_point_size/4 — using that
    # exact bound instead of a fixed 1.0, and sizing the index cells to it so
    # the 3x3-cell scan spans ~3x the radius rather than 3 mm, keeps this loop
    # proportional to real neighbours on dense plane fill (issue #363: the
    # fixed radius/cell made it dominate plane-step runtime).
    max_point_size = max((p[4] for p in all_points), default=0.0)
    max_tolerance = max(max_point_size / 4, tolerance)
    point_index = SpatialIndex(cell_size=max(max_tolerance * 1.01, 0.05))
    for x, y, layer, pid, size in all_points:
        point_index.add(x, y, layer, pid, size)
    for x1, y1, l1, id1, size1 in all_points:
        # Query nearby points on same layer
        for x2, y2, id2, size2 in point_index.query_nearby(x1, y1, l1, max_tolerance):
            if id2 <= id1:  # Avoid duplicate checks
                continue
            # Use max(size1, size2) / 4, but at least the minimum tolerance
            point_tolerance = max(max(size1, size2) / 4, tolerance)
            if points_match(x1, y1, x2, y2, point_tolerance):
                _union(id1, id2)

    # Same-net pads whose copper physically overlaps are connected even when
    # their centres sit further apart than the tight point tolerance above —
    # e.g. a large exposed-pad EP and the thermal-via pads scattered near its
    # corners (issue #108). Bounding circles prune candidates; the union
    # itself needs the exact shape-vs-shape overlap test — circle distance
    # alone joined 1.5x0.7 rect pads a full 0.3 mm apart, hiding genuinely
    # unrouted castellated-module pads from both the grader and the
    # multipoint router that trusts this graph (issue #346).
    if len(pad_repr_id) > 1:
        pad_reach = {idx: _pad_bounding_radius(pads[idx])
                     for idx in pad_repr_id}
        max_pad_reach = max(pad_reach.values())
        pad_center_index = SpatialIndex(cell_size=1.0)
        for idx in pad_repr_id:
            pad = pads[idx]
            pad_center_index.add(pad.global_x, pad.global_y, 'pad', idx, pad_reach[idx])
        for idx in pad_repr_id:
            pad = pads[idx]
            query_radius = pad_reach[idx] + max_pad_reach + tolerance
            for x2, y2, jdx, _ in pad_center_index.query_nearby(
                    pad.global_x, pad.global_y, 'pad', query_radius):
                if jdx <= idx:  # Avoid duplicate checks / self
                    continue
                other = pads[jdx]
                # Copper only shares a layer when both pads occupy it (a
                # drilled pad spans every copper layer, like a via).
                if other.drill <= 0 and pad.drill <= 0:
                    if not (pad_copper_layers[idx] & pad_copper_layers[jdx]):
                        continue
                reach = pad_reach[idx] + pad_reach[jdx] + tolerance
                dx = pad.global_x - other.global_x
                dy = pad.global_y - other.global_y
                if dx * dx + dy * dy <= reach * reach and \
                        _pads_copper_touch(pad, other, tolerance):
                    _union(pad_repr_id[idx], pad_repr_id[jdx])

    # A via dropped *inside* an SMD pad's copper connects that pad even when the
    # via centre is offset from the pad centre — e.g. a via-in-pad at the end of
    # a 0.6×1.95 mm plane pad sits >0.15 mm from the centre, so the tight
    # centre-to-centre proximity test above misses it and the pad reads as
    # unconnected despite a physical via in it (issue #89 via-in-pad subcase).
    # Union a via to a pad whenever the via COPPER overlaps the pad outline
    # and they share a copper layer. Center-containment is not enough: a
    # via-in-pad placed off-centre on a small circular pad can have its
    # centre just outside the outline while the barrel overlaps the pad
    # copper by >0.1mm -- KiCad grades that connected (kuchen /PWR1V2
    # U1: via centre 0.283mm from a 0.42 circle pad centre, 0.137mm real
    # copper overlap, kicad-cli reports 0 unconnected; the old rule made it
    # a phantom incomplete net). Same overlap-credit semantics as the #285
    # endpoint-cap rule below: margin = via radius (less epsilon), so the
    # strict width-clamped twin keeps its tight gate automatically.
    if via_repr_id and pad_repr_id:
        via_pos_index = SpatialIndex(cell_size=1.0)
        max_via_r = 0.0
        for via_idx in via_repr_id:
            v = vias[via_idx]
            vsize = getattr(v, 'size', 0.6)
            max_via_r = max(max_via_r, vsize / 2)
            via_pos_index.add(v.x, v.y, '_via', via_idx, vsize)
        for pad_idx in pad_repr_id:
            pad = pads[pad_idx]
            reach = max(pad.size_x, pad.size_y) / 2 + tolerance + max_via_r
            for vx, vy, via_idx, vsize in via_pos_index.query_nearby(
                    pad.global_x, pad.global_y, '_via', reach):
                if not (via_copper_layers[via_idx] & pad_copper_layers[pad_idx]):
                    continue
                _m = max(vsize / 2 - 1e-6, tolerance)
                if _point_in_pad(vx, vy, pad, margin=_m):
                    _union(pad_repr_id[pad_idx], via_repr_id[via_idx])

    # A track that *ends inside* a pad's copper outline connects that pad even
    # when the endpoint is offset from the pad centre by more than the tight
    # centre-to-centre tolerance above — e.g. a route landing in a large
    # through-hole connector pad enters the annular ring but stops well over
    # size/4 from the pad centre, so the proximity test misses it though the
    # copper physically touches (issue #195). Mirror the via-in-pad rule onto
    # segment endpoints: union a pad to any segment endpoint that lies within the
    # pad outline on a copper layer the pad occupies.
    if pad_repr_id and segments:
        endpoint_index = SpatialIndex(cell_size=1.0)
        for seg_idx, seg in enumerate(segments):
            if getattr(seg, 'graphic', False):
                continue  # graphics never conduct (#513 item 6, see above)
            if seg.layer.endswith('.Cu'):
                endpoint_index.add(seg.start_x, seg.start_y, seg.layer, seg_idx * 2, seg.width)
                endpoint_index.add(seg.end_x, seg.end_y, seg.layer, seg_idx * 2 + 1, seg.width)
        for pad_idx in pad_repr_id:
            pad = pads[pad_idx]
            reach = max(pad.size_x, pad.size_y) / 2 + tolerance
            for layer in pad_copper_layers[pad_idx]:
                for ex, ey, eid, ewidth in endpoint_index.query_nearby(
                        pad.global_x, pad.global_y, layer, reach):
                    # The endpoint's round cap (radius width/2) physically
                    # overlaps the pad whenever it reaches the outline --
                    # same overlap-credit semantics as the #285 endpoint-cap
                    # rule (the strict twin clamps widths, so the strict
                    # graph keeps its tight gate automatically).
                    _m = max(ewidth / 2 - 1e-6, tolerance)
                    if _point_in_pad(ex, ey, pad, margin=_m):
                        _union(pad_repr_id[pad_idx], eid)

    # Build spatial index for segments. Cell size = the widest credit reach
    # any point below can need, so every query below is a single 3x3-cell
    # ring — on 0.05mm-pitch plane fill the old fixed 1mm cells held hundreds
    # of segments each and this loop dominated plane-step runtime (#363).
    max_seg_width = max((seg.width for seg in segments), default=0.0)
    _max_reach = max((max_point_size + max_seg_width) / 2, tolerance)
    seg_index = SegmentIndex(cell_size=max(_max_reach * 1.01, 0.05))
    for seg_idx, seg in enumerate(segments):
        if getattr(seg, 'graphic', False):
            continue  # graphics never conduct (#513 item 6): a point lying on
            # a graphic must not union through it (T-junction/through-pad rules)
        seg_start_id = seg_idx * 2
        seg_index.add(seg, seg_start_id)

    # A track that passes THROUGH a pad mid-span connects it even though
    # neither endpoint lands anywhere near the pad -- e.g. a fanout tap
    # threading a capacitor pad on its way out (steppenprobe C7, #479): both
    # endpoints sat >1mm outside, the endpoint rule above missed it, and the
    # pad graded as a phantom disconnect while the router's terminal selector
    # (correctly) dropped it as already connected -- so every retry did
    # nothing and the board could never grade complete. Mirror
    # connectivity._pad_on_group's conservative gate exactly: credit only when
    # the centreline reaches well inside the pad (segment half-width plus a
    # quarter of the smaller pad dimension from the pad CENTRE), so a track
    # merely grazing the pad outline still grades as disconnected.
    if pad_repr_id and segments:
        for pad_idx in pad_repr_id:
            pad = pads[pad_idx]
            px, py, reach_pad = _pad_credit_disc(pad)
            for layer in pad_copper_layers[pad_idx]:
                for seg, seg_start_id in seg_index.query_near(
                        px, py, layer, radius=max_seg_width / 2 + reach_pad):
                    dx = seg.end_x - seg.start_x
                    dy = seg.end_y - seg.start_y
                    seg_len_sq = dx * dx + dy * dy
                    if seg_len_sq < 1e-8:
                        cx, cy = seg.start_x, seg.start_y
                    else:
                        t = ((px - seg.start_x) * dx
                             + (py - seg.start_y) * dy) / seg_len_sq
                        t = max(0.0, min(1.0, t))
                        cx = seg.start_x + t * dx
                        cy = seg.start_y + t * dy
                    if math.sqrt((px - cx) ** 2 + (py - cy) ** 2) \
                            <= seg.width / 2 + reach_pad:
                        _union(pad_repr_id[pad_idx], seg_start_id)

    # Check for T-junctions: points that lie on the middle of a segment (same layer)
    # Use spatial index for O(n) average instead of O(n × m)
    for px, py, player, pid, psize in all_points:
        ptype = point_info[pid][0]
        # Query segments that might contain this point. Endpoint/via points
        # credit out to (psize + seg_width)/2; the query ring must cover
        # that for LARGE vias (size + width can exceed the 1mm cell). Use the
        # net's real max segment width for the bound, not a fixed 1.0 — the
        # constant put the ring at 5x5 cells for every point on dense plane
        # fill and dominated plane-step runtime (issue #363).
        _reach = max((psize + max_seg_width) / 2, tolerance)
        for seg, seg_start_id in seg_index.query_near(px, py, player,
                                                      radius=_reach):
            seg_end_id = seg_start_id + 1
            # Skip if this point IS one of the segment's endpoints
            if pid == seg_start_id or pid == seg_end_id:
                continue
            # Check if point lies on this segment. For a SEGMENT endpoint the
            # copper is a round end cap of radius psize/2 (its track's half
            # width), so it physically overlaps this segment's copper whenever
            # the centreline distance is under (psize + seg.width)/2 -- two
            # 0.1mm tracks whose endpoints sit 0.05mm apart overlap in a
            # 0.087mm-wide lens, which KiCad counts as connected (issue #285:
            # eit_mux /S1 graded as a phantom "0.05mm gap"; the old width/2
            # threshold also made exact-tangency flip on FP epsilon across the
            # file write/parse round-trip). Exact tangency (zero-width copper)
            # is still NOT credited, so a real end-to-end gap stays flagged.
            # strict view: demand a real STRICT_JOINT_OVERLAP copper
            # lens instead of the grading epsilon -- two fat rail tips a
            # hair's width apart are separate FRAGMENTS to a planner even
            # where the grader shades them connected.
            _cap_margin = STRICT_JOINT_OVERLAP if strict_fragments else 1e-6
            if ptype in ('segment_start', 'segment_end'):
                seg_tolerance = max((psize + seg.width) / 2 - _cap_margin, tolerance)
            elif ptype == 'via':
                # A via barrel (psize = via diameter) physically overlaps this
                # segment's copper whenever the centerline passes within
                # (via_radius + track_half_width) -- KiCad counts that as
                # connected. The old width/2 tolerance missed e.g. a stub
                # ending 0.141mm from a 0.35 via's centre (inside the barrel,
                # glasgow /SDA #217) and graded a connected net as split.
                # Exact tangency is still NOT credited, matching the #285
                # endpoint-cap rule.
                seg_tolerance = max((psize + seg.width) / 2 - _cap_margin, tolerance)
            else:
                seg_tolerance = max(seg.width / 2, tolerance)
            if point_on_segment(px, py, seg.start_x, seg.start_y, seg.end_x, seg.end_y, seg_tolerance):
                # Connect this point to the segment (via one of its endpoints)
                _union(pid, seg_start_id)

    # Optional reusable graph for exclusion re-evaluation (see _union note).
    # pad_index_repr maps each input pad's INDEX to a representative point id,
    # so a caller replaying the edges through its own UnionFind can read off
    # per-pad components in the same id space (segment i's endpoints are point
    # ids 2i / 2i+1). Used by the multipoint component grouping (issue #317).
    graph = ({'num_points': point_id, 'pad_ids': list(pad_ids),
              'pad_locations': list(pad_locations), 'edges': edges,
              'pad_index_repr': dict(pad_repr_id),
              'via_index_repr': dict(via_repr_id),
              'zone_index_repr': dict(zone_repr_id),
              'num_segments': len(segments),
              'strict_fragments': strict_fragments,
              'zone_blob_fallback': _zone_blob_fallback}
             if return_graph else None)

    # Check if all pads are in the same component
    if not pad_ids:
        return {
            'connected': True,
            'num_components': 0,
            'pad_components': {},
            'disconnected_pads': [],
            'message': 'No pads found for this net',
            'debug_info': None,
            'graph': graph,
        }

    pad_roots = [uf.find(pid) for pid in pad_ids]
    unique_roots = set(pad_roots)

    # Find which pads are disconnected from the main group
    root_counts = defaultdict(int)
    for root in pad_roots:
        root_counts[root] += 1
    main_root = max(root_counts.keys(), key=lambda r: root_counts[r])

    disconnected = []
    seen_pads = set()  # Track (x, y, component_ref) to avoid duplicates for through-hole pads
    for pid, loc in zip(pad_ids, pad_locations):
        if uf.find(pid) != main_root:
            # For through-hole pads, only report once (not per layer)
            pad_key = (round(loc[0], 4), round(loc[1], 4), loc[3])  # (x, y, component_ref)
            if pad_key not in seen_pads:
                seen_pads.add(pad_key)
                disconnected.append(loc)

    # Build debug info if verbose
    debug_info = None
    if verbose and len(unique_roots) > 1:
        # Group all points by component
        components = defaultdict(list)
        for pt in all_points:
            x, y, layer, pid, size = pt
            root = uf.find(pid)
            info = point_info.get(pid, ('unknown',))
            components[root].append((x, y, layer, info))

        # For each component, find boundary points (potential disconnection points)
        component_summaries = {}
        for root, points in components.items():
            by_layer = defaultdict(list)
            for x, y, layer, info in points:
                by_layer[layer].append((x, y, info))

            # Summarize component
            summary = {
                'layers': list(by_layer.keys()),
                'points_by_layer': {l: len(pts) for l, pts in by_layer.items()},
                'has_pads': any(info[0] == 'pad' for _, _, _, info in points),
                'has_vias': any(info[0] == 'via' for _, _, _, info in points),
                'sample_points': [(x, y, layer, info[0]) for x, y, layer, info in points[:10]]
            }
            component_summaries[root] = summary

        debug_info = {
            'components': component_summaries,
            'component_points': dict(components),  # Raw points by component for gap analysis
            'main_root': main_root,
            'all_points': all_points,
            'point_info': point_info,
            'segments': segments,
            'vias': vias
        }

    return {
        'connected': len(unique_roots) == 1,
        'num_components': len(unique_roots),
        'pad_components': {loc: uf.find(pid) for pid, loc in zip(pad_ids, pad_locations)},
        'disconnected_pads': disconnected,
        'message': None,
        'debug_info': debug_info,
        'graph': graph,
    }


def analyze_conn_excluding(graph: Dict, excluded_seg_indices=()) -> Dict:
    """Re-evaluate net connectivity from a prebuilt graph (check_net_connectivity
    with return_graph=True) with some segments EXCLUDED, WITHOUT rebuilding the
    expensive spatial graph (#263).

    excluded_seg_indices index into the ORIGINAL segments list. Segment i's two
    endpoint points are ids 2i, 2i+1; excluding it drops every edge that touches
    them (its own start<->end union and any adjacency/T-junction/pad union to its
    endpoints), leaving those points isolated -- harmless, since only PAD roots
    decide connectivity. Returns {connected, num_components, disconnected_pads},
    matching check_net_connectivity on the reduced segment set.

    Caveat: this reuses the point set / copper-layer set built from the FULL
    segment list, so it diverges from a true recompute only if excluding a
    segment empties a copper layer (which would shrink a via/pad's layer span) --
    never the case on a plane net with zones + many segments per layer. Callers
    that need a hard guarantee should equivalence-check against a real recompute.
    """
    excl = set()
    for i in excluded_seg_indices:
        excl.add(2 * i)
        excl.add(2 * i + 1)
    uf = UnionFind()
    for a, b in graph['edges']:
        if a in excl or b in excl:
            continue
        uf.union(a, b)
    pad_ids = graph['pad_ids']
    pad_locations = graph['pad_locations']
    if not pad_ids:
        return {'connected': True, 'num_components': 0, 'disconnected_pads': []}
    pad_roots = [uf.find(pid) for pid in pad_ids]
    unique_roots = set(pad_roots)
    # Copper components over ALL points (pads + vias + non-excluded segment
    # endpoints): 'num_components' below is PAD components only, which lets
    # a removal strand a pad-less sliver unnoticed (castor POLLUX_SUB_IN's
    # 28um dangle, 0708d). Consumers that must not create islands or stubs
    # gate on this count instead.
    excluded = set(excluded_seg_indices)
    copper_roots = set(unique_roots)
    for vid in graph.get('via_index_repr', {}).values():
        copper_roots.add(uf.find(vid))
    n_segs_total = graph.get('num_segments', 0)
    for i_ in range(n_segs_total):
        if i_ in excluded:
            continue
        copper_roots.add(uf.find(2 * i_))
    # Copper components over ALL points (pads + vias + non-excluded segment
    # endpoints): 'num_components' below is PAD components only, which lets
    # a removal strand a pad-less sliver unnoticed (castor POLLUX_SUB_IN's
    # 28um dangle, 0708d). Consumers that must not create islands or stubs
    # gate on this count instead.
    excluded = set(excluded_seg_indices)
    copper_roots = set(unique_roots)
    for vid in graph.get('via_index_repr', {}).values():
        copper_roots.add(uf.find(vid))
    n_segs_total = graph.get('num_segments', 0)
    for i_ in range(n_segs_total):
        if i_ in excluded:
            continue
        copper_roots.add(uf.find(2 * i_))
    root_counts = defaultdict(int)
    for r in pad_roots:
        root_counts[r] += 1
    main_root = max(root_counts.keys(), key=lambda r: root_counts[r])
    disconnected = []
    seen_pads = set()
    for pid, loc in zip(pad_ids, pad_locations):
        if uf.find(pid) != main_root:
            pad_key = (round(loc[0], 4), round(loc[1], 4), loc[3])
            if pad_key not in seen_pads:
                seen_pads.add(pad_key)
                disconnected.append(loc)
    return {'connected': len(unique_roots) == 1,
            'num_components': len(unique_roots),
            'disconnected_pads': disconnected}


def find_gap_between_components(debug_info: Dict, tolerance: float) -> Optional[Dict]:
    """Analyze debug info to find where the gap is between disconnected components.

    Returns information about the likely disconnection point.
    """
    if not debug_info:
        return None

    component_summaries = debug_info['components']
    component_points = debug_info.get('component_points', {})
    all_points = debug_info['all_points']
    segments = debug_info['segments']
    vias = debug_info['vias']

    if len(component_points) < 2:
        return None

    # Find the closest points between different components
    min_dist = float('inf')
    gap_info = None

    roots = list(component_points.keys())
    for i, root1 in enumerate(roots):
        for root2 in roots[i+1:]:
            for pt1 in component_points[root1]:
                x1, y1, l1, info1 = pt1[0], pt1[1], pt1[2], pt1[3]
                for pt2 in component_points[root2]:
                    x2, y2, l2, info2 = pt2[0], pt2[1], pt2[2], pt2[3]
                    # Only compare points on the same layer (that's where connections happen)
                    if l1 != l2:
                        continue
                    dist = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        info1_type = info1[0] if isinstance(info1, tuple) else info1
                        info2_type = info2[0] if isinstance(info2, tuple) else info2
                        gap_info = {
                            'distance': dist,
                            'point1': (x1, y1, l1, info1_type),
                            'point2': (x2, y2, l2, info2_type),
                            'component1_root': root1,
                            'component2_root': root2
                        }

    # Also find if there are points on different layers at the same position
    # (missing via situation)
    for root1 in roots:
        for root2 in roots:
            if root1 >= root2:
                continue
            for pt1 in component_points[root1]:
                x1, y1, l1, info1 = pt1[0], pt1[1], pt1[2], pt1[3]
                for pt2 in component_points[root2]:
                    x2, y2, l2, info2 = pt2[0], pt2[1], pt2[2], pt2[3]
                    if l1 == l2:
                        continue
                    if abs(x1 - x2) < tolerance and abs(y1 - y2) < tolerance:
                        # Points at same position but different layers and not connected
                        info1_type = info1[0] if isinstance(info1, tuple) else info1
                        info2_type = info2[0] if isinstance(info2, tuple) else info2
                        return {
                            'type': 'missing_via',
                            'position': (x1, y1),
                            'layers': [l1, l2],
                            'point1_type': info1_type,
                            'point2_type': info2_type,
                            'message': f"Gap at ({x1:.3f}, {y1:.3f}): {info1_type} on {l1} not connected to {info2_type} on {l2}"
                        }

    if gap_info:
        gap_info['type'] = 'gap_on_layer'
        gap_info['message'] = (
            f"Nearest gap: {gap_info['distance']:.3f}mm on {gap_info['point1'][2]} between "
            f"{gap_info['point1'][3]} at ({gap_info['point1'][0]:.3f}, {gap_info['point1'][1]:.3f}) and "
            f"{gap_info['point2'][3]} at ({gap_info['point2'][0]:.3f}, {gap_info['point2'][1]:.3f})"
        )

    return gap_info


def run_connectivity_check(pcb_file: str, net_patterns: Optional[List[str]] = None,
                           tolerance: float = 0.02, quiet: bool = False,
                           verbose: bool = False, component: Optional[str] = None,
                           routed_only: bool = False) -> List[Dict]:
    """Run connectivity checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        net_patterns: Optional list of net name patterns (fnmatch style) to check.
        tolerance: Minimum connection tolerance in mm (default: 0.02mm)
        quiet: If True, only print a summary line unless there are issues
        verbose: If True, show detailed info about where breaks are
        component: Optional component reference to filter nets (e.g., "U1")
        routed_only: If True, only check routed nets (skip unrouted net detection)

    Returns:
        List of connectivity issues found
    """
    if quiet and (net_patterns or component):
        # Print a brief summary line in quiet mode
        desc = ', '.join(net_patterns) if net_patterns else f"component {component}"
        print(f"Checking {desc} for connectivity...", end=" ", flush=True)
    elif not quiet:
        print(f"Loading {pcb_file}...")

    pcb_data = parse_kicad_pcb(pcb_file)

    if not quiet:
        total_pads = sum(len(pads) for pads in pcb_data.pads_by_net.values())
        zone_info = f", {len(pcb_data.zones)} zones" if pcb_data.zones else ""
        print(f"Found {len(pcb_data.segments)} segments, {len(pcb_data.vias)} vias, {total_pads} pads{zone_info}")

    # Group segments by net
    segments_by_net = defaultdict(list)
    for seg in pcb_data.segments:
        segments_by_net[seg.net_id].append(seg)

    # Group vias by net
    vias_by_net = defaultdict(list)
    for via in pcb_data.vias:
        vias_by_net[via.net_id].append(via)

    # Group zones by net
    zones_by_net = defaultdict(list)
    for zone in pcb_data.zones:
        zones_by_net[zone.net_id].append(zone)

    # Use existing pads_by_net from pcb_data
    pads_by_net = pcb_data.pads_by_net

    # Filter by component if specified
    component_net_ids = None
    if component:
        # Shared with route.py via net_queries (#537): a checker that resolved
        # --component differently from the router would be verifying a different
        # set of nets than the one that was routed.
        #
        # This also drops net 0 from the component's net set, which the local
        # loop used to include. Net 0 is the no-net pseudo-net (#497): on a board
        # that carries it in `nets`, --component graded it as a real net and
        # reported every no-net pad on the part as "disconnected" -- interf_u
        # --component U9 shipped a phantom "(net 0): 6 disconnected components".
        # The no-component path never had this (net 0 has no segments/vias, so
        # its `and net_id in pads_by_net` arm excludes it); only the component
        # filter let it through.
        from net_queries import nets_for_components
        component_net_ids = set(nets_for_components(pcb_data, [component]).net_ids)
        if not quiet:
            print(f"Found {len(component_net_ids)} nets on component {component}")

    # Determine which nets to check
    nets_to_check = []
    for net_id, net_info in pcb_data.nets.items():
        # Filter by component if specified
        if component_net_ids is not None and net_id not in component_net_ids:
            continue

        if net_patterns:
            if matches_any_pattern(net_info.name, net_patterns):
                nets_to_check.append((net_id, net_info.name))
        elif component_net_ids is not None:
            # Component specified but no patterns - check all component nets with segments or pads
            if net_id in segments_by_net or net_id in pads_by_net:
                nets_to_check.append((net_id, net_info.name))
        else:
            # Only check nets that have copper (segments, or vias -- a via alone
            # can bridge opposite-layer pads it overlaps, issue #285) and pads
            if (net_id in segments_by_net or net_id in vias_by_net) and net_id in pads_by_net:
                nets_to_check.append((net_id, net_info.name))

    # Multi-board files (a panel / module + control board drawn as separate
    # Edge.Cuts outlines, e.g. len42_filter2, #479): a net whose pads span two
    # outlines can NEVER be copper-joined across them -- the link is a
    # board-to-board connector mated at assembly. Grade such nets PER OUTLINE:
    # within each outline the net's pads must be connected; the inter-board
    # edge is free. Single-outline boards (the overwhelming majority) are
    # untouched.
    _outlines = pcb_data.board_info.board_outlines or []
    if len(_outlines) < 2:
        _outlines = None

    def _which_outline(px, py):
        if _outlines:
            for _oi, _poly in enumerate(_outlines):
                if point_in_polygon(px, py, _poly):
                    return _oi
        return None

    skipped_cross_board = []

    # Find unrouted nets (pads but no segments) unless routed_only
    unrouted_nets = []
    skipped_noconnect = 0
    skipped_noconnect_multi = []  # (name, pad_count) with >=2 pads (#513 item 7)
    if not routed_only:
        copper_layers = pcb_data.board_info.copper_layers or ['F.Cu', 'B.Cu']
        for net_id, net_info in pcb_data.nets.items():
            # Net 0 (empty name) is the unconnected catch-all, not a real net.
            # Do NOT skip power nets by name (issue #92): a power net is only
            # "fine when unrouted" if a zone actually covers it, which the
            # has_zones check below decides. urchin's +3V3 had no plane on this
            # board and was wrongly hidden by the old hardcoded name list.
            if not net_info.name:
                continue
            # Filter by component if specified
            if component_net_ids is not None and net_id not in component_net_ids:
                continue
            # Filter by pattern if specified
            if net_patterns and not matches_any_pattern(net_info.name, net_patterns):
                continue
            # Check if net has pads but no copper at all. A net with VIAS but no
            # segments is NOT automatically unrouted (issue #285, icepi_zero's
            # USB-C CC2 nets): a single via can legitimately bridge two
            # opposite-layer pads it physically overlaps (via-in-pad), and the
            # per-net union-find below is via/pad-shape aware -- let it decide.
            has_pads = net_id in pads_by_net and len(pads_by_net[net_id]) >= 2
            has_segments = net_id in segments_by_net
            has_vias = net_id in vias_by_net
            has_zones = net_id in zones_by_net
            if has_pads and not has_segments and not has_zones and not has_vias:
                # A net whose pads all overlap (e.g. a castellated module's
                # co-located TH+SMD pad pairs) is already connected (issue #92).
                if _net_pads_connected_by_overlap(pads_by_net[net_id], copper_layers):
                    continue
                # KiCad auto-named no-connects sharing one net (USB shield
                # tabs: 'unconnected-(J1-SHIELD-PadS1)' spanning 4 pads).
                # Wildcard routing skips them (net_queries.expand_net_patterns)
                # and human-routed references leave them unrouted (the
                # connector shell joins them mechanically), so grading them
                # manufactures phantom incompleteness (#479 ch32v003_usb /
                # ch32v006_dev). An explicit --nets pattern still checks them.
                if not net_patterns and net_info.name.lower().startswith('unconnected-'):
                    skipped_noconnect += 1
                    # #513 item 7: >=2 pads under one auto-named no-connect can
                    # be a reversible footprint's doubled pins that DO need a
                    # trace (klein_kb) -- surface them by name instead of
                    # hiding them in the skip count (shield tabs, #479, are
                    # the benign case; the human reference disambiguates).
                    if len(pads_by_net[net_id]) >= 2:
                        skipped_noconnect_multi.append(
                            (net_info.name, len(pads_by_net[net_id])))
                    continue
                # Multi-board: only an outline holding >=2 of the net's pads
                # has anything routable. A one-pad-per-board net is purely a
                # board-to-board link -- nothing on either board to route.
                if _outlines:
                    _counts = {}
                    for _p in pads_by_net[net_id]:
                        _oi = _which_outline(_p.global_x, _p.global_y)
                        _counts[_oi] = _counts.get(_oi, 0) + 1
                    if max(_counts.values()) <= 1:
                        skipped_cross_board.append(net_info.name)
                        continue
                unrouted_nets.append((net_id, net_info.name, len(pads_by_net[net_id])))

    # A scope that matched NOTHING is not a clean board. This printed
    # `Checking 0 nets matching: [...]` and then went on to
    # `ALL NETS FULLY CONNECTED!` and exit 0 -- so a typo'd or shell-mangled
    # --nets turned the instrument the project says to always run before
    # calling a route clean into a rubber stamp, invisible to `&&` and to
    # `set -e`. Found live when a shell rewrote `/IO_Banks/Z0` into a Windows
    # path. Same failure mode as the oracle branch ~200 lines below, which was
    # already fixed there.
    if (net_patterns or component) and not nets_to_check:
        _what = []
        if net_patterns:
            _what.append(f"--nets {net_patterns}")
        if component:
            _what.append(f"component {component}")
        print(f"ERROR: {' and '.join(_what)} matched NO nets on this board. "
              f"That is a scope that selected nothing, not a board that is "
              f"connected -- refusing to report a result. Check the pattern "
              f"(a leading '/' is rewritten by some shells; MSYS_NO_PATHCONV=1 "
              f"on Git Bash), or drop the flag to check every net.",
              file=sys.stderr)
        # Returned as an issue rather than an exit code: this function's
        # contract is List[Dict] and library callers unpack it. main() maps
        # `scope_error` to exit 2, which is distinct from 1 ("found real
        # problems") so a caller can tell a bad scope from a bad board.
        return [{'scope_error': True, 'net_patterns': net_patterns,
                 'component': component,
                 'description': 'the requested scope matched no nets'}]

    if not quiet:
        if net_patterns and component:
            print(f"Checking {len(nets_to_check)} nets on {component} matching: {net_patterns}")
        elif net_patterns:
            print(f"Checking {len(nets_to_check)} nets matching: {net_patterns}")
        elif component:
            print(f"Checking {len(nets_to_check)} nets on component {component}")
        else:
            print(f"Checking {len(nets_to_check)} routed nets")
        if skipped_noconnect:
            print(f"  Skipped {skipped_noconnect} unrouted no-connect net(s) "
                  f"('unconnected-*'); pass --nets to check them")
        if skipped_noconnect_multi:
            _lst = ', '.join(f"'{n}' ({c} pads)"
                             for n, c in sorted(skipped_noconnect_multi)[:8])
            print(f"  WARNING: {len(skipped_noconnect_multi)} of the skipped "
                  f"no-connect net(s) have >=2 pads: {_lst}. A reversible "
                  f"footprint's doubled pins are auto-named this way and DO "
                  f"need a trace -- check the human reference and route them "
                  f"by name if so (#513 item 7).")
        if _outlines:
            print(f"  Multi-board file: {len(_outlines)} board outlines; "
                  f"nets graded per outline (board-to-board links exempt)")

    issues = []

    # Report unrouted nets as issues
    for net_id, net_name, num_pads in unrouted_nets:
        issues.append({
            'net_id': net_id,
            'net_name': net_name,
            'num_segments': 0,
            'num_vias': 0,
            'num_pads': num_pads,
            'num_components': num_pads,  # Each pad is its own component
            'disconnected_pads': [],
            'unrouted': True,
            'message': f'Unrouted net with {num_pads} pads'
        })

    for net_id, net_name in nets_to_check:
        segments = segments_by_net.get(net_id, [])
        vias = vias_by_net.get(net_id, [])
        pads = pads_by_net.get(net_id, [])
        zones = zones_by_net.get(net_id, [])

        result = check_net_connectivity(net_id, segments, vias, pads, zones, tolerance, verbose=verbose,
                                        pcb_data=pcb_data)

        # Multi-board: the net is complete when each outline's pads share one
        # connected component -- the only missing edges then run between
        # outlines, where no copper can ever go (net_break_within_outlines).
        if not result['connected'] and _outlines:
            _broken, _ = net_break_within_outlines(pcb_data, result)
            if not _broken:
                skipped_cross_board.append(net_name)
                continue

        if not result['connected']:
            issue = {
                'net_id': net_id,
                'net_name': net_name,
                'num_segments': len(segments),
                'num_vias': len(vias),
                'num_pads': len(pads),
                'num_components': result['num_components'],
                'disconnected_pads': result['disconnected_pads'],
                'message': result.get('message'),
                'debug_info': result.get('debug_info')
            }

            # Analyze the gap
            if verbose and result.get('debug_info'):
                gap = find_gap_between_components(result['debug_info'], tolerance)
                if gap:
                    issue['gap_info'] = gap

            issues.append(issue)

    if skipped_cross_board and not quiet:
        print(f"  {len(skipped_cross_board)} net(s) complete within each board "
              f"outline (only board-to-board links missing): "
              f"{', '.join(sorted(skipped_cross_board))}")

    # KiCad reconciliation for ZONE-BACKED nets, BOTH directions. Signal nets
    # keep pure copper grading; KICAD_NO_GRADE_RECONCILE=1 disables for A/B.
    #
    # Forward (quantization phantoms): the fill model's ~0.05mm raster
    # provably cannot hold a legal fill corridor narrower than ~3 cells, so a
    # pour-connected net can grade split here while KiCad's exact polygon
    # fill connects it (quickfeather GND: one 0.17mm corridor). An incomplete
    # net that OWNS a zone with zero KiCad unconnected links is reclassified
    # connected (reported, not silent).
    #
    # Reverse (refill divergence): a net this grading passed -- or never
    # graded at all: a pour-only net has no tracks to enter the copper check
    # -- while KiCad's refill still demands links. Causes: the refill
    # regrowing at a looser clearance than the fill the router verified,
    # pad-less copper clusters (a dangling via) invisible to pads-only
    # grading (KiCad demands links between ALL copper clusters -- the
    # island-semantics fixtures), or capsule-tolerance credit KiCad's exact
    # geometry rejects. Eligible: owns a zone, or has copper on the board;
    # the deliberate exemptions (cross-board links, auto-named no-connects)
    # stay exempt. Without this direction those ship silently and only
    # surface when the user opens KiCad (eth_tap BOOT0 class). The whole
    # reconcile still gates on the board having zones at all, so zone-less
    # synthetic/unit boards never pay the kicad-cli call.
    _zone_issue_ids = {i['net_id'] for i in issues
                       if any(z.net_id == i['net_id'] for z in pcb_data.zones)}
    # #654: this block used to print ONLY when it CHANGED something, so
    # "ran and agreed", "skipped -- no kicad-cli", "disabled" and "not
    # applicable" were one indistinguishable silence -- and a run that never
    # cross-checked still ended `ALL NETS FULLY CONNECTED!` / exit 0. That is
    # exactly the ambiguity the cross-check exists to remove, and resolving it
    # cost a hand call to kicad_unconnected() mid-campaign. Every path below
    # now records a state, and exactly ONE status line is printed after the
    # block, so a zone-backed verdict can be trusted or distrusted from the
    # report alone.
    _recon = {'state': 'not_applicable', 'links': 0, 'reclass': 0,
              'kicad_only': 0, 'detail': ''}
    if not pcb_data.zones:
        _recon['detail'] = 'board has no zones'
    elif os.environ.get('KICAD_NO_GRADE_RECONCILE'):
        _recon['state'] = 'disabled'
        _recon['detail'] = 'KICAD_NO_GRADE_RECONCILE is set'
    if pcb_data.zones and not os.environ.get('KICAD_NO_GRADE_RECONCILE'):
        try:
            from kicad_oracle import find_kicad_cli, kicad_unconnected
            _cli = find_kicad_cli()
            _links = kicad_unconnected(pcb_file, _cli) if _cli else None
            if _links is None:
                # kicad_unconnected() returns None for a missing CLI, a DRC
                # timeout (ORACLE_DRC_TIMEOUT 240s), a bad return code, or
                # unreadable JSON; only the timeout prints anything of its own.
                _recon['state'] = 'did_not_run'
                _recon['detail'] = ('kicad-cli not found' if not _cli else
                                    'the oracle returned nothing -- DRC '
                                    'timeout, bad exit, or unreadable output')
            if _links is not None:
                _recon['state'] = 'ran'
                _recon['links'] = len(_links)
                _linked_nets = {lk[0] for lk in _links}
                _reclass = [i for i in issues
                            if i['net_id'] in _zone_issue_ids
                            and i['net_name'] not in _linked_nets]
                if _reclass:
                    _recon['reclass'] = len(_reclass)
                    issues[:] = [i for i in issues if i not in _reclass]
                    _names = ', '.join(i['net_name'] for i in _reclass)
                    print(f"  {len(_reclass)} zone-backed net(s) reclassified "
                          f"CONNECTED by KiCad refill (fill-model "
                          f"quantization): {_names}")
                # Reverse direction: KiCad-only unconnected, within the
                # graded scope (pattern/component filters honored).
                #
                # Multi-outline boards: a link whose two clusters lie on
                # DIFFERENT outlines is board-to-board (Edge.Cuts clips the
                # fill per sub-board; no copper can ever join it) -- the
                # pad-based per-outline exemption never sees PAD-LESS pours
                # (g474 Earth, len42 GND), so filter at the link level:
                # distinct endpoints resolve by outline membership directly;
                # a same-point zone|zone link needs KiCad's exact islands
                # (one pcbnew refill, lazily, multi-outline boards only) to
                # find which outline the anchor island and its partners are
                # on. Unavailable exact fill keeps the flag (conservative).
                _outl = pcb_data.board_info.board_outlines or []
                _multi_out = len(_outl) >= 2
                _exact_g = {'fetched': False, 'map': None}

                def _out_of(x, y):
                    for _oi, _o in enumerate(_outl):
                        if point_in_polygon(x, y, _o):
                            return _oi
                    return None

                def _link_cross_board(lk):
                    if not _multi_out:
                        return False
                    _a, _b = lk[1], lk[2]
                    if abs(_a[0] - _b[0]) > 1e-6 or abs(_a[1] - _b[1]) > 1e-6:
                        _oa, _ob = _out_of(*_a[:2]), _out_of(*_b[:2])
                        return (_oa is not None and _ob is not None
                                and _oa != _ob)
                    if not _exact_g['fetched']:
                        _exact_g['fetched'] = True
                        try:
                            from kicad_exact_fill import refill_islands
                            _exact_g['map'] = refill_islands(pcb_file)
                        except Exception:
                            _exact_g['map'] = None
                    _m = _exact_g['map']
                    if not _m:
                        return False
                    from kicad_exact_fill import point_in_poly
                    _isl = [poly for (_n2, _ly), _polys in _m.items()
                            if _n2 == lk[0] for poly in _polys]
                    _cont = next((p for p in _isl
                                  if point_in_poly(_a[0], _a[1], p)), None)
                    if _cont is None:
                        # Anchor ON the island boundary (KiCad anchors zone
                        # links at fill vertices): nearest vertex within 1mm.
                        _best = (1.0, None)
                        for p in _isl:
                            for vx, vy in p:
                                _d = ((vx - _a[0]) ** 2
                                      + (vy - _a[1]) ** 2) ** 0.5
                                if _d < _best[0]:
                                    _best = (_d, p)
                        _cont = _best[1]
                    if _cont is None:
                        return False
                    _ao = _out_of(*_cont[0])
                    return not any(_out_of(*p[0]) == _ao
                                   for p in _isl if p is not _cont)

                _issue_names = {i['net_name'] for i in issues}
                _skip_names = set(skipped_cross_board)
                _zone_net_ids = {_z.net_id for _z in pcb_data.zones}
                _eligible = {}
                for _nid, _n in pcb_data.nets.items():
                    if _n.name and (_nid in _zone_net_ids
                                    or _nid in segments_by_net
                                    or _nid in vias_by_net):
                        _eligible.setdefault(_n.name, _nid)
                _rev = []
                for _name, _nid in sorted(_eligible.items()):
                    if (_name not in _linked_nets or _name in _issue_names
                            or _name in _skip_names):
                        continue
                    if net_patterns and not matches_any_pattern(_name,
                                                                net_patterns):
                        continue
                    if not net_patterns \
                            and _name.lower().startswith('unconnected-'):
                        continue
                    if component_net_ids is not None \
                            and _nid not in component_net_ids:
                        continue
                    _nl = [lk for lk in _links if lk[0] == _name
                           and not _link_cross_board(lk)]
                    if not _nl:
                        print(f"  {_name}: KiCad link(s) span board "
                              f"outlines only (board-to-board) -- exempt")
                        continue

                    def _ep(e):
                        _lyr = f" {e[2]}" if e[2] else ""
                        return f"({e[0]:.3f}, {e[1]:.3f}){_lyr} {e[3]}"

                    _rev.append({
                        'net_id': _nid,
                        'net_name': _name,
                        'num_segments': len(segments_by_net.get(_nid, [])),
                        'num_vias': len(vias_by_net.get(_nid, [])),
                        'num_pads': len(pads_by_net.get(_nid, [])),
                        'num_components': len(_nl) + 1,
                        'disconnected_pads': [],
                        'kicad_only': True,
                        'message': ("KiCad refill reports "
                                    f"{len(_nl)} unconnected link(s) copper "
                                    "grading missed: "
                                    + '; '.join(f"{_ep(lk[1])} <-> "
                                                f"{_ep(lk[2])}"
                                                for lk in _nl[:4])),
                    })
                if _rev:
                    _recon['kicad_only'] = len(_rev)
                    issues.extend(_rev)
                    _names = ', '.join(i['net_name'] for i in _rev)
                    print(f"  {len(_rev)} net(s) UNCONNECTED per KiCad "
                          f"refill though copper grading passed them: "
                          f"{_names}")
        except Exception as _re:
            _recon['state'] = 'error'
            _recon['detail'] = str(_re)

    # #654: the single always-printed status line. Emitted for EVERY state,
    # including the two that used to be silent (ran-and-agreed, and skipped),
    # because their silence was identical to a clean pass.
    if not quiet:
        print("  " + reconcile_status_line(_recon))

    # Report results
    if quiet:
        if issues:
            print(f"FAILED ({len(issues)} issues)")
        else:
            print("OK")
            return issues

    # Print detailed results (always for non-quiet, or when issues in quiet mode)
    if not quiet or issues:
        print("\n" + "=" * 60 if not quiet else "=" * 60)
        if issues:
            # Separate unrouted from connectivity issues
            unrouted_issues = [i for i in issues if i.get('unrouted')]
            connectivity_issues = [i for i in issues if not i.get('unrouted')]

            print(f"FOUND {len(issues)} ISSUES:\n")

            if unrouted_issues:
                print(f"  Unrouted nets ({len(unrouted_issues)}):")
                for issue in unrouted_issues:
                    print(f"    {issue['net_name']} ({issue['num_pads']} pads)")
                print()

            if connectivity_issues:
                print(f"  Connectivity issues ({len(connectivity_issues)}):")
                for issue in connectivity_issues:
                    print(f"\n  {issue['net_name']} (net {issue['net_id']}):")
                    print(f"    Segments: {issue['num_segments']}, Vias: {issue['num_vias']}, Pads: {issue['num_pads']}")
                    print(f"    Disconnected components: {issue['num_components']}")
                    if issue['disconnected_pads']:
                        print(f"    Disconnected pads:")
                        for loc in issue['disconnected_pads'][:5]:
                            print(f"      ({loc[0]:.2f}, {loc[1]:.2f}) on {loc[2]} [{loc[3]}]")
                        if len(issue['disconnected_pads']) > 5:
                            print(f"      ... and {len(issue['disconnected_pads']) - 5} more")
                    if issue.get('gap_info'):
                        gap = issue['gap_info']
                        print(f"    Break location: {gap['message']}")
                        if verbose and gap.get('type') == 'gap_on_layer':
                            debug = issue.get('debug_info')
                            if debug:
                                # Show component details
                                for root, summary in debug['components'].items():
                                    is_main = root == debug['main_root']
                                    print(f"    Component {'(main)' if is_main else '(disconnected)'}: "
                                          f"layers={summary['layers']}, "
                                          f"has_pads={summary['has_pads']}, has_vias={summary['has_vias']}")
                                    if verbose:
                                        print(f"      Points by layer: {summary['points_by_layer']}")
                    if issue.get('message'):
                        print(f"    Note: {issue['message']}")
        else:
            print("ALL NETS FULLY CONNECTED!")

        print("=" * 60)
    return issues


if __name__ == "__main__":
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    parser = argparse.ArgumentParser(description='Check PCB for track connectivity (disconnected routes)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Net name patterns to check (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--component', '-C',
                        help='Check all nets connected to this component (e.g., U1)')
    parser.add_argument('--tolerance', '-t', type=float, default=0.02,
                        help='Minimum connection tolerance in mm (default: 0.02)')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are issues')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed break location info for disconnected nets')
    parser.add_argument('--routed-only', '-r', action='store_true',
                        help='Only check routed nets (skip unrouted net detection)')

    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()

    issues = run_connectivity_check(args.pcb, args.nets, args.tolerance, args.quiet, args.verbose, args.component, args.routed_only)
    # 2 = the scope selected nothing, so no board was graded. Deliberately not
    # 1: a caller must be able to tell "your pattern is wrong" from "this board
    # has unconnected nets", and must never read either as success.
    if any(i.get('scope_error') for i in issues):
        sys.exit(2)
    sys.exit(1 if issues else 0)

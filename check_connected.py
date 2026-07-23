"""
Connectivity Checker - Verify that tracks form fully connected routes from source to target pads.
"""
from __future__ import annotations

import sys
import argparse
import math
import fnmatch
from typing import List, Dict, Set, Tuple, Optional
from collections import defaultdict
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad, PCBData, Zone
from net_queries import expand_pad_layers


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
    for x, y in _pad_perimeter_points(pi):
        if point_to_pad_distance(x, y, pj) <= tolerance:
            return True
    for x, y in _pad_perimeter_points(pj):
        if point_to_pad_distance(x, y, pi) <= tolerance:
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


def make_real_fill_validator(pcb_data, net_id, margin: float = 0.25):
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
    validate() is O(local)."""
    import math as _m
    _buckets = {}

    def _build(layer):
        b = {}

        def _add(x1, y1, x2, y2, reach, obj):
            for bx in range(int(min(x1, x2) - reach) - 1,
                            int(max(x1, x2) + reach) + 2):
                for by in range(int(min(y1, y2) - reach) - 1,
                                int(max(y1, y2) + reach) + 2):
                    b.setdefault((bx, by), []).append(obj)
        for v in pcb_data.vias:
            if v.net_id != net_id:
                _add(v.x, v.y, v.x, v.y, v.size / 2 + margin,
                     ('c', v.x, v.y, v.size / 2))
        for s in pcb_data.segments:
            if s.net_id != net_id and s.layer == layer:
                _add(s.start_x, s.start_y, s.end_x, s.end_y,
                     s.width / 2 + margin, ('s', s))
        for plist in pcb_data.pads_by_net.values():
            for p in plist:
                if p.net_id == net_id:
                    continue
                if p.drill <= 0 and layer not in p.layers \
                        and '*.Cu' not in p.layers:
                    continue
                r = max(p.size_x, p.size_y) / 2
                _add(p.global_x, p.global_y, p.global_x, p.global_y,
                     r + margin, ('p', p))
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


def check_net_connectivity(net_id: int, segments: List[Segment], vias: List[Via],
                           pads: List[Pad], zones: List[Zone] = None,
                           tolerance: float = 0.02,
                           verbose: bool = False,
                           return_graph: bool = False,
                           zone_credit_validator=None,
                           pcb_data=None) -> Dict:
    """Check connectivity for a single net.

    Args:
        net_id: The net ID being checked
        segments: Track segments belonging to this net
        vias: Vias belonging to this net
        pads: Pads belonging to this net
        zones: Zones (power planes) belonging to this net
        tolerance: Connection tolerance in mm
        verbose: If True, include detailed debug info

    Returns dict with:
        - connected: bool - whether all pads are connected
        - num_components: int - number of disconnected components
        - pad_components: dict mapping pad location to component id
        - disconnected_pads: list of pad locations not connected to the main component
        - debug_info: dict with detailed component info (when verbose=True)
    """
    if zones is None:
        zones = []
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
    for zone in zones:
        if zone.layer.endswith('.Cu'):
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
            pad_size = 0.4
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

        # Find which points are inside the zone polygon
        points_in_zone = []   # legacy blob (no model verdict)
        points_by_comp = {}   # fill component id -> [pid]
        for x, y, layer, pid, size in points_on_layer:
            if point_in_polygon(x, y, zone.polygon):
                # Removal gates pass a fill validator (#outline-over-credit,
                # bitaxe Q2): outline membership only counts where real fill
                # can provably exist. GRADING callers pass None and keep the
                # permissive outline credit.
                if zone_credit_validator is not None and \
                        not zone_credit_validator(x, y, zone.layer):
                    continue
                _c = _zm.query_component(x, y, size=size) if _zm is not None else None
                if _c is None:
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
        seg_start_id = seg_idx * 2
        seg_index.add(seg, seg_start_id)

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
            if ptype in ('segment_start', 'segment_end'):
                seg_tolerance = max((psize + seg.width) / 2 - 1e-6, tolerance)
            elif ptype == 'via':
                # A via barrel (psize = via diameter) physically overlaps this
                # segment's copper whenever the centerline passes within
                # (via_radius + track_half_width) -- KiCad counts that as
                # connected. The old width/2 tolerance missed e.g. a stub
                # ending 0.141mm from a 0.35 via's centre (inside the barrel,
                # glasgow /SDA #217) and graded a connected net as split.
                # Exact tangency is still NOT credited, matching the #285
                # endpoint-cap rule.
                seg_tolerance = max((psize + seg.width) / 2 - 1e-6, tolerance)
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
              'num_segments': len(segments)}
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
        component_net_ids = set()
        for net_id, pads in pads_by_net.items():
            for pad in pads:
                if pad.component_ref == component:
                    component_net_ids.add(net_id)
                    break
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

    # Find unrouted nets (pads but no segments) unless routed_only
    unrouted_nets = []
    skipped_noconnect = 0
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
                    continue
                unrouted_nets.append((net_id, net_info.name, len(pads_by_net[net_id])))

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

    args = parser.parse_args()

    issues = run_connectivity_check(args.pcb, args.nets, args.tolerance, args.quiet, args.verbose, args.component, args.routed_only)
    sys.exit(1 if issues else 0)

"""
DRC Checker - Find overlapping tracks and vias between different nets.
"""
from __future__ import annotations

import sys
import argparse
import math
import fnmatch
import os
import numpy as np
from collections import defaultdict
from typing import List, Tuple, Set, Optional, Dict, Any
from kicad_parser import parse_kicad_pcb, Segment, Via, Pad
from geometry_utils import (
    point_to_segment_distance,
    closest_point_on_segment,
    segment_to_segment_closest_points,
    segment_to_segment_distance as _seg_seg_dist_coords,
)
from net_queries import expand_pad_layers


# A grading tolerance is a FRACTION of the clearance (--clearance-margin), which
# collapses to exactly 0.0 when the caller grades at margin 0 -- the honest
# setting, and the one the perturbed-corpus runs grade at. At 0.0 the comparison
# `overlap > tolerance` then fires on double-precision residue: two pads at
# exactly their required distance compute an overlap of ~1e-16mm through the
# hypot/sqrt path and get reported as a violation that no geometry contains
# (measured, run 7: three phantom flags on a board whose real count was 0).
#
# So the fraction gets an absolute floor. 1e-9mm is a picometre: three orders of
# magnitude below KiCad's own 1nm file resolution, so it can never hide real
# geometry, and many orders above the residue it exists to absorb.
FP_EPS_MM = 1e-9


def _grade_tol(clearance: float, clearance_margin: float) -> float:
    """The fractional grading tolerance, floored above float residue."""
    return max(clearance * clearance_margin, FP_EPS_MM)

import routing_defaults as defaults


# Per-run memo for expand_pad_layers. Within one run_drc call routing_layers is a
# single fixed list, and pads share a handful of distinct layer-sets (['F.Cu'],
# ['*.Cu'], ...), yet expand_pad_layers is called once per pad per check (tens of
# thousands of times). Caching by the pad's layer tuple collapses that to a few
# real computations. Identical results -- the expansion is a pure function of
# (pad_layers, routing_layers). The cache is keyed on the routing_layers object
# identity; a new run passes a new list, which clears the cache. We hold a
# reference to that list so its id() can't be recycled while the cache is live.
_EXPAND_CACHE: Dict[tuple, List[str]] = {}
_EXPAND_ROUTING = None

# Same-net endpoints closer than this are treated as a COINCIDENT (clean) joint;
# a larger gap that is still within cap-overlap is a fragile soft joint (#soft-joint).
# ~10um is above grid-quantization noise (~8um) so a snapped junction is not flagged.
# Single source of truth lives in routing_constants; kept under the old private
# name here for internal use.
from routing_constants import SOFT_JOINT_MIN_GAP as _SOFT_JOINT_MIN_GAP

# The one endpoint-coincidence radius (same value everywhere: 0.02mm / 20um).
from connectivity import (COINCIDENCE_TOL, endpoint_reaches_pad,
                          endpoint_reaches_via)


def pad_copper_layers(pad, board_copper) -> set:
    """The set of real copper layers a pad's copper occupies.

    KiCad writes two wildcards a pad's `layers` list can carry: ``*.Cu`` (every
    copper layer -- a through-hole barrel) and ``F&B.Cu`` (front and back only).
    #697 lifted this out of ``run_drc`` so the PLACEMENT side
    (placement/legality.py) resolves layer scope from the same function rather
    than a hand-mirrored copy.

    It now DELEGATES to ``expand_pad_layers`` rather than re-implementing the
    expansion. It originally forked because that function passed ``F&B.Cu``
    through verbatim; #722 fixed it there instead, which is the right place --
    ``expand_pad_layers`` is what check_connected and the ROUTER scope pads
    with, so the fork left the authority and the router wrong while curing only
    the clearance paths. Two spellings of one expansion is the defect class
    #695/#722 are about; this is the set-returning adapter, not a second answer.
    """
    return set(expand_pad_layers(list(getattr(pad, 'layers', None) or []),
                                 list(board_copper)))


def pads_shared_layer_clearance(eff: float, layer_rules, layers_a, layers_b=None):
    """KiCad's per-layer (.kicad_dru) clearance for two items that meet on their
    SHARED copper layers, with REPLACE semantics (#498).

    A custom rule REPLACES the net/class-resolved value on its layer rather than
    raising it, so: every shared layer ruled -> max(rule values); only some
    ruled -> max(eff, rule values); none ruled -> eff unchanged. TH geometry is
    identical on every layer, so the max over shared layers is exact.

    Returns `eff` untouched when there are no rules -- the strict no-op that
    makes this free for boards without a .kicad_dru (i.e. almost all of them).
    """
    if not layer_rules:
        return eff
    shared = set(layers_a)
    if layers_b is not None:
        shared &= set(layers_b)
    vals = [layer_rules[l] for l in shared if l in layer_rules]
    if not vals:
        return eff
    if all(l in layer_rules for l in shared):
        return max(vals)        # every shared layer ruled: rules replace
    return max([eff] + vals)


def _expand_cu(pad_layers: List[str], routing_layers: List[str]) -> List[str]:
    """Memoized expand_pad_layers for the duration of one run_drc call."""
    global _EXPAND_ROUTING
    if routing_layers is not _EXPAND_ROUTING:
        _EXPAND_CACHE.clear()
        _EXPAND_ROUTING = routing_layers
    key = tuple(pad_layers)
    cached = _EXPAND_CACHE.get(key)
    if cached is None:
        cached = expand_pad_layers(pad_layers, routing_layers)
        _EXPAND_CACHE[key] = cached
    return cached


class SpatialIndex:
    """Grid-based spatial index for fast proximity queries."""

    def __init__(self, cell_size: float = 2.0):
        """Initialize with given cell size in mm."""
        self.cell_size = cell_size
        self.inv_cell_size = 1.0 / cell_size
        # Dict[layer][cell_key] -> list of (object, net_id)
        self.cells_by_layer: Dict[str, Dict[Tuple[int, int], List[Tuple[Any, int]]]] = defaultdict(lambda: defaultdict(list))
        # For objects that span all layers (vias)
        self.all_layer_cells: Dict[Tuple[int, int], List[Tuple[Any, int]]] = defaultdict(list)

    def _get_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Get cell coordinates for a point."""
        return (int(x * self.inv_cell_size), int(y * self.inv_cell_size))

    def _get_segment_cells(self, seg: Segment) -> Set[Tuple[int, int]]:
        """Get all cells a segment passes through."""
        cells = set()
        x1, y1 = seg.start_x, seg.start_y
        x2, y2 = seg.end_x, seg.end_y

        # Add endpoint cells
        cells.add(self._get_cell(x1, y1))
        cells.add(self._get_cell(x2, y2))

        # Walk along segment and add intermediate cells
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            # Sample every half cell size
            steps = max(1, int(length * self.inv_cell_size * 2))
            for i in range(1, steps):
                t = i / steps
                x = x1 + t * dx
                y = y1 + t * dy
                cells.add(self._get_cell(x, y))

        return cells

    def add_segment(self, seg: Segment, net_id: int):
        """Add a segment to the index."""
        cells = self._get_segment_cells(seg)
        layer_cells = self.cells_by_layer[seg.layer]
        for cell in cells:
            layer_cells[cell].append((seg, net_id))

    def add_via(self, via: Via, net_id: int):
        """Add a via to the index (spans all layers)."""
        cell = self._get_cell(via.x, via.y)
        self.all_layer_cells[cell].append((via, net_id))

    @staticmethod
    def _pad_half_extents(pad: Pad) -> Tuple[float, float]:
        """Axis-aligned half-extents of the pad's REAL copper. A rotated pad's
        corners protrude past the unrotated size_x/size_y bbox (a long pad at
        30 deg bulges by up to half its length), and a custom pad's polygons can
        extend past the anchor size -- under-indexing either lets copper sit in
        cells the broad phase never registered, so a real violation is missed
        (false clean)."""
        half_x = pad.size_x / 2
        half_y = pad.size_y / 2
        rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
        if rr:
            rad = math.radians(rr)
            c, s = abs(math.cos(rad)), abs(math.sin(rad))
            half_x, half_y = half_x * c + half_y * s, half_x * s + half_y * c
        # pad.polygons are ABSOLUTE board coordinates (the exact checks feed
        # board-space query points straight into _point_to_polys_distance) --
        # measure their extent from the pad centre. Treating them as pad-local
        # offsets gave ~board-sized half-extents and indexed every custom pad
        # into every cell (sofle_pico: 266 custom pads -> a 40-minute grade).
        for poly in (getattr(pad, 'polygons', None) or []):
            for px, py in poly:
                half_x = max(half_x, abs(px - pad.global_x))
                half_y = max(half_y, abs(py - pad.global_y))
        return half_x, half_y

    def add_pad(self, pad: Pad, net_id: int, expanded_layers: List[str]):
        """Add a pad to the index."""
        # Pad covers a rectangular area (rotation/polygon-aware)
        half_x, half_y = self._pad_half_extents(pad)
        min_cell = self._get_cell(pad.global_x - half_x, pad.global_y - half_y)
        max_cell = self._get_cell(pad.global_x + half_x, pad.global_y + half_y)

        for layer in expanded_layers:
            if not layer.endswith('.Cu'):
                continue
            layer_cells = self.cells_by_layer[layer]
            for cx in range(min_cell[0], max_cell[0] + 1):
                for cy in range(min_cell[1], max_cell[1] + 1):
                    layer_cells[(cx, cy)].append((pad, net_id))

    def get_nearby_segments(self, seg: Segment) -> List[Tuple[Segment, int]]:
        """Get segments that might be near the given segment (same layer, nearby cells)."""
        cells = self._get_segment_cells(seg)
        layer_cells = self.cells_by_layer[seg.layer]

        seen = set()
        result = []
        for cell in cells:
            for obj, net_id in layer_cells.get(cell, []):
                if isinstance(obj, Segment) and id(obj) not in seen:
                    seen.add(id(obj))
                    result.append((obj, net_id))
        return result

    def get_nearby_for_via(self, via: Via, layer: str) -> List[Tuple[Any, int]]:
        """Get objects near a via on a specific layer."""
        cell = self._get_cell(via.x, via.y)
        # Check neighboring cells too (via has size)
        result = []
        seen = set()
        layer_cells = self.cells_by_layer[layer]
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in layer_cells.get(neighbor, []):
                    if id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_vias(self, via: Via) -> List[Tuple[Via, int]]:
        """Get vias near the given via."""
        cell = self._get_cell(via.x, via.y)
        result = []
        seen = set()
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in self.all_layer_cells.get(neighbor, []):
                    if isinstance(obj, Via) and id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_pads(self, x: float, y: float, layer: str) -> List[Tuple[Pad, int]]:
        """Get pads near a point on a specific layer."""
        cell = self._get_cell(x, y)
        result = []
        seen = set()
        layer_cells = self.cells_by_layer[layer]
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                neighbor = (cell[0] + dx, cell[1] + dy)
                for obj, net_id in layer_cells.get(neighbor, []):
                    if isinstance(obj, Pad) and id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_pads_for_pad(self, pad: "Pad", layer: str) -> List[Tuple["Pad", int]]:
        """Get pads near another pad on a layer. Unlike get_nearby_pads (which
        keys off a single point), this scans every cell the query pad spans plus
        a one-cell margin, so a large pad does not miss a neighbour sitting near
        its edge rather than its center (pad-pad check, #234)."""
        half_x, half_y = self._pad_half_extents(pad)
        min_cell = self._get_cell(pad.global_x - half_x, pad.global_y - half_y)
        max_cell = self._get_cell(pad.global_x + half_x, pad.global_y + half_y)
        layer_cells = self.cells_by_layer[layer]
        seen = set()
        result = []
        for cx in range(min_cell[0] - 1, max_cell[0] + 2):
            for cy in range(min_cell[1] - 1, max_cell[1] + 2):
                for obj, net_id in layer_cells.get((cx, cy), []):
                    if isinstance(obj, Pad) and id(obj) not in seen:
                        seen.add(id(obj))
                        result.append((obj, net_id))
        return result

    def get_nearby_pads_for_segment(self, seg: Segment) -> List[Tuple[Pad, int]]:
        """Get pads near ANY point of a segment (its layer), not just its
        endpoints. The pad-segment check used to query around the two endpoints
        only, so a long straight segment grazing a pad mid-run (a few mm from
        both ends) never saw that pad as a candidate -- a missed violation
        (false clean)."""
        layer_cells = self.cells_by_layer[seg.layer]
        seen = set()
        result = []
        for (cx, cy) in self._get_segment_cells(seg):
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for obj, net_id in layer_cells.get((cx + dx, cy + dy), []):
                        if isinstance(obj, Pad) and id(obj) not in seen:
                            seen.add(id(obj))
                            result.append((obj, net_id))
        return result


def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """Check if a net name matches any of the given patterns (fnmatch style)."""
    for pattern in patterns:
        if fnmatch.fnmatch(name, pattern):
            return True
    return False


def segment_to_segment_distance(seg1: Segment, seg2: Segment) -> float:
    """Calculate minimum distance between two segments."""
    dist, _, _ = segment_to_segment_closest_points(seg1, seg2)
    return dist


def segments_cross(seg1: Segment, seg2: Segment, tolerance: float = 0.001) -> Tuple[bool, Optional[Tuple[float, float]]]:
    """Check if two segments on the same layer cross each other.

    Returns (True, intersection_point) if they cross, (False, None) otherwise.
    Segments that share an endpoint are not considered crossing.
    """
    if seg1.layer != seg2.layer:
        return False, None

    x1, y1 = seg1.start_x, seg1.start_y
    x2, y2 = seg1.end_x, seg1.end_y
    x3, y3 = seg2.start_x, seg2.start_y
    x4, y4 = seg2.end_x, seg2.end_y

    # Check if segments share an endpoint (not a crossing)
    def points_equal(ax, ay, bx, by):
        return abs(ax - bx) < tolerance and abs(ay - by) < tolerance

    if (points_equal(x1, y1, x3, y3) or points_equal(x1, y1, x4, y4) or
        points_equal(x2, y2, x3, y3) or points_equal(x2, y2, x4, y4)):
        return False, None

    # Direction vectors
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x4 - x3, y4 - y3

    # Cross product of direction vectors
    cross = dx1 * dy2 - dy1 * dx2

    if abs(cross) < 1e-10:
        # Parallel segments - no crossing
        return False, None

    # Solve for parameters t and u where:
    # (x1, y1) + t * (dx1, dy1) = (x3, y3) + u * (dx2, dy2)
    dx3, dy3 = x3 - x1, y3 - y1
    t = (dx3 * dy2 - dy3 * dx2) / cross
    u = (dx3 * dy1 - dy3 * dx1) / cross

    # Check if intersection is within both segments (exclusive of endpoints)
    eps = 0.001  # Small margin to exclude near-endpoint intersections
    if eps < t < 1 - eps and eps < u < 1 - eps:
        # Calculate intersection point
        ix = x1 + t * dx1
        iy = y1 + t * dy1
        return True, (ix, iy)

    return False, None


def check_segment_overlap(seg1: Segment, seg2: Segment, clearance: float, clearance_margin: float = 0.05):
    """Check if two segments on the same layer violate clearance.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).
                         Violations smaller than clearance * clearance_margin are ignored.

    Returns:
        (has_violation, overlap, closest_pt1, closest_pt2)
    """
    if seg1.layer != seg2.layer:
        return False, 0.0, None, None

    # Required distance is half-widths plus clearance
    required_dist = seg1.width / 2 + seg2.width / 2 + clearance
    actual_dist, pt1, pt2 = segment_to_segment_closest_points(seg1, seg2)
    overlap = required_dist - actual_dist

    # Use clearance-based tolerance (5% of clearance by default)
    tolerance = _grade_tol(clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap, pt1, pt2
    return False, 0.0, None, None


def check_via_segment_overlap(via: Via, seg: Segment, clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float]:
    """Check if a via overlaps with a segment on any common layer.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).
    """
    # Standard through-hole vias go through ALL copper layers, not just the ones listed
    # Only skip non-copper layers
    if not seg.layer.endswith('.Cu'):
        return False, 0.0

    required_dist = via.size / 2 + seg.width / 2 + clearance
    actual_dist = point_to_segment_distance(via.x, via.y,
                                            seg.start_x, seg.start_y,
                                            seg.end_x, seg.end_y)
    overlap = required_dist - actual_dist

    tolerance = _grade_tol(clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_via_via_overlap(via1: Via, via2: Via, clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float]:
    """Check if two vias overlap.

    Args:
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).
    """
    # All vias are through-hole, so they always potentially conflict
    required_dist = via1.size / 2 + via2.size / 2 + clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    tolerance = _grade_tol(clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def point_to_rect_distance(px: float, py: float, cx: float, cy: float,
                           half_x: float, half_y: float,
                           corner_radius: float = 0.0) -> float:
    """Calculate distance from a point to an axis-aligned rectangle with optional rounded corners.

    Args:
        px, py: Point coordinates
        cx, cy: Rectangle center coordinates
        half_x, half_y: Rectangle half-widths
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        Distance from point to rectangle edge (0 if point is inside)
    """
    # Position relative to rectangle center
    rel_x = abs(px - cx)
    rel_y = abs(py - cy)

    if corner_radius > 0:
        # Inner rectangle bounds (where corners start)
        inner_half_x = half_x - corner_radius
        inner_half_y = half_y - corner_radius

        # Check if point is in a corner region
        if rel_x > inner_half_x and rel_y > inner_half_y:
            # Distance to corner arc center
            dx = rel_x - inner_half_x
            dy = rel_y - inner_half_y
            dist_to_corner_center = math.sqrt(dx * dx + dy * dy)
            # Distance to arc edge (negative if inside)
            return max(0, dist_to_corner_center - corner_radius)

    # Point is along a flat edge - rectangular distance
    dx = max(0, rel_x - half_x)
    dy = max(0, rel_y - half_y)
    return math.sqrt(dx * dx + dy * dy)


def segment_to_rect_distance(x1: float, y1: float, x2: float, y2: float,
                             cx: float, cy: float, half_x: float, half_y: float,
                             corner_radius: float = 0.0) -> Tuple[float, Tuple[float, float]]:
    """Calculate minimum distance from a segment to an axis-aligned rectangle.

    Args:
        x1, y1, x2, y2: Segment endpoints
        cx, cy: Rectangle center coordinates
        half_x, half_y: Rectangle half-widths
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        (distance, closest_point_on_segment)
    """
    # Sample points along the segment and find minimum distance to rectangle.
    # point_to_rect_distance is inlined here (this is the DRC hotspot -- one call
    # per sample, hundreds of thousands per board) with the corner-radius branch
    # invariants hoisted out of the loop. The per-sample arithmetic is identical
    # to point_to_rect_distance, so the result is unchanged.
    min_dist = float('inf')
    closest_pt = (x1, y1)
    dx_seg = x2 - x1
    dy_seg = y2 - y1
    has_corner = corner_radius > 0
    inner_half_x = half_x - corner_radius
    inner_half_y = half_y - corner_radius
    sqrt = math.sqrt

    # Check endpoints and intermediate points (keep the original expression form
    # verbatim so num_samples is bit-identical to before)
    num_samples = max(10, int(math.sqrt((x2-x1)**2 + (y2-y1)**2) / 0.05))  # Sample every ~0.05mm
    for i in range(num_samples + 1):
        t = i / num_samples
        px = x1 + t * dx_seg
        py = y1 + t * dy_seg
        rel_x = abs(px - cx)
        rel_y = abs(py - cy)
        if has_corner and rel_x > inner_half_x and rel_y > inner_half_y:
            ddx = rel_x - inner_half_x
            ddy = rel_y - inner_half_y
            dist = max(0, sqrt(ddx * ddx + ddy * ddy) - corner_radius)
        else:
            ex = max(0, rel_x - half_x)
            ey = max(0, rel_y - half_y)
            dist = sqrt(ex * ex + ey * ey)
        if dist < min_dist:
            min_dist = dist
            closest_pt = (px, py)

    return min_dist, closest_pt


def _into_pad_frame(x: float, y: float, pad: Pad,
                    cos_r: float, sin_r: float) -> Tuple[float, float]:
    """Rotate a global point into a diagonal pad's local frame about its center
    (R(-rect_rotation)), so the axis-aligned rect-distance routines apply."""
    dx = x - pad.global_x
    dy = y - pad.global_y
    return (pad.global_x + dx * cos_r + dy * sin_r,
            pad.global_y - dx * sin_r + dy * cos_r)


def _point_in_poly(x: float, y: float, poly) -> bool:
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def _point_to_polys_distance(x: float, y: float, polys) -> float:
    """Min distance from a point to any of the polygons (0 if inside one)."""
    best = float('inf')
    for poly in polys:
        if _point_in_poly(x, y, poly):
            return 0.0
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            d = point_to_segment_distance(x, y, x1, y1, x2, y2)
            if d < best:
                best = d
    return best


# Sweep item 6 (#625 follow-up): per-poly edge arrays, memoized like the
# ring arrays above (a pad's polygons list lives as long as the pad).
_POLYS_EDGE_CACHE: Dict[int, tuple] = {}


def _polys_edge_arrays(polys):
    key = id(polys)
    fp = (len(polys), tuple(len(p) for p in polys))
    hit = _POLYS_EDGE_CACHE.get(key)
    if hit is not None and hit[0] == fp:
        return hit[1]
    vx, vy, ex, ey = [], [], [], []
    for poly in polys:
        P = np.asarray(poly, dtype=np.float64)
        if len(P) < 2:
            continue
        Q = np.roll(P, -1, axis=0)
        vx.append(P[:, 0]); vy.append(P[:, 1])
        ex.append(Q[:, 0]); ey.append(Q[:, 1])
    if not vx:
        arrays = None
    else:
        x1 = np.concatenate(vx); y1 = np.concatenate(vy)
        x2 = np.concatenate(ex); y2 = np.concatenate(ey)
        dx, dy = x2 - x1, y2 - y1
        arrays = (x1, y1, dx, dy, dx * dx + dy * dy)
    if len(_POLYS_EDGE_CACHE) > 64:
        _POLYS_EDGE_CACHE.clear()
    _POLYS_EDGE_CACHE[key] = (fp, arrays)
    return arrays


def _segment_to_polys_distance(x1: float, y1: float, x2: float, y2: float, polys):
    """Min distance from a segment to custom-pad polygon copper (0 if it enters),
    sampled along the segment like segment_to_rect_distance. Returns (dist, pt).

    Sweep item 6 (#625 follow-up): the 0.05mm samples x poly edges used to run
    the scalar _point_to_polys_distance per sample (millions of calls per DRC
    on custom-pad boards). The broadcast below NOMINATES the minimal samples
    (multiply-squared kernel; **2 = libm pow rounds 1 ULP apart on rare
    values), then the winners are recomputed with the scalar in sample order,
    preserving the strict first-minimum tie-break -- returns byte-identical."""
    length = math.hypot(x2 - x1, y2 - y1)
    n = max(10, int(length / 0.05))
    arrays = _polys_edge_arrays(polys)
    if arrays is not None and (n + 1) * len(arrays[0]) >= 4096:
        px1, py1, pdx, pdy, plen_sq = arrays
        t = np.arange(n + 1, dtype=np.float64) / n
        sx = x1 + t * (x2 - x1)
        sy = y1 + t * (y2 - y1)
        # inside-any test + min edge distance per sample, chunked.
        best_d2 = np.empty(n + 1)
        inside = np.zeros(n + 1, dtype=bool)
        _B = max(1, 2_000_000 // max(1, len(px1)))
        for s in range(0, n + 1, _B):
            bx = sx[s:s + _B, None]
            by = sy[s:s + _B, None]
            for poly in polys:
                P = np.asarray(poly, dtype=np.float64)
                if len(P) < 2:
                    continue
                yi = P[:, 1][None, :]
                yj = np.roll(P[:, 1], 1)[None, :]
                xi = P[:, 0][None, :]
                xj = np.roll(P[:, 0], 1)[None, :]
                cond = (yi > by) != (yj > by)
                with np.errstate(divide='ignore', invalid='ignore'):
                    xint = (xj - xi) * (by - yi) / (yj - yi) + xi
                    crossing = cond & (bx < xint)
                inside[s:s + _B] |= (np.count_nonzero(crossing, axis=1) & 1).astype(bool)
            best_d2[s:s + _B] = _pt_edges_d2(bx, by, px1[None, :], py1[None, :],
                                             pdx[None, :], pdy[None, :],
                                             plen_sq[None, :]).min(axis=1)
        best_d2 = np.where(inside, 0.0, best_d2)
        m = best_d2.min()
        cand = np.nonzero(best_d2 <= m + 8 * np.spacing(m))[0]
        best = float('inf')
        best_pt = (x1, y1)
        for i in cand:
            d = _point_to_polys_distance(float(sx[i]), float(sy[i]), polys)
            if d < best:
                best = d
                best_pt = (float(sx[i]), float(sy[i]))
        return best, best_pt
    best = float('inf')
    best_pt = (x1, y1)
    for i in range(n + 1):
        t = i / n
        px = x1 + t * (x2 - x1)
        py = y1 + t * (y2 - y1)
        d = _point_to_polys_distance(px, py, polys)
        if d < best:
            best = d
            best_pt = (px, py)
    return best, best_pt


_GRAPHIC_EFFECTIVE_NETS = None  # set per check run by _build_graphic_unification


def _build_graphic_unification(pcb_data):
    """KiCad derives a copper GRAPHIC's net from CONNECTIVITY, unifying every
    net whose copper physically touches the art (#337). Cluster touching
    graphics (flood over edge-contact), then record each cluster's EFFECTIVE
    net set = the file attributes plus every net whose segment/via/pad touches
    the cluster. Pair checks treat a graphic as same-net with any effective
    net -- eurorack's jack art carries a stale +12V attribute yet is soldered
    into the OUT nets, so its 68um "grazes" are internal spacing of one
    electrical net, which KiCad correctly ignores."""
    import math as _m
    global _GRAPHIC_EFFECTIVE_NETS
    _GRAPHIC_EFFECTIVE_NETS = {}
    graphics = [sg for sg in pcb_data.segments if getattr(sg, 'graphic', False)]
    if not graphics:
        return

    def seg_seg_touch(a, b):
        if a.layer != b.layer:
            return False
        lim = a.width / 2.0 + b.width / 2.0 + 1e-6
        return _seg_seg_distance(a, b) <= lim

    def _seg_seg_distance(a, b):
        best = float('inf')
        for (px, py) in ((a.start_x, a.start_y), (a.end_x, a.end_y)):
            best = min(best, _pt_seg_d(px, py, b))
        for (px, py) in ((b.start_x, b.start_y), (b.end_x, b.end_y)):
            best = min(best, _pt_seg_d(px, py, a))
        if best > 0:
            # Endpoint sampling misses CROSSING strokes (distance 0 with all
            # four endpoints far away) -- two graphic lines drawn as an X
            # never clustered and their intersection graded as a short.
            from check_drc import segments_cross as _sc
            try:
                crossed, _ = _sc(a, b)
                if crossed:
                    return 0.0
            except Exception:
                pass
        return best

    def _pt_seg_d(px, py, sgm):
        x1, y1, x2, y2 = sgm.start_x, sgm.start_y, sgm.end_x, sgm.end_y
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / L2))
        return _m.hypot(px - (x1 + t * dx), py - (y1 + t * dy))

    # union-find over graphics by physical contact
    parent = list(range(len(graphics)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(graphics)):
        for j in range(i + 1, len(graphics)):
            if seg_seg_touch(graphics[i], graphics[j]):
                parent[find(i)] = find(j)

    clusters = {}
    for i, g in enumerate(graphics):
        clusters.setdefault(find(i), []).append(g)

    for root, members in clusters.items():
        eff = {g.net_id for g in members}
        for g in members:
            hw = g.width / 2.0
            # touching routed/input segments
            for sg in pcb_data.segments:
                if getattr(sg, 'graphic', False) or sg.layer != g.layer:
                    continue
                if _seg_seg_distance(g, sg) <= hw + sg.width / 2.0 + 1e-6:
                    eff.add(sg.net_id)
            # touching vias (barrel spans all layers)
            for v in pcb_data.vias:
                if _pt_seg_d(v.x, v.y, g) <= hw + (v.size or 0.5) / 2.0 + 1e-6:
                    eff.add(v.net_id)
            # touching pads (on the graphic's layer)
            for pads in pcb_data.pads_by_net.values():
                for pd in pads:
                    lys = pd.layers or []
                    if g.layer not in lys and '*.Cu' not in lys:
                        continue
                    mid_d = min(point_to_pad_distance(g.start_x, g.start_y, pd),
                                point_to_pad_distance(g.end_x, g.end_y, pd))
                    if mid_d <= hw + 1e-6:
                        eff.add(pd.net_id)
        for g in members:
            _GRAPHIC_EFFECTIVE_NETS[id(g)] = eff


def _graphic_pair_is_same_net(seg_a, seg_b, net_a, net_b):
    """True when a pair involving copper GRAPHICS is the same electrical net
    under connectivity unification (see _build_graphic_unification). For a
    graphic-vs-graphic pair, two clusters sharing ANY effective net are one
    electrical net (eurorack's two jack arts both solder into the OUT nets)."""
    if _GRAPHIC_EFFECTIVE_NETS is None:
        return False
    ga = seg_a is not None and getattr(seg_a, 'graphic', False)
    gb = seg_b is not None and getattr(seg_b, 'graphic', False)
    if ga and gb:
        ea = _GRAPHIC_EFFECTIVE_NETS.get(id(seg_a)) or set()
        eb = _GRAPHIC_EFFECTIVE_NETS.get(id(seg_b)) or set()
        return bool(ea & eb)
    for g, flag, other_net in ((seg_a, ga, net_b), (seg_b, gb, net_a)):
        if flag:
            eff = _GRAPHIC_EFFECTIVE_NETS.get(id(g))
            if eff is not None and other_net in eff:
                return True
    return False


def point_to_pad_distance(px: float, py: float, pad: Pad) -> float:
    """Edge-to-edge distance from a point to a pad's copper (0 if inside the
    copper). Handles custom-polygon pads, rounded/rect/circle/oval shapes, and
    diagonal (rect_rotation) pads -- the same geometry the pad-segment and
    pad-via checks use, factored out for reuse by the pad-pad check (#234)."""
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        return _point_to_polys_distance(px, py, pad_polys)

    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0

    x, y = px, py
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        x, y = _into_pad_frame(x, y, pad, math.cos(rad), math.sin(rad))
    return point_to_rect_distance(x, y, pad.global_x, pad.global_y,
                                  pad.size_x / 2, pad.size_y / 2, corner_radius)


def _pad_perimeter_points(pad: Pad, n_per_side: int = 8) -> List[Tuple[float, float]]:
    """Sample points around a pad's copper perimeter in global coordinates.
    For custom-polygon pads, walk the real polygon edges; otherwise walk the
    (rotated) rounded-rect outline that matches point_to_pad_distance --
    circle/oval/roundrect corners are arcs, NOT the sharp bounding-box corners.
    Sampling the bbox made two non-touching circle pads read as overlapping
    (the corner sample sits sqrt(2)*r - r outside the real copper; issue #260,
    same phantom class as the #232 custom-pad bboxes). Used to measure pad-pad
    and pad-edge gaps by cross-sampling one pad's perimeter against the other
    shape's distance fn."""
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        pts = []
        for poly in pad_polys:
            m = len(poly)
            for i in range(m):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % m]
                for k in range(n_per_side):
                    t = k / n_per_side
                    pts.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        return pts

    hx, hy = pad.size_x / 2, pad.size_y / 2
    cx, cy = pad.global_x, pad.global_y
    if pad.shape in ('circle', 'oval'):
        r = min(hx, hy)
    elif pad.shape == 'roundrect':
        r = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        r = 0.0
    r = min(r, hx, hy)
    ix, iy = hx - r, hy - r  # half-extents of the straight (un-rounded) sections
    local = []
    edges = [((-ix, -hy), (ix, -hy)), ((hx, -iy), (hx, iy)),
             ((ix, hy), (-ix, hy)), ((-hx, iy), (-hx, -iy))]
    for (x1, y1), (x2, y2) in edges:
        if x1 == x2 and y1 == y2:
            continue  # fully-round side (circle/stadium): no straight section
        for k in range(n_per_side):
            t = k / n_per_side
            local.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    if r > 0.0:
        # Quarter arcs joining the edges, centered on the inner-rect corners.
        for acx, acy, a0 in ((ix, -iy, -90.0), (ix, iy, 0.0),
                             (-ix, iy, 90.0), (-ix, -iy, 180.0)):
            for k in range(n_per_side):
                a = math.radians(a0 + 90.0 * k / n_per_side)
                local.append((acx + r * math.cos(a), acy + r * math.sin(a)))
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        c, s = math.cos(rad), math.sin(rad)
        return [(cx + lx * c - ly * s, cy + lx * s + ly * c) for lx, ly in local]
    return [(cx + lx, cy + ly) for lx, ly in local]


def _pad_has_no_copper(pad: Pad) -> bool:
    """True for pads with no copper to clearance-check: NPTH mechanical holes
    (KiCad lists *.Cu on them for hole keep-out, but an np_thru_hole pad carries
    no ring -- its 'size' is just the mask opening) and pads declaring no copper
    layer at all. Their drill still matters and is covered by the hole checks
    (copper-to-hole, drill-to-drill); flagging their phantom 'copper' against
    neighbouring pads made KiCad-clean boards fail (issue #260)."""
    if getattr(pad, 'pad_type', '') == 'np_thru_hole':
        return True
    return not any(l == '*.Cu' or l.endswith('.Cu') for l in pad.layers)


# Sweep item 5 (#625 follow-up): the pad-pad and pad-touch passes ran 64
# scalar point_to_pad_distance calls per candidate pair (~2.5M per grade).
# Perimeter samples are memoized per pad (built by the SAME scalar sampler,
# so the values are identical), and each pair runs one broadcast of the
# multiply-squared kernel that NOMINATES minimal/borderline samples; the
# verdict/returned distance is recomputed on those with the scalar itself
# (**2 = libm pow rounds 1 ULP apart on rare values).
_PAD_PERIMETER_CACHE: Dict[int, tuple] = {}


def _pad_perimeter_array(pad):
    key = id(pad)
    fp = (pad.pad_number, round(pad.global_x, 9), round(pad.global_y, 9),
          round(pad.size_x, 9), round(pad.size_y, 9))
    hit = _PAD_PERIMETER_CACHE.get(key)
    if hit is not None and hit[0] == fp:
        return hit[1]
    pts = _pad_perimeter_points(pad)
    arr = (np.array([p[0] for p in pts], dtype=np.float64),
           np.array([p[1] for p in pts], dtype=np.float64), pts)
    if len(_PAD_PERIMETER_CACHE) > 4096:
        _PAD_PERIMETER_CACHE.clear()
    _PAD_PERIMETER_CACHE[key] = (fp, arr)
    return arr


def _pad_dist_d2_proxy(xs, ys, pad):
    """Squared-distance PROXY from points to a pad's copper via multiply
    kernels -- monotone with the scalar point_to_pad_distance to within 1 ULP
    (used only to nominate candidates; verdicts recompute the scalar)."""
    polys = getattr(pad, 'polygons', None)
    if polys:
        arrays = _polys_edge_arrays(polys)
        if arrays is None:
            return np.full(xs.shape, np.inf)
        px1, py1, pdx, pdy, plen_sq = arrays
        d2 = _pt_edges_d2(xs[:, None], ys[:, None], px1[None, :], py1[None, :],
                          pdx[None, :], pdy[None, :], plen_sq[None, :]).min(axis=1)
        inside = np.zeros(xs.shape, dtype=bool)
        for poly in polys:
            P = np.asarray(poly, dtype=np.float64)
            if len(P) < 2:
                continue
            yi = P[:, 1][None, :]; yj = np.roll(P[:, 1], 1)[None, :]
            xi = P[:, 0][None, :]; xj = np.roll(P[:, 0], 1)[None, :]
            cond = (yi > ys[:, None]) != (yj > ys[:, None])
            with np.errstate(divide='ignore', invalid='ignore'):
                xint = (xj - xi) * (ys[:, None] - yi) / (yj - yi) + xi
                crossing = cond & (xs[:, None] < xint)
            inside |= (np.count_nonzero(crossing, axis=1) & 1).astype(bool)
        return np.where(inside, 0.0, d2)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0
    x, y = xs, ys
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        dx0 = xs - pad.global_x
        dy0 = ys - pad.global_y
        x = pad.global_x + dx0 * cos_r + dy0 * sin_r
        y = pad.global_y - dx0 * sin_r + dy0 * cos_r
    rel_x = np.abs(x - pad.global_x)
    rel_y = np.abs(y - pad.global_y)
    half_x, half_y = pad.size_x / 2, pad.size_y / 2
    dxe = np.maximum(0.0, rel_x - half_x)
    dye = np.maximum(0.0, rel_y - half_y)
    d = np.sqrt(dxe * dxe + dye * dye)
    if corner_radius > 0:
        inner_x = half_x - corner_radius
        inner_y = half_y - corner_radius
        corner = (rel_x > inner_x) & (rel_y > inner_y)
        cdx = rel_x - inner_x
        cdy = rel_y - inner_y
        d = np.where(corner,
                     np.maximum(0.0, np.sqrt(cdx * cdx + cdy * cdy) - corner_radius),
                     d)
    return d * d


def check_pad_pad_overlap(pad1: Pad, pad2: Pad, clearance: float,
                          routing_layers: List[str],
                          clearance_margin: float = 0.05
                          ) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
    """Check if two pads of different nets are below clearance (or overlap/short)
    on a shared copper layer (issue #234). KiCad flags these as clearance /
    shorting_items; check_drc previously only had pad-segment and pad-via passes.

    Distance is edge-to-edge, computed by cross-sampling each pad's perimeter
    against the other pad's exact distance function (rect/roundrect/circle +
    rect_rotation, or the real polygon for custom pads -- avoiding the #232
    bounding-box phantom-hit caveat).

    Returns (has_violation, overlap_mm, closest_point).
    """
    l1 = _expand_cu(pad1.layers, routing_layers)
    l2 = _expand_cu(pad2.layers, routing_layers)
    shared = any(l in l2 and l.endswith('.Cu') for l in l1)
    if not shared:
        return False, 0.0, None

    # Nominate minimal samples with the broadcast proxy, then recompute the
    # winners with the scalar in the scalar's own order (pad1 samples then
    # pad2's, strict first-minimum) -- byte-identical returns.
    x1a, y1a, pts1 = _pad_perimeter_array(pad1)
    x2a, y2a, pts2 = _pad_perimeter_array(pad2)
    d2a = _pad_dist_d2_proxy(x1a, y1a, pad2) if len(x1a) else np.empty(0)
    d2b = _pad_dist_d2_proxy(x2a, y2a, pad1) if len(x2a) else np.empty(0)
    m = min(d2a.min() if len(d2a) else np.inf, d2b.min() if len(d2b) else np.inf)
    best = float('inf')
    best_pt = None
    if np.isfinite(m):
        thr = m + 8 * np.spacing(m)
        for d2, pts, other in ((d2a, pts1, pad2), (d2b, pts2, pad1)):
            if not len(d2):
                continue
            for i in np.nonzero(d2 <= thr)[0]:
                px, py = pts[i]
                d = point_to_pad_distance(px, py, other)
                if d < best:
                    best = d
                    best_pt = (px, py)

    overlap = clearance - best
    if overlap > _grade_tol(clearance, clearance_margin):
        return True, overlap, best_pt
    return False, 0.0, None


def _net_tie_span_waived(pcb_data, seg, seg_net: int, partner_pad, clearance: float) -> bool:
    """True when a tie-net segment's collision with its net-tie PARTNER pad is
    the KiCad-legal local contact: every sample of the segment whose copper
    reaches the partner keeps that copper within the tied net's OWN pad of the
    same footprint group (KiCad DRC_ENGINE::IsNetTieExclusion -- the collision
    point must lie on the own pad)."""
    fp = pcb_data.footprints.get(getattr(partner_pad, 'component_ref', None))
    if fp is None or not getattr(fp, 'net_tie_groups', None):
        return False
    by_num = {}
    for p in fp.pads:
        by_num.setdefault(p.pad_number, []).append(p)
    own_pads = []
    for group in fp.net_tie_groups:
        members = [p for num in group for p in by_num.get(num, [])]
        if any(p is partner_pad for p in members):
            own_pads.extend(p for p in members if p.net_id == seg_net)
    if not own_pads:
        return False
    # The collision positions are where the track's COPPER meets the partner
    # pad -- approximated per sample by the closest point ON the partner (the
    # sample itself when inside). KiCad waives when the contact lies on the
    # own pad; near-clearance spans whose contact projection stays on the own
    # pad are accepted (the human cynthion escape), while a track ploughing
    # through the partner's heart has interior contact points far from the
    # own pad and stays flagged.
    def _cp_on_partner(px, py):
        rot = getattr(partner_pad, 'rect_rotation', 0.0) or 0.0
        dx, dy = px - partner_pad.global_x, py - partner_pad.global_y
        if rot:
            r = math.radians(rot)
            c, s = math.cos(r), math.sin(r)
            lx, ly = dx * c + dy * s, -dx * s + dy * c
        else:
            lx, ly = dx, dy
        hx, hy = partner_pad.size_x / 2, partner_pad.size_y / 2
        qx, qy = max(-hx, min(hx, lx)), max(-hy, min(hy, ly))
        if rot:
            r = math.radians(rot)
            c, s = math.cos(r), math.sin(r)
            return (partner_pad.global_x + qx * c - qy * s,
                    partner_pad.global_y + qx * s + qy * c)
        return (partner_pad.global_x + qx, partner_pad.global_y + qy)

    half_w = seg.width / 2
    n = max(2, int(math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y) / 0.01) + 1)
    eps = 0.011  # one sample step + KiCad's collision epsilon headroom
    contacts = []
    best_d, best_pt = 1e9, None
    for i in range(n + 1):
        t = i / n
        px = seg.start_x + (seg.end_x - seg.start_x) * t
        py = seg.start_y + (seg.end_y - seg.start_y) * t
        d = point_to_pad_distance(px, py, partner_pad)
        if d < half_w:  # copper genuinely reaches the partner here
            contacts.append(_cp_on_partner(px, py))
        if d < best_d:
            best_d, best_pt = d, (px, py)
    if not contacts:
        # Pure clearance graze, no copper contact: test the deepest sample's
        # projection onto the partner.
        if best_pt is None:
            return False
        contacts = [_cp_on_partner(best_pt[0], best_pt[1])]
    return all(
        any(point_to_pad_distance(cx, cy, own) <= eps for own in own_pads)
        for cx, cy in contacts)


def check_pad_segment_overlap(pad: Pad, seg: Segment, clearance: float,
                               routing_layers: List[str],
                               clearance_margin: float = 0.05) -> Tuple[bool, float, Optional[Tuple[float, float]]]:
    """Check if a segment is too close to a pad on the same layer.

    Args:
        pad: Pad object with global_x, global_y, size_x, size_y, layers
        seg: Segment to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm, closest_point_on_segment)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = _expand_cu(pad.layers, routing_layers)

    # Check if segment is on a layer the pad is on
    if seg.layer not in expanded_layers:
        return False, 0.0, None

    # Custom comb/finger pads: measure against the real copper polygon(s), not the
    # bounding box, so a track legitimately threading a finger channel is not
    # flagged (issue #188).
    pad_polys = getattr(pad, 'polygons', None)
    if pad_polys:
        dist_to_pad, closest_pt = _segment_to_polys_distance(
            seg.start_x, seg.start_y, seg.end_x, seg.end_y, pad_polys)
        required_dist = seg.width / 2 + clearance
        overlap = required_dist - dist_to_pad
        if overlap > _grade_tol(clearance, clearance_margin):
            return True, overlap, closest_pt
        return False, 0.0, None

    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(pad.size_x, pad.size_y) / 2
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0.0

    # Calculate distance from segment to rectangular pad (with optional rounded
    # corners). For diagonal pads, rotate the segment into the pad's local frame
    # so the axis-aligned rect distance is exact (distance is rotation-invariant).
    sx0, sy0, ex0, ey0 = seg.start_x, seg.start_y, seg.end_x, seg.end_y
    if pad.rect_rotation:
        rad = math.radians(pad.rect_rotation)
        cos_r, sin_r = math.cos(rad), math.sin(rad)
        sx0, sy0 = _into_pad_frame(sx0, sy0, pad, cos_r, sin_r)
        ex0, ey0 = _into_pad_frame(ex0, ey0, pad, cos_r, sin_r)
    dist_to_pad, closest_pt = segment_to_rect_distance(
        sx0, sy0, ex0, ey0,
        pad.global_x, pad.global_y, pad.size_x / 2, pad.size_y / 2,
        corner_radius
    )
    if pad.rect_rotation and closest_pt is not None:
        # Report the closest point back in the global frame (R(+rect_rotation)).
        cdx = closest_pt[0] - pad.global_x
        cdy = closest_pt[1] - pad.global_y
        closest_pt = (pad.global_x + cdx * cos_r - cdy * sin_r,
                      pad.global_y + cdx * sin_r + cdy * cos_r)

    # Required clearance: segment half-width + clearance
    # (dist_to_pad is already edge-to-edge from pad)
    required_dist = seg.width / 2 + clearance
    overlap = required_dist - dist_to_pad

    tolerance = _grade_tol(clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap, closest_pt

    return False, 0.0, None


def check_pad_via_overlap(pad: Pad, via: Via, clearance: float,
                          routing_layers: List[str],
                          clearance_margin: float = 0.05) -> Tuple[bool, float]:
    """Check if a via is too close to a pad.

    Args:
        pad: Pad object
        via: Via to check against
        clearance: Minimum clearance in mm
        routing_layers: List of routing layer names (for expanding *.Cu wildcards)
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm)
    """
    # Expand pad layers (handles *.Cu wildcards)
    expanded_layers = _expand_cu(pad.layers, routing_layers)

    # Vias are through-hole, so they conflict with pads on any copper layer
    if not any(layer.endswith('.Cu') for layer in expanded_layers):
        return False, 0.0

    # Distance from via center to pad edge. Handles custom comb/finger polygons
    # (issue #188), rounded corners, and diagonal (rect_rotation) pads.
    dist_to_pad = point_to_pad_distance(via.x, via.y, pad)

    # Required clearance: via half-size + clearance
    # (dist_to_pad is already edge-to-edge from pad)
    required_dist = via.size / 2 + clearance
    overlap = required_dist - dist_to_pad

    tolerance = _grade_tol(clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap

    return False, 0.0


def check_via_drill_overlap(via1: Via, via2: Via, hole_to_hole_clearance: float,
                            clearance_margin: float = 0.05) -> Tuple[bool, float]:
    """Check if two via drill holes violate hole-to-hole clearance.

    Args:
        via1, via2: Via objects with drill attribute
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm)
    """
    # Required distance between drill hole centers
    required_dist = via1.drill / 2 + via2.drill / 2 + hole_to_hole_clearance
    actual_dist = math.sqrt((via1.x - via2.x)**2 + (via1.y - via2.y)**2)
    overlap = required_dist - actual_dist

    tolerance = _grade_tol(hole_to_hole_clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_pad_drill_via_overlap(pad: Pad, via: Via, hole_to_hole_clearance: float,
                                clearance_margin: float = 0.05) -> Tuple[bool, float]:
    """Check if a via drill hole is too close to a pad's drill hole.

    Args:
        pad: Pad object with drill attribute (through-hole pad)
        via: Via to check against
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm)
    """
    if pad.drill <= 0:
        return False, 0.0  # SMD pad, no drill

    # Distance from the via's hole to the pad's drill CAPSULE (slot-aware:
    # a slot's hole edge follows its axis, not a long-dimension circle).
    from kicad_parser import pad_drill_capsule
    (c1x, c1y), (c2x, c2y), prad = pad_drill_capsule(pad)
    required_dist = prad + via.drill / 2 + hole_to_hole_clearance
    ddx, ddy = c2x - c1x, c2y - c1y
    len2 = ddx * ddx + ddy * ddy
    if len2 > 0:
        t = max(0.0, min(1.0, ((via.x - c1x) * ddx + (via.y - c1y) * ddy) / len2))
        actual_dist = math.sqrt((via.x - (c1x + t * ddx))**2 + (via.y - (c1y + t * ddy))**2)
    else:
        actual_dist = math.sqrt((pad.global_x - via.x)**2 + (pad.global_y - via.y)**2)
    overlap = required_dist - actual_dist

    tolerance = _grade_tol(hole_to_hole_clearance, clearance_margin)
    if overlap > tolerance:
        return True, overlap
    return False, 0.0


def check_segment_board_edge(seg: Segment, board_bounds: Tuple[float, float, float, float],
                             clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float, str]:
    """Check if a segment is too close to the board edge.

    Args:
        seg: Segment to check
        board_bounds: (min_x, min_y, max_x, max_y) of the board
        clearance: Minimum clearance from board edge in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm, edge_name)
    """
    min_x, min_y, max_x, max_y = board_bounds
    half_width = seg.width / 2
    required_clearance = clearance + half_width
    tolerance = _grade_tol(clearance, clearance_margin)

    # Report the WORST overlap over all endpoints x edges (not the first match,
    # so a strict margin=0 result can derive any margined verdict exactly).
    best_overlap, best_edge = 0.0, ""
    for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
        for dist, name in ((x - min_x, "left"), (max_x - x, "right"),
                           (y - min_y, "bottom"), (max_y - y, "top")):
            overlap = required_clearance - dist
            if overlap > tolerance and overlap > best_overlap:
                best_overlap, best_edge = overlap, name
    if best_edge:
        return True, best_overlap, best_edge
    return False, 0.0, ""


def check_via_board_edge(via: Via, board_bounds: Tuple[float, float, float, float],
                         clearance: float, clearance_margin: float = 0.05) -> Tuple[bool, float, str]:
    """Check if a via is too close to the board edge.

    Args:
        via: Via to check
        board_bounds: (min_x, min_y, max_x, max_y) of the board
        clearance: Minimum clearance from board edge in mm
        clearance_margin: Fraction of clearance to use as tolerance (default 0.05 = 5%).

    Returns:
        (has_violation, overlap_mm, edge_name)
    """
    min_x, min_y, max_x, max_y = board_bounds
    half_size = via.size / 2
    required_clearance = clearance + half_size
    tolerance = _grade_tol(clearance, clearance_margin)

    x, y = via.x, via.y

    # Report the WORST overlap over all edges (not the first match, so a
    # strict margin=0 result can derive any margined verdict exactly).
    best_overlap, best_edge = 0.0, ""
    for dist, name in ((x - min_x, "left"), (max_x - x, "right"),
                       (y - min_y, "bottom"), (max_y - y, "top")):
        overlap = required_clearance - dist
        if overlap > tolerance and overlap > best_overlap:
            best_overlap, best_edge = overlap, name
    if best_edge:
        return True, best_overlap, best_edge

    return False, 0.0, ""


# --- Real-outline board-edge geometry (issue #236) ---------------------------
# The bbox checks above measure to the rectangular board extent. On a board with
# an internal cutout, slot, or notch, copper routed INTO the cutout sits inside
# the bbox and is never flagged. These helpers measure to the actual Edge.Cuts
# outline (outer ring + interior cutouts), matching KiCad's copper_edge_clearance.

def board_edge_geometry(board_info) -> Tuple[List[List[Tuple[float, float]]],
                                             Optional[List[Tuple[float, float]]],
                                             List[List[Tuple[float, float]]]]:
    """Return (edge_rings, outer_outline, cutouts) for the real Edge.Cuts.

    edge_rings is the flat list of closed vertex rings (outer outline + each
    cutout) to measure clearance against; outer_outline / cutouts are returned
    separately for the on-board (inside-outline, outside-cutouts) test. Any ring
    is returned only if it has >=3 vertices; an empty edge_rings means the parser
    found no usable outline and the caller should fall back to the bbox checks.
    """
    outlines = [o for o in (getattr(board_info, 'board_outlines', None) or [])
                if len(o) >= 3]
    if not outlines:
        outline = getattr(board_info, 'board_outline', None) or []
        outlines = [outline] if len(outline) >= 3 else []
    cutouts = [c for c in (getattr(board_info, 'board_cutouts', None) or []) if len(c) >= 3]
    # `outer` is a LIST of outer rings (multi-outline boards, issue #304:
    # split keyboards carry several disjoint outlines in one file);
    # _point_on_board accepts either form. None when the board has no outline.
    outer = outlines or None
    rings = list(outlines)
    rings.extend(cutouts)
    # Milled inner contours (#505): Edge.Cuts geometry that is NOT a hole and
    # NOT an outer ring -- a pad-containing inner outline that
    # drop_pad_containing_cutouts reclassified. Copper must hold its edge
    # clearance from them, so they belong in `rings`; they must stay OUT of
    # `cutouts`, or _point_on_board would call every pad they enclose off-board
    # (the #291 regression this reclassification exists to avoid).
    rings.extend(c for c in (getattr(board_info, 'board_edge_contours', None) or [])
                 if len(c) >= 3)
    return rings, outer, cutouts


# Sweep item 1 (#625 follow-up): the board-edge pass calls the two ring
# distances for EVERY segment, via, and pad perimeter sample with no
# prefilter, and a curved Edge.Cuts outline tessellates to 1-2k edges --
# 20-40M scalar seg-seg calls per full-board DRC. The rings are fixed for a
# whole check run, so their edge arrays are memoized (id + fingerprint --
# the fingerprint revalidates against id reuse) and each query is one
# broadcast of the exact scalar formulas. Rings with fewer than 32 edges
# (plain rectangular outlines) keep the scalar loop: numpy call overhead
# would make them slower, and both paths return identical values.
_RINGS_EDGE_CACHE: Dict[int, tuple] = {}
_RINGS_VECTOR_MIN_EDGES = 32


def _rings_edge_arrays(rings):
    """(ex1, ey1, dx, dy, len_sq) float64 arrays over every ring edge."""
    key = id(rings)
    fp = (len(rings), tuple(len(r) for r in rings),
          tuple(rings[0][0]) if rings and len(rings[0]) else None)
    hit = _RINGS_EDGE_CACHE.get(key)
    if hit is not None and hit[0] == fp:
        return hit[1]
    ex1, ey1, ex2, ey2 = [], [], [], []
    for ring in rings:
        P = np.asarray(ring, dtype=np.float64)
        Q = np.roll(P, -1, axis=0)
        ex1.append(P[:, 0]); ey1.append(P[:, 1])
        ex2.append(Q[:, 0]); ey2.append(Q[:, 1])
    x1 = np.concatenate(ex1); y1 = np.concatenate(ey1)
    x2 = np.concatenate(ex2); y2 = np.concatenate(ey2)
    dx, dy = x2 - x1, y2 - y1
    arrays = (x1, y1, x2, y2, dx, dy, dx * dx + dy * dy)
    if len(_RINGS_EDGE_CACHE) > 8:
        _RINGS_EDGE_CACHE.clear()
    _RINGS_EDGE_CACHE[key] = (fp, arrays)
    return arrays


def _pt_edges_d2(px, py, x1, y1, dx, dy, len_sq):
    """Squared point-to-segment distance against every ring edge -- the
    point_to_segment_distance formula, term for term (proj = x1 + t*dx, then
    px - proj: the association matters, (px-x1) - t*dx rounds 1-2 ULP apart)."""
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.clip(((px - x1) * dx + (py - y1) * dy) / len_sq, 0.0, 1.0)
    t = np.where(len_sq < 1e-10, 0.0, t)
    ddx = px - (x1 + t * dx)
    ddy = py - (y1 + t * dy)
    return ddx * ddx + ddy * ddy


def _point_to_rings_distance(x: float, y: float,
                             rings: List[List[Tuple[float, float]]]) -> float:
    """Min distance from a point to any edge ring's boundary."""
    if sum(len(r) for r in rings) >= _RINGS_VECTOR_MIN_EDGES:
        x1, y1, x2, y2, dx, dy, len_sq = _rings_edge_arrays(rings)
        if not len(x1):
            return float('inf')
        # The vector kernel squares with a multiply; the scalar squares with
        # `**2` = libm pow, which rounds 1 ULP differently on ~0.1% of values
        # (macOS arm64) and no numpy op reproduces it. So the broadcast only
        # NOMINATES the minimal edges (a few-ULP window); the returned value
        # is recomputed on those with the scalar itself -- byte-identical by
        # construction, and the candidate set is 1-2 edges.
        d2 = _pt_edges_d2(x, y, x1, y1, dx, dy, len_sq)
        m = d2.min()
        cand = np.nonzero(d2 <= m + 8 * np.spacing(m))[0]
        return min(point_to_segment_distance(x, y, x1[i], y1[i], x2[i], y2[i])
                   for i in cand)
    best = float('inf')
    for ring in rings:
        n = len(ring)
        for i in range(n):
            x1, y1 = ring[i]
            x2, y2 = ring[(i + 1) % n]
            d = point_to_segment_distance(x, y, x1, y1, x2, y2)
            if d < best:
                best = d
    return best


def _segment_to_rings_distance(x1: float, y1: float, x2: float, y2: float,
                               rings: List[List[Tuple[float, float]]]) -> float:
    """Min distance from a track segment to any edge ring's boundary (0 if it
    crosses an edge)."""
    if sum(len(r) for r in rings) >= _RINGS_VECTOR_MIN_EDGES:
        ex1, ey1, ex2, ey2, dx, dy, len_sq = _rings_edge_arrays(rings)
        if not len(ex1):
            return float('inf')
        # segments_intersect, vectorized: the same four ccw() comparisons.
        def _ccw(ax, ay, bx, by, cx, cy):
            return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)
        inter = ((_ccw(x1, y1, ex1, ey1, ex2, ey2)
                  != _ccw(x2, y2, ex1, ey1, ex2, ey2))
                 & (_ccw(x1, y1, x2, y2, ex1, ey1)
                    != _ccw(x1, y1, x2, y2, ex2, ey2)))
        if inter.any():
            # An intersected edge contributes exactly 0.0 to the scalar min.
            return 0.0
        # Endpoint-to-other-segment distances, both directions (the scalar
        # min(d1..d4)), on squared values. As in _point_to_rings_distance,
        # the multiply-squared kernel only NOMINATES minimal edges; the
        # returned value comes from the scalar on those (pow-vs-multiply
        # rounds 1 ULP apart on rare values).
        sdx, sdy = x2 - x1, y2 - y1
        slen_sq = sdx * sdx + sdy * sdy
        d2 = np.minimum(
            np.minimum(_pt_edges_d2(x1, y1, ex1, ey1, dx, dy, len_sq),
                       _pt_edges_d2(x2, y2, ex1, ey1, dx, dy, len_sq)),
            np.minimum(
                _pt_edges_d2(ex1, ey1, np.float64(x1), np.float64(y1),
                             np.float64(sdx), np.float64(sdy), np.float64(slen_sq)),
                _pt_edges_d2(ex2, ey2, np.float64(x1), np.float64(y1),
                             np.float64(sdx), np.float64(sdy), np.float64(slen_sq))))
        m = d2.min()
        cand = np.nonzero(d2 <= m + 8 * np.spacing(m))[0]
        return min(_seg_seg_dist_coords(x1, y1, x2, y2,
                                        ex1[i], ey1[i], ex2[i], ey2[i])
                   for i in cand)
    best = float('inf')
    for ring in rings:
        n = len(ring)
        for i in range(n):
            ex1, ey1 = ring[i]
            ex2, ey2 = ring[(i + 1) % n]
            d = _seg_seg_dist_coords(x1, y1, x2, y2, ex1, ey1, ex2, ey2)
            if d < best:
                best = d
    return best


def _point_on_board(x: float, y: float, outer, cutouts) -> bool:
    """True if (x, y) is on copper-bearing board: inside AN outer outline and
    not inside any cutout. A point off-board / inside a cutout is a hard edge
    violation regardless of its distance to the nearest edge segment.

    ``outer`` may be one vertex ring or a LIST of outer rings (multi-outline
    boards, issue #304); on-board means inside ANY of them.
    """
    if outer:
        outers = [outer] if isinstance(outer[0], tuple) else outer
        if not any(_point_in_poly(x, y, o) for o in outers):
            return False
    for cut in cutouts:
        if _point_in_poly(x, y, cut):
            return False
    return True


# A point must be at least this far outside the Edge.Cuts outline (or inside a
# cutout) before routing treats it as unreachable and skips it (#291). Pads ON
# or grazing the outline (castellated / edge connectors) stay routable.
OFF_BOARD_TOLERANCE = 0.5


def make_off_board_test(board_info, tolerance: float = OFF_BOARD_TOLERANCE):
    """Return f(x, y) -> True when the point is clearly off the copper-bearing
    board: outside the Edge.Cuts outline (or inside a cutout) by more than
    `tolerance` mm, falling back to the board_bounds bbox when the parser found
    no usable outline. Returns None when the board has neither (no test
    possible).

    Routing pre-flights use this to treat off-board pads as unreachable (issue
    #291): the routable area stops at the board edge, so copper drawn toward an
    off-board pad -- or between two off-board pads, beyond the edge keep-out's
    blocked band -- can only end as board-edge DRC.
    """
    rings, outer, cutouts = board_edge_geometry(board_info)
    if rings:
        def test(x: float, y: float) -> bool:
            if _point_on_board(x, y, outer, cutouts):
                return False
            return _point_to_rings_distance(x, y, rings) > tolerance
        return test
    bounds = getattr(board_info, 'board_bounds', None)
    if bounds:
        min_x, min_y, max_x, max_y = bounds
        def test(x: float, y: float) -> bool:
            return (x < min_x - tolerance or x > max_x + tolerance or
                    y < min_y - tolerance or y > max_y + tolerance)
        return test
    return None


def check_segment_board_edge_poly(seg: Segment, rings, outer, cutouts,
                                   clearance: float,
                                   clearance_margin: float = 0.05
                                   ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a track measured against the real Edge.Cuts."""
    required = clearance + seg.width / 2
    tolerance = _grade_tol(clearance, clearance_margin)
    # A track endpoint off-board / inside a cutout is a definite violation.
    for x, y in [(seg.start_x, seg.start_y), (seg.end_x, seg.end_y)]:
        if not _point_on_board(x, y, outer, cutouts):
            dist = _point_to_rings_distance(x, y, rings)
            return True, required + dist, "off-board"
    dist = _segment_to_rings_distance(seg.start_x, seg.start_y, seg.end_x, seg.end_y, rings)
    overlap = required - dist
    if overlap > tolerance:
        return True, overlap, "edge"
    return False, 0.0, ""


def check_via_board_edge_poly(via: Via, rings, outer, cutouts,
                              clearance: float,
                              clearance_margin: float = 0.05
                              ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a via measured against the real Edge.Cuts."""
    required = clearance + via.size / 2
    tolerance = _grade_tol(clearance, clearance_margin)
    if not _point_on_board(via.x, via.y, outer, cutouts):
        dist = _point_to_rings_distance(via.x, via.y, rings)
        return True, required + dist, "off-board"
    dist = _point_to_rings_distance(via.x, via.y, rings)
    overlap = required - dist
    if overlap > tolerance:
        return True, overlap, "edge"
    return False, 0.0, ""


def check_pad_board_edge(pad: Pad, rings, outer, cutouts,
                         clearance: float, board_bounds,
                         clearance_margin: float = 0.05
                         ) -> Tuple[bool, float, str]:
    """Board-edge clearance for a pad (issue #236). Measures the pad copper edge
    (sampled perimeter, or the bbox-rectangle distance when no outline exists)
    to the real Edge.Cuts. Returns (has_violation, overlap_mm, edge)."""
    tolerance = _grade_tol(clearance, clearance_margin)
    if rings:
        best = float('inf')
        off_board = False
        for px, py in _pad_perimeter_points(pad):
            if not _point_on_board(px, py, outer, cutouts):
                off_board = True
            d = _point_to_rings_distance(px, py, rings)
            if d < best:
                best = d
        if off_board:
            return True, clearance + best, "off-board"
        overlap = clearance - best
        if overlap > tolerance:
            return True, overlap, "edge"
        return False, 0.0, ""

    # No usable outline -> bbox fallback, measured to the pad copper edge.
    if board_bounds is None:
        return False, 0.0, ""
    min_x, min_y, max_x, max_y = board_bounds
    best = float('inf')
    edge = ""
    for px, py in _pad_perimeter_points(pad):
        for d, name in ((px - min_x, "left"), (max_x - px, "right"),
                        (py - min_y, "bottom"), (max_y - py, "top")):
            if d < best:
                best = d
                edge = name
    overlap = clearance - best
    if overlap > tolerance:
        return True, overlap, edge
    return False, 0.0, ""


def check_track_width(seg: Segment, min_track_width: float,
                      size_margin: float = 0.0) -> Tuple[bool, float]:
    """Check if a segment is thinner than the minimum manufacturable track width.

    Args:
        seg: Segment to check
        min_track_width: Minimum allowed track width in mm
        size_margin: Absolute tolerance in mm; widths within this of the floor pass

    Returns:
        (is_too_thin, shortfall_mm)
    """
    shortfall = min_track_width - seg.width
    if shortfall > size_margin:
        return True, shortfall
    return False, 0.0


def check_via_size(via: Via, min_via_diameter: float, min_via_drill: float,
                   size_margin: float = 0.0) -> Tuple[bool, bool, float, float]:
    """Check if a via's outer diameter or drill is below the fab floor.

    Returns:
        (diameter_too_small, drill_too_small, dia_shortfall_mm, drill_shortfall_mm)
    """
    dia_short = min_via_diameter - via.size
    drill_short = min_via_drill - via.drill
    dia_bad = dia_short > size_margin
    drill_bad = drill_short > size_margin
    return dia_bad, drill_bad, dia_short, drill_short


def write_debug_lines(pcb_file: str, violations: List[dict], clearance: float, layer: str = "User.7"):
    """Write debug lines to PCB file showing violation locations.

    Adds gr_line elements connecting closest points of violating segments.
    """
    import uuid

    # Read the PCB file
    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate gr_line elements for segment-segment violations
    debug_lines = []
    print(f"\nDebug lines (center-to-center distance, required clearance = {clearance}mm):")
    for v in violations:
        if v['type'] in ('segment-segment', 'segment-segment-track-rule') \
                and 'closest_pt1' in v and v['closest_pt1']:
            pt1 = v['closest_pt1']
            pt2 = v['closest_pt2']
            dist = math.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
            # Track width is typically 0.1mm, so required center-to-center = 0.1 + clearance = 0.2mm
            required = 0.1 + clearance  # half-width + half-width + clearance = track_width + clearance
            violation_amt = required - dist
            print(f"  {v['net1']} <-> {v['net2']}: dist={dist:.4f}mm, required={required:.3f}mm, violation={violation_amt:.4f}mm")
            print(f"    from ({pt1[0]:.4f}, {pt1[1]:.4f}) to ({pt2[0]:.4f}, {pt2[1]:.4f})")

            line = f'''\t(gr_line
\t\t(start {pt1[0]:.6f} {pt1[1]:.6f})
\t\t(end {pt2[0]:.6f} {pt2[1]:.6f})
\t\t(stroke
\t\t\t(width 0.05)
\t\t\t(type solid)
\t\t)
\t\t(layer "{layer}")
\t\t(uuid "{uuid.uuid4()}")
\t)'''
            debug_lines.append(line)

    if not debug_lines:
        print(f"No debug lines to write")
        return

    # Insert before the final closing paren
    debug_text = '\n'.join(debug_lines)
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + debug_text + '\n' + content[last_paren:]

    with open(pcb_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"\nWrote {len(debug_lines)} debug line(s) to layer {layer}")


def _edge_phrase(edge: str) -> str:
    """Human-readable board-edge violation phrase from the edge label."""
    if edge == 'off-board':
        return "off the board / in a cutout"
    if edge in ('', 'edge'):
        return "too close to board edge"
    return f"too close to {edge} board edge"  # bbox fallback: left/right/top/bottom


def edge_clearance_severity(pcb_file: str) -> Optional[str]:
    """Return the board's ``copper_edge_clearance`` DRC severity from the sibling
    .kicad_pro ('error' / 'warning' / 'ignore'), or None when unset / no project.

    KiCad's DRC does NOT run a rule whose severity is 'ignore' -- not even
    ``kicad-cli pcb drc --severity-all`` (that flag surfaces every ENABLED
    severity, it does not re-enable a disabled rule). So a board whose author
    set copper_edge_clearance to 'ignore' (common on castellated / edge-connector
    / route-to-edge hobby boards -- e.g. nrfmicro) reports ZERO board-edge items
    from the KiCad oracle. check_drc reads the same setting and skips its
    board-edge check to match, instead of manufacturing phantom SEGMENT-BOARD-EDGE
    items the board's own DRC deliberately suppresses (#427)."""
    import os as _os
    import json as _json
    pro = _os.path.splitext(pcb_file)[0] + '.kicad_pro'
    try:
        with open(pro, encoding='utf-8') as f:
            j = _json.load(f)
    except (OSError, ValueError):
        return None
    return (((j.get('board', {}) or {}).get('design_settings', {}) or {})
            .get('rule_severities', {}) or {}).get('copper_edge_clearance')


def _np_capsule_to_tracks(h1x, h1y, h2x, h2y,
                          sx1, sy1, sx2, sy2, dx, dy, safe_len2):
    """Vectorized min distance from a drill CAPSULE's axis ((h1),(h2)) to every
    track segment (arrays as built by the copper-to-hole check); 0 for proper
    crossings (orientation sign test). Round drills pass a zero-length axis.
    Shared by the copper-to-hole check and the NPTH-slot edge check (#448)."""
    def _pts(hx, hy):
        t = np.clip(((hx - sx1) * dx + (hy - sy1) * dy) / safe_len2, 0.0, 1.0)
        return np.sqrt((hx - (sx1 + t * dx)) ** 2 + (hy - (sy1 + t * dy)) ** 2)
    dist = np.minimum(_pts(h1x, h1y), _pts(h2x, h2y))
    hdx, hdy = h2x - h1x, h2y - h1y
    hlen2 = hdx * hdx + hdy * hdy
    if hlen2 > 1e-12:
        for px, py in ((sx1, sy1), (sx2, sy2)):
            t2 = np.clip(((px - h1x) * hdx + (py - h1y) * hdy) / hlen2, 0.0, 1.0)
            dist = np.minimum(dist, np.sqrt((px - (h1x + t2 * hdx)) ** 2 +
                                            (py - (h1y + t2 * hdy)) ** 2))
        d1 = dx * (h1y - sy1) - dy * (h1x - sx1)
        d2 = dx * (h2y - sy1) - dy * (h2x - sx1)
        d3 = hdx * (sy1 - h1y) - hdy * (sx1 - h1x)
        d4 = hdx * (sy2 - h1y) - hdy * (sx2 - h1x)
        dist = np.where((d1 * d2 < 0) & (d3 * d4 < 0), 0.0, dist)
    return dist


def _violation_xy(v: dict) -> Optional[Tuple[float, float]]:
    """One representative board coordinate for a violation record. The record
    shapes vary by check; try the point-bearing keys, midpoint a 4-tuple."""
    for k in ('closest_pt1', 'cross_point', 'via_loc', 'pad_loc',
              'loc1', 'seg_loc', 'loc2'):
        p = v.get(k)
        if not p:
            continue
        try:
            if len(p) >= 4:
                return ((p[0] + p[2]) / 2.0, (p[1] + p[3]) / 2.0)
            if len(p) >= 2:
                return (float(p[0]), float(p[1]))
        except (TypeError, ValueError):
            continue
    return None


def render_violation_panels(pcb_file: str, violations: List[dict],
                            out_dir: str, size: int = 900,
                            margin_mm: float = 1.5, cluster_gap: float = 4.0,
                            max_panels: int = 8) -> List[str]:
    """Question-scoped crops of the violation clusters (run-6 image work).

    Nothing in the toolchain drew DRC positions before this: the numbers said
    "278 violations" and the picture that would show WHERE never existed, so
    run 5 re-derived every cluster from coordinates by hand. One panel per
    spatial cluster (greedy merge within ``cluster_gap`` mm), largest cluster
    first, each labeled with the count, the dominant types and the rect --
    the paired number rides in the caption, so the "no clearance from pixels"
    rule holds: the panel shows WHERE, the record says HOW MUCH.

    Returns the panel paths (also printed). Accepted (waived) records are
    skipped. Degrades to [] with a note if Pillow is unavailable.
    """
    pts = []
    for v in violations:
        if v.get('accepted'):
            continue
        xy = _violation_xy(v)
        if xy is not None:
            pts.append((xy[0], xy[1], v))
    if not pts:
        return []
    try:
        from route_render import (BoardRenderer, mm_ruler_overlay,
                                  ref_label_overlay)
        from kicad_parser import parse_kicad_pcb
    except Exception as e:
        print(f"  (--render skipped: {e})")
        return []
    # Greedy spatial clustering: a point joins the first cluster whose bbox it
    # sits within cluster_gap of; clusters are merged transitively enough for
    # a triage picture (this mirrors render_placement's focus-gap approach).
    clusters: List[dict] = []
    for x, y, v in sorted(pts, key=lambda p: (p[0], p[1])):
        home = None
        for c in clusters:
            if (c['x0'] - cluster_gap <= x <= c['x1'] + cluster_gap and
                    c['y0'] - cluster_gap <= y <= c['y1'] + cluster_gap):
                home = c
                break
        if home is None:
            home = {'x0': x, 'y0': y, 'x1': x, 'y1': y, 'items': []}
            clusters.append(home)
        home['x0'] = min(home['x0'], x); home['y0'] = min(home['y0'], y)
        home['x1'] = max(home['x1'], x); home['y1'] = max(home['y1'], y)
        home['items'].append((x, y, v))

    clusters.sort(key=lambda c: -len(c['items']))
    os.makedirs(out_dir, exist_ok=True)
    pcb = parse_kicad_pcb(pcb_file)
    dropped = sum(len(c['items']) for c in clusters[max_panels:])
    paths = []
    for i, c in enumerate(clusters[:max_panels]):
        view = (c['x0'] - margin_mm, c['y0'] - margin_mm,
                c['x1'] + margin_mm, c['y1'] + margin_mm)
        r = BoardRenderer(pcb, size=size, view=view)
        marker_pts = [(x, y) for x, y, _ in c['items']]

        def _marks(d, rr, _pts=marker_pts):
            rad = max(3.0, rr.tf.length(0.25))
            for mx, my in _pts:
                px, py = rr.tf.pt(mx, my)
                d.ellipse([px - rad, py - rad, px + rad, py + rad],
                          outline=(255, 60, 60), width=max(2, int(rad / 3)))
        from collections import Counter
        types = Counter(v.get('type', '?') for _, _, v in c['items'])
        top = ', '.join(f"{t} x{n}" for t, n in types.most_common(3))
        label = (f"drc {len(c['items'])}: {top} @"
                 f"({c['x0']:.1f},{c['y0']:.1f})-({c['x1']:.1f},{c['y1']:.1f})mm")
        out = os.path.join(out_dir, f"drc_cluster{i + 1}.png")
        # Ref labels + mm ruler make the panel matchable to the violation
        # records (which cite nets and coordinates): under the rings, above
        # the copper.
        r.frame(label=label,
                overlays=[ref_label_overlay(pcb), mm_ruler_overlay(),
                          _marks]).save(out)
        paths.append(out)
        print(f"  DRC render: {out} -- {label}")
    if dropped:
        print(f"  DRC render: {dropped} violation(s) in "
              f"{len(clusters) - max_panels} further cluster(s) NOT rendered "
              f"(--render caps at {max_panels} panels)")
    return paths


def run_drc(pcb_file: str, clearance: float = 0.1, net_patterns: Optional[List[str]] = None,
            debug_output: bool = False, quiet: bool = False,
            hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE, board_edge_clearance: float = 0.0,
            hole_clearance: float = 0.0,
            clearance_margin: float = 0.05, max_print: int = 20,
            min_track_width: Optional[float] = None,
            min_via_diameter: Optional[float] = None,
            min_via_drill: Optional[float] = None,
            check_sizes: bool = True, size_margin: float = 0.0,
            check_pad_edge: bool = False, print_summary: bool = True,
            net_clearances: Optional[Dict[str, float]] = None,
            respect_edge_severity: bool = True):
    """Run DRC checks on the PCB file.

    Args:
        pcb_file: Path to the KiCad PCB file
        clearance: Minimum clearance in mm
        net_patterns: Optional list of net name patterns (fnmatch style) to focus on.
                     If provided, only checks involving at least one matching net are reported.
        debug_output: If True, write debug lines to User.7 layer showing violation locations
        quiet: If True, only print a summary line unless there are violations
        hole_to_hole_clearance: Minimum clearance between drill hole edges in mm
            (default: the fab floor, routing_defaults.HOLE_TO_HOLE_CLEARANCE)
        board_edge_clearance: Minimum clearance from board edge in mm (0 = use clearance)
        min_track_width: Minimum manufacturable track width in mm. None = derive the
            JLC fab floor from the board's copper-layer count (issue #176).
        min_via_diameter: Minimum via outer diameter in mm. None = fab floor.
        min_via_drill: Minimum via drill in mm. None = fab floor.
        check_sizes: If True, flag tracks/vias below the fab floor (track-width and
            via/hole size checks). These catch sub-fab copper that the clearance-only
            checks miss (a board's own min_track_width DRC rule can be lowered to match
            undersized copper, so it never trips -- the fab floor is the real limit).
        size_margin: Absolute tolerance in mm for the size checks; a width/diameter
            within this of the floor is not flagged (default 0 = exact floor).
        net_clearances: Optional {net_name: clearance_mm} from the board's
            netclasses (issue #326). Pair checks grade at
            max(clearance, class(net_a), class(net_b), pad overrides) --
            KiCad's per-pair resolution -- instead of the single global value.
        respect_edge_severity: When True (default), skip the board-edge check on
            boards whose sibling .kicad_pro sets copper_edge_clearance severity to
            'ignore' -- matching the KiCad oracle, which runs no board-edge check
            there (#427). Pass False to force the check regardless of the setting.
    """
    # Board-edge clearance: the explicit/board value when set, else the fab
    # copper-to-edge floor (0.20, #439 -- the routers pin the edge up to it, so a
    # project-less final's copper is >=0.20 from the edge) or the copper clearance,
    # whichever is larger.
    # #441: pin the copper-to-edge grade floor to the fab minimum (fab_tiers
    # board_edge, the active fab tier's copper-to-edge floor) even when the
    # board/CLI declares a smaller one -- a board setting a sub-fab (or 0) edge
    # rule would otherwise grade clean while copper runs to the milled edge. A
    # declared rule ABOVE the fab floor (e.g. 0.5) is honored via max().
    from fix_kicad_drc_settings import fab_edge_floor
    _fab_edge = fab_edge_floor()
    effective_board_edge_clearance = (max(board_edge_clearance, _fab_edge)
                                      if board_edge_clearance > 0 else max(clearance, _fab_edge))
    if quiet and net_patterns:
        # Print a brief summary line in quiet mode
        print(f"Checking {', '.join(net_patterns)} for DRC...", end=" ", flush=True)
    elif not quiet:
        print(f"Loading {pcb_file}...")

    pcb_data = parse_kicad_pcb(pcb_file)
    # #337: unify copper-graphic nets by connectivity before pair checks
    _build_graphic_unification(pcb_data)

    if not quiet:
        print(f"Found {len(pcb_data.segments)} segments and {len(pcb_data.vias)} vias")

    # Pairwise required clearance, KiCad-style (issue #326): each item's own
    # clearance is its netclass value (net_clearances, name-keyed -> resolved
    # to ids here); pads additionally carry their local/footprint override in
    # pad.local_clearance (the parser resolves footprint inheritance). The
    # requirement for a pair is the MAX of the two items' values and the
    # global floor. All zero-cost when the board has neither netclasses nor
    # overrides: _pair_cl returns the global scalar unchanged.
    _ncl_by_id: Dict[int, float] = {}
    if net_clearances:
        for nid, net in pcb_data.nets.items():
            c = net_clearances.get(net.name, 0.0)
            if c > clearance:
                _ncl_by_id[nid] = c

    # #498: per-layer clearance rules from the board's own sibling .kicad_dru,
    # auto-read exactly like netclasses. A rule REPLACES the net/class-resolved
    # pair value on its layer (KiCad precedence: custom rules outrank classes,
    # tightening or relaxing); a pad's local override stays above (KiCad gives
    # local overrides precedence over rules). The router routes to the same map
    # (kicad_dru.install_layer_clearances), so grading without it would
    # manufacture phantom flags on relaxed layers and miss real ones on
    # tightened layers.
    _board_copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    from kicad_dru import read_board_layer_clearances
    _lcl, _dru_notes = read_board_layer_clearances(pcb_file, _board_copper)
    if not quiet:
        for _n in _dru_notes:
            print(f"  .kicad_dru: {_n}")
        if _lcl:
            print("Per-layer clearance rules (.kicad_dru, #498): "
                  + ", ".join(f"{l}:{v:g}" for l, v in sorted(_lcl.items())))

    # Track-scoped clearance rules from the same .kicad_dru (#735). Grader-side
    # they are PAIR-EXACT (a rule binds a specific (a, b) pair, other_only
    # exempts member siblings), which is <= the router's per-obstacle-net
    # over-approximation -- so router output always grades clean. Applied at
    # the SEG-SEG site only (KiCad's Type=='track' binds tracks).
    from kicad_dru import read_board_track_clearances, track_pair_clearance
    _track_rules, _track_notes = read_board_track_clearances(pcb_file)
    _cls_of: Dict[int, set] = {}
    if _track_rules:
        try:
            from list_nets import net_class_memberships
            _cls_of = net_class_memberships(
                pcb_file, {nid: n.name for nid, n in pcb_data.nets.items()
                           if n.name})
        except Exception:
            _track_rules = []
    if not quiet and _track_rules:
        for _n in _track_notes:
            print(f"  .kicad_dru: {_n}")
        print("Track-to-track clearance rules (.kicad_dru): "
              + ", ".join(f"'{r.cls}':{r.clearance_mm:g}"
                          f"{'(other-only)' if r.other_only else ''}"
                          for r in _track_rules))

    def _track_pair_cl(net_a: int, net_b: int, layer: str):
        """Effective seg-seg clearance for the pair, plus the TrackRule
        that RAISED it (None when no track rule binds above the base value).
        The rule identity is what lets the violation record distinguish a
        structural, floor-governed rule pair from a physical graze.

        The binding predicate itself lives in `kicad_dru.track_pair_clearance`
        (#735) so the fanout-clearance connector gate resolves a track pair
        through THIS code rather than a second copy of it. Only the base value
        is this grader's own -- `_pair_cl` reads the netclass/layer state that
        exists nowhere else.

        The empty-list early-out stays HERE rather than inside the resolver:
        this runs per nearby seg-seg pair, and a board with no rules must not
        pay a call for it (the same zero-cost-when-undeclared property the
        netclass and override channels above have)."""
        eff = _pair_cl(net_a, net_b, layer=layer)
        if not _track_rules:
            return eff, None
        return track_pair_clearance(_track_rules, _cls_of.get(net_a, ()),
                                    _cls_of.get(net_b, ()), eff)

    def _layer_cl(layer: str, eff: float) -> float:
        v = _lcl.get(layer) if _lcl else None
        return v if v is not None else eff

    def _stack_cl(eff: float) -> float:
        # Stack-spanning pairs (via-via): the barrels meet on every copper
        # layer -> max over the rules, same formula the router stamps with.
        return max([eff] + list(_lcl.values())) if _lcl else eff

    def _pad_copper(pad):
        return pad_copper_layers(pad, _board_copper)

    def _pads_cl(eff: float, pad, other_pad=None) -> float:
        # Pad-vs-via / pad-vs-pad meet on their SHARED copper layers; TH
        # geometry is identical on every layer, so the max over shared layers
        # is the exact requirement. The body lives at module level
        # (pads_shared_layer_clearance) so placement/legality.py resolves the
        # same rule rather than a hand-mirrored copy -- #697.
        if not _lcl:
            return eff          # strict no-op: expand nothing (see the helper)
        return pads_shared_layer_clearance(
            eff, _lcl, _pad_copper(pad),
            _pad_copper(other_pad) if other_pad is not None else None)

    def _pair_cl(net_a: int, net_b: int, layer: str = None) -> float:
        base = clearance if not _ncl_by_id else \
            max(clearance, _ncl_by_id.get(net_a, 0.0), _ncl_by_id.get(net_b, 0.0))
        if layer is not None:
            return _layer_cl(layer, base)
        return base

    # A pad / footprint clearance OVERRIDE replaces the class/rule value,
    # floored at rules.min_clearance (KiCad 10, measured by
    # tests/oracle/constraint_agreement.py) -- the same helper the router's
    # obstacle stamps use, so the two cannot drift.
    from design_rules import override_clearance as _override_clr, \
        board_min_clearance_for as _bm_for
    _board_min_clr = _bm_for(pcb_data, pcb_file)

    def _pad_pair_cl(pad, other_net: int, layer: str = None, other_pad=None) -> float:
        eff = _pair_cl(pad.net_id, other_net)
        if layer is not None:
            eff = _layer_cl(layer, eff)
        else:
            eff = _pads_cl(eff, pad, other_pad)
        return _override_clr(eff, _board_min_clr, pad, other_pad)

    def _mark_required(v: dict, eff: float) -> dict:
        # Attribute above-global requirements (local override / netclass) in
        # the violation record, mirroring KiCad's "pad clearance X mm" wording.
        if eff > clearance + 1e-9:
            v['required_mm'] = eff
        return v

    # Resolve the fab-floor minimums for the track-width / via-size checks. Default
    # to the JLC manufacturing floor for the board's copper-layer count (issue #176).
    if check_sizes:
        from list_nets import fab_floor_min
        copper_layers = getattr(pcb_data.board_info, 'copper_layers', None) or []
        copper_count = len(copper_layers) if copper_layers else 2
        # Grade against the DEEPEST floor the active fab tier can reach (fab_floor_min):
        # for 'standard' that's the advanced rung it escalates to (0.25 dia / 0.15
        # drill on 4+ layers), so legitimately-escalated fine vias aren't flagged;
        # for 'advanced'/overrides it's the hard floor (issue #237).
        # #857/#530: the PHYSICAL fab floor (the override file, else the
        # advanced rung), not the selected tier's: the tier bounds what the
        # router may descend to on its own, while an explicit --via-size 0.3
        # is accepted as asked, so grading at the tier would flag every via
        # the operator requested. The board's own minimums (below) are what
        # KiCad grades; `--fab-tier advanced` and `--fab-overrides` still
        # tighten this through physical_fab_floor.
        from fab_tiers import physical_fab_floor
        fab = physical_fab_floor(copper_count)
        eff_min_track = min_track_width if min_track_width is not None else fab['track_width']
        eff_min_via_dia = min_via_diameter if min_via_diameter is not None else fab['via_diameter']
        eff_min_via_drill = min_via_drill if min_via_drill is not None else fab['via_drill']
        # #530: the board's OWN size minimums -- Board Setup rules.min_* and
        # every .kicad_dru track_width / via_diameter / hole_size rule, resolved
        # per net and layer in KiCad's order, raised to the fab floor -- are
        # what KiCad grades. An explicit --min-* flag still overrides.
        _size_rules = None
        try:
            from design_rules import DesignRules as _DR
            _size_rules = _DR.from_project(pcb_data, pcb_file, fab_floor=fab,
                                           copper_layers=copper_layers)
            if not (_size_rules.board_min or _size_rules.rules):
                _size_rules = None
        except Exception as _dre:                              # noqa: BLE001
            if not quiet:
                print(f"  (design-rule size floors unavailable: {_dre})")
            _size_rules = None

        def _track_floor(seg):
            if min_track_width is not None or _size_rules is None:
                return eff_min_track
            v = _size_rules.floor('track_width', seg.net_id, seg.layer)
            return v if v is not None else eff_min_track

        def _via_floors(via):
            if _size_rules is None:
                return eff_min_via_dia, eff_min_via_drill
            d = (eff_min_via_dia if min_via_diameter is not None
                 else (_size_rules.floor('via_diameter', via.net_id, type='via') or eff_min_via_dia))
            h = (eff_min_via_drill if min_via_drill is not None
                 else (_size_rules.floor('hole_size', via.net_id, type='via') or eff_min_via_drill))
            return d, h
        if not quiet:
            print(f"Size floors ({copper_count}-layer fab): track >= {eff_min_track}mm, "
                  f"via dia >= {eff_min_via_dia}mm, via drill >= {eff_min_via_drill}mm"
                  + ("; per-net board/rule minimums applied (#530)" if _size_rules else ""))

    # Helper to check if a net_id matches the filter patterns
    def net_matches_filter(net_id: int) -> bool:
        if net_patterns is None:
            return True  # No filter, include all
        net_info = pcb_data.nets.get(net_id, None)
        if net_info is None:
            return False
        return matches_any_pattern(net_info.name, net_patterns)

    if net_patterns and not quiet:
        print(f"Filtering to nets matching: {net_patterns}")

    # Copper layers for pad layer expansion and the per-layer pad/via passes.
    # Use the BOARD's copper layers, not the layers segments happen to sit on:
    # a via can overlap a pad on a layer that carries no tracks yet (e.g. a
    # fanout via ring vs a B.Cu test pad before anything routes on B.Cu), and
    # the segment-derived list silently skipped that layer's checks (#253).
    routing_layers = [l for l in (pcb_data.board_info.copper_layers or [])
                      if l.endswith('.Cu')]
    if not routing_layers:
        # sorted(), NOT list(set(...)): layer names are STRINGS and CPython
        # randomizes string hashing per process, so the fallback order varied
        # run to run (observed: ['B.Cu','F.Cu','In2.Cu','In1.Cu'] vs
        # ['In1.Cu','In2.Cu','B.Cu','F.Cu'] in two runs of the same chain).
        # This list keys _EXPAND_CACHE and drives the per-layer pad/via passes,
        # so a varying order makes DRC results order-dependent.
        routing_layers = sorted(set(seg.layer for seg in pcb_data.segments
                                    if seg.layer.endswith('.Cu')))
    if not routing_layers:
        routing_layers = ['F.Cu', 'B.Cu']  # Fallback

    # Build spatial index for fast proximity queries
    if not quiet:
        print("Building spatial index...")
    spatial_idx = SpatialIndex(cell_size=2.0)  # 2mm cells

    # Add all segments to spatial index
    for seg in pcb_data.segments:
        spatial_idx.add_segment(seg, seg.net_id)

    # Add all vias to spatial index
    for via in pcb_data.vias:
        spatial_idx.add_via(via, via.net_id)

    # Add all pads to spatial index
    pads_by_net = pcb_data.pads_by_net
    for net_id, pads in pads_by_net.items():
        for pad in pads:
            expanded_layers = _expand_cu(pad.layers, routing_layers)
            spatial_idx.add_pad(pad, net_id, expanded_layers)

    # Group vias by net (still needed for some checks)
    vias_by_net = {}
    for via in pcb_data.vias:
        if via.net_id not in vias_by_net:
            vias_by_net[via.net_id] = []
        vias_by_net[via.net_id].append(via)

    violations = []
    _accepted_edge = []  # pad-covered edge items: published (not counted) for kicad_drc_compare

    # Pre-compute matching nets for filtering
    if net_patterns:
        matching_net_ids = set(net_id for net_id in pcb_data.nets.keys() if net_matches_filter(net_id))
        if not quiet:
            print(f"Filtering to {len(matching_net_ids)} matching nets")
    else:
        matching_net_ids = None

    # Check segment-to-segment violations using spatial index
    if not quiet:
        print("\nChecking segment-to-segment clearances...")

    checked_pairs = set()  # Track checked segment pairs to avoid duplicates
    for seg1 in pcb_data.segments:
        net1 = seg1.net_id
        net1_matches = matching_net_ids is None or net1 in matching_net_ids

        # Get nearby segments from spatial index (same layer only)
        for seg2, net2 in spatial_idx.get_nearby_segments(seg1):
            if net1 == net2:
                continue  # Same net
            if seg1 is seg2:
                continue  # Same segment

            # Skip if neither net matches filter
            net2_matches = matching_net_ids is None or net2 in matching_net_ids
            if not net1_matches and not net2_matches:
                continue

            # Avoid checking same pair twice
            pair_key = (min(id(seg1), id(seg2)), max(id(seg1), id(seg2)))
            if pair_key in checked_pairs:
                continue
            checked_pairs.add(pair_key)

            _eff, _trule = _track_pair_cl(net1, net2, layer=seg1.layer)
            has_violation, overlap, pt1, pt2 = check_segment_overlap(seg1, seg2, _eff, clearance_margin)
            if has_violation and _graphic_pair_is_same_net(seg1, seg2, net1, net2):
                has_violation = False
            if has_violation:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                # Track-rule classification: the pair is RULE-governed (not a
                # physical graze) when a track rule raised the clearance AND
                # the copper gap (eff - overlap) still clears the base pair
                # value -- i.e. the violation exists only because of the rule.
                # Those pairs are the structural population a registered
                # check_dru floor gates; graders may treat them as advisory.
                _rule_only = False
                if _trule is not None:
                    _base = _pair_cl(net1, net2, layer=seg1.layer)
                    _rule_only = (_eff - overlap) >= _base * (1 - clearance_margin)
                _v = {
                    'type': ('segment-segment-track-rule' if _rule_only
                             else 'segment-segment'),
                    'net1': net1_str,
                    'net2': net2_str,
                    'layer': seg1.layer,
                    'overlap_mm': overlap,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                    'closest_pt1': pt1,
                    'closest_pt2': pt2,
                }
                if _rule_only:
                    # full rule name; harness floors key on the prefix before
                    # ':' (split(':')[0]) -- keep both derivable.
                    _v['track_rule'] = _trule.name
                violations.append(_mark_required(_v, _eff))

            # Also check for segment crossings (different nets)
            crosses, cross_point = segments_cross(seg1, seg2)
            if crosses:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                violations.append({
                    'type': 'segment-crossing',
                    'net1': net1_str,
                    'net2': net2_str,
                    'layer': seg1.layer,
                    'cross_point': cross_point,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                })

    # Check for same-net segment crossings using spatial index
    if not quiet:
        print("Checking for same-net segment crossings...")
    same_net_checked = set()
    for seg1 in pcb_data.segments:
        net_id = seg1.net_id
        if matching_net_ids is not None and net_id not in matching_net_ids:
            continue
        for seg2, net2 in spatial_idx.get_nearby_segments(seg1):
            if net2 != net_id:
                continue  # Different net
            if seg1 is seg2:
                continue
            pair_key = (min(id(seg1), id(seg2)), max(id(seg1), id(seg2)))
            if pair_key in same_net_checked:
                continue
            same_net_checked.add(pair_key)
            crosses, cross_point = segments_cross(seg1, seg2)
            if crosses:
                net_name = pcb_data.nets.get(net_id, None)
                net_str = net_name.name if net_name else f"net_{net_id}"
                violations.append({
                    'type': 'segment-crossing-same-net',
                    'net1': net_str,
                    'net2': net_str,
                    'layer': seg1.layer,
                    'cross_point': cross_point,
                    'loc1': (seg1.start_x, seg1.start_y, seg1.end_x, seg1.end_y),
                    'loc2': (seg2.start_x, seg2.start_y, seg2.end_x, seg2.end_y),
                })

    # Same-net SOFT JOINT: a DANGLING FREE END (a segment terminus that is NOT a
    # shared vertex, NOT on a via, and NOT on an own-net pad) that reaches the rest
    # of the net ONLY by cap-overlapping another dangling free end. A clean route
    # ends every piece at a coincident vertex, a via, or a pad; a free end floating
    # in copper means the real connecting segment was ripped and never restored (or
    # a tap landed on-grid short of the off-grid endpoint), leaving the net held by
    # a sliver of overlap -- a fragile near-open. Only DANGLING ends qualify, so
    # parallel same-net tracks and normal bends (shared vertices) are not flagged.
    if not quiet:
        print("Checking for same-net soft joints (dangling free ends)...")
    from collections import defaultdict as _dd
    def _rk(x, y):
        return (round(x, 3), round(y, 3))
    _ep_count = _dd(int)
    for s in pcb_data.segments:
        if matching_net_ids is not None and s.net_id not in matching_net_ids:
            continue
        _ep_count[(s.net_id, s.layer, _rk(s.start_x, s.start_y))] += 1
        _ep_count[(s.net_id, s.layer, _rk(s.end_x, s.end_y))] += 1
    _vias_by_net = _dd(list)
    for v in pcb_data.vias:
        _vias_by_net[v.net_id].append(v)
    _copper = list(getattr(pcb_data.board_info, 'copper_layers', None) or ())
    def _at_anchor(nid, x, y, layer, width):
        """Does this end's own COPPER reach a same-net via barrel or pad?

        Shared predicate (connectivity.endpoint_reaches_*), so this stays
        byte-identical to check_weird's soft-joint anchor and to the repair
        pass. The cap is what physically touches, so the cap radius is the
        credit -- for the pad exactly as for the via (#722) -- and the pad
        must actually carry copper on this layer, as check_connected requires.
        """
        r = (width or 0.0) / 2.0
        for v in _vias_by_net.get(nid, []):
            if endpoint_reaches_via(x, y, r, v, (layer,), _copper):
                return True
        for p in pcb_data.pads_by_net.get(nid, []):
            if endpoint_reaches_pad(x, y, r, (layer,), p):
                return True
        return False
    _dangles = _dd(list)  # (net_id, layer) -> [(x, y, width)]
    for s in pcb_data.segments:
        if matching_net_ids is not None and s.net_id not in matching_net_ids:
            continue
        for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            if _ep_count[(s.net_id, s.layer, _rk(x, y))] != 1:
                continue  # shared vertex = clean joint
            if _at_anchor(s.net_id, x, y, s.layer, s.width):
                continue  # terminates on a via / own pad = legitimate
            # A soft joint is a PAIR: a graphic is carried as a flag so
            # only an art-MEETS-art pair is dropped, never the actionable
            # TRACK end paired with art (#337, #722).
            _dangles[(s.net_id, s.layer)].append(
                (x, y, s.width, getattr(s, 'graphic', False)))
    for (net_id, layer), ends in _dangles.items():
        for i in range(len(ends)):
            xi, yi, wi, gi = ends[i]
            for j in range(i + 1, len(ends)):
                xj, yj, wj, gj = ends[j]
                gap = math.hypot(xi - xj, yi - yj)
                cap = (wi + wj) / 2.0
                if gi and gj:
                    continue  # art meets art: nothing anyone can act on
                if _SOFT_JOINT_MIN_GAP < gap < cap - 1e-6:
                    if gi:  # report where the fix goes: the TRACK end
                        xi, yi, xj, yj = xj, yj, xi, yi
                    net_name = pcb_data.nets.get(net_id, None)
                    net_str = net_name.name if net_name else f"net_{net_id}"
                    violations.append({
                        'type': 'segment-endpoint-gap',
                        'net1': net_str, 'net2': net_str, 'layer': layer,
                        'gap_mm': gap, 'overlap_mm': cap - gap,
                        'loc1': (xi, yi), 'loc2': (xj, yj),
                    })


    # Check via-to-segment violations using spatial index
    if not quiet:
        print("Checking via-to-segment clearances...")
    for via in pcb_data.vias:
        via_net = via.net_id
        via_net_matches = matching_net_ids is None or via_net in matching_net_ids

        # Check against segments on each copper layer (vias go through all layers)
        for layer in routing_layers:
            for obj, seg_net in spatial_idx.get_nearby_for_via(via, layer):
                if not isinstance(obj, Segment):
                    continue
                seg = obj
                if via_net == seg_net:
                    continue  # Same net

                seg_net_matches = matching_net_ids is None or seg_net in matching_net_ids
                if not via_net_matches and not seg_net_matches:
                    continue

                _eff = _pair_cl(via_net, seg_net, layer=seg.layer)
                has_violation, overlap = check_via_segment_overlap(via, seg, _eff, clearance_margin)
                if has_violation and _graphic_pair_is_same_net(seg, None, seg_net, via_net):
                    has_violation = False
                if has_violation:
                    via_net_name = pcb_data.nets.get(via_net, None)
                    seg_net_name = pcb_data.nets.get(seg_net, None)
                    via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                    seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                    violations.append(_mark_required({
                        'type': 'via-segment',
                        'net1': via_net_str,
                        'net2': seg_net_str,
                        'layer': seg.layer,
                        'overlap_mm': overlap,
                        'via_loc': (via.x, via.y),
                        'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    }, _eff))

    # Check via-to-via violations using spatial index
    if not quiet:
        print("Checking via-to-via clearances...")
    via_via_checked = set()
    for via1 in pcb_data.vias:
        net1 = via1.net_id
        net1_matches = matching_net_ids is None or net1 in matching_net_ids

        for via2, net2 in spatial_idx.get_nearby_vias(via1):
            if via1 is via2:
                continue

            net2_matches = matching_net_ids is None or net2 in matching_net_ids
            if not net1_matches and not net2_matches:
                continue

            pair_key = (min(id(via1), id(via2)), max(id(via1), id(via2)))
            if pair_key in via_via_checked:
                continue
            via_via_checked.add(pair_key)

            _eff = _stack_cl(_pair_cl(net1, net2) if net1 != net2 else clearance)
            has_violation, overlap = check_via_via_overlap(via1, via2, _eff, clearance_margin)
            if has_violation:
                net1_name = pcb_data.nets.get(net1, None)
                net2_name = pcb_data.nets.get(net2, None)
                net1_str = net1_name.name if net1_name else f"net_{net1}"
                net2_str = net2_name.name if net2_name else f"net_{net2}"
                violations.append(_mark_required({
                    'type': 'via-via' if net1 != net2 else 'via-via-same-net',
                    'net1': net1_str,
                    'net2': net2_str,
                    'overlap_mm': overlap,
                    'loc1': (via1.x, via1.y),
                    'loc2': (via2.x, via2.y),
                }, _eff))

    # Check pad-to-segment violations using spatial index
    if not quiet:
        print("Checking pad-to-segment clearances...")

    pad_net_ids = list(pads_by_net.keys())
    for seg in pcb_data.segments:
        seg_net = seg.net_id
        seg_net_matches = matching_net_ids is None or seg_net in matching_net_ids

        # Get pads near ANY cell the segment passes through (endpoint-only
        # queries missed a pad grazed mid-run on a long segment -- false clean)
        for pad, pad_net in spatial_idx.get_nearby_pads_for_segment(seg):
            if pad_net == seg_net:
                continue  # Same net
            if _pad_has_no_copper(pad):
                continue  # NPTH hole: covered by the copper-to-hole check
            # Net-tie exemption (footprint net_tie_pad_groups): KiCad permits
            # the tied net's copper to contact the partner pad where the
            # collision point lies ON the tied net's own pad
            # (DRC_ENGINE::IsNetTieExclusion) -- a Kelvin sense track exits
            # dead-centre through its own tab. The waiver mirrors that
            # locality: every point of the segment's violating span must keep
            # its copper within reach of the own tie pad.
            if hasattr(pcb_data, 'net_tie_exempt_pad_ids') and \
                    id(pad) in pcb_data.net_tie_exempt_pad_ids(seg_net) and \
                    _net_tie_span_waived(pcb_data, seg, seg_net, pad, clearance):
                continue

            pad_net_matches = matching_net_ids is None or pad_net in matching_net_ids
            if not seg_net_matches and not pad_net_matches:
                continue

            _eff = _pad_pair_cl(pad, seg_net, layer=seg.layer)
            has_violation, overlap, closest_pt = check_pad_segment_overlap(
                pad, seg, _eff, routing_layers, clearance_margin
            )
            if has_violation and _graphic_pair_is_same_net(seg, None, seg_net, pad_net):
                has_violation = False
            if has_violation:
                pad_net_name = pcb_data.nets.get(pad_net, None)
                seg_net_name = pcb_data.nets.get(seg_net, None)
                pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg_net}"
                violations.append(_mark_required({
                    'type': 'pad-segment',
                    'net1': pad_net_str,
                    'net2': seg_net_str,
                    'layer': seg.layer,
                    'overlap_mm': overlap,
                    'pad_loc': (pad.global_x, pad.global_y),
                    'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    'closest_pt': closest_pt,
                }, _eff))

    # Check pad-to-via violations using spatial index
    if not quiet:
        print("Checking pad-to-via clearances...")

    _pv_seen = set()  # a *.Cu pad is indexed on EVERY layer: test each pair once
    for via in pcb_data.vias:
        via_net = via.net_id
        via_net_matches = matching_net_ids is None or via_net in matching_net_ids

        for layer in routing_layers:
            for pad, pad_net in spatial_idx.get_nearby_pads(via.x, via.y, layer):
                if pad_net == via_net:
                    continue  # Same net
                if _pad_has_no_copper(pad):
                    continue  # NPTH hole: covered by the drill-to-drill check
                _pv_key = (id(pad), id(via))
                if _pv_key in _pv_seen:
                    continue  # already tested on another layer (the check is layer-independent)
                _pv_seen.add(_pv_key)

                pad_net_matches = matching_net_ids is None or pad_net in matching_net_ids
                if not via_net_matches and not pad_net_matches:
                    continue

                _eff = _pad_pair_cl(pad, via_net)
                has_violation, overlap = check_pad_via_overlap(
                    pad, via, _eff, routing_layers, clearance_margin
                )
                if has_violation:
                    pad_net_name = pcb_data.nets.get(pad_net, None)
                    via_net_name = pcb_data.nets.get(via_net, None)
                    pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                    via_net_str = via_net_name.name if via_net_name else f"net_{via_net}"
                    violations.append(_mark_required({
                        'type': 'pad-via',
                        'net1': pad_net_str,
                        'net2': via_net_str,
                        'overlap_mm': overlap,
                        'pad_loc': (pad.global_x, pad.global_y),
                        'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                        'via_loc': (via.x, via.y),
                    }, _eff))

    # Check pad-to-pad violations using spatial index (issue #234). Two pads of
    # DIFFERENT nets that overlap (a short) or sit below clearance on a shared
    # copper layer -- e.g. a placement step nudging a cap onto an IC pad. KiCad
    # flags these (shorting_items / clearance); check_drc previously had only the
    # pad-segment and pad-via passes.
    if not quiet:
        print("Checking pad-to-pad clearances...")
    pad_pad_checked = set()
    for pad_net, pads in pads_by_net.items():
        net1_matches = matching_net_ids is None or pad_net in matching_net_ids
        for pad1 in pads:
            if _pad_has_no_copper(pad1):
                continue  # NPTH hole: no copper to short/graze
            for layer in _expand_cu(pad1.layers, routing_layers):
                if not layer.endswith('.Cu'):
                    continue
                for pad2, pad2_net in spatial_idx.get_nearby_pads_for_pad(pad1, layer):
                    if pad2 is pad1:
                        continue
                    if pad2_net == pad_net:
                        continue  # Same net -- allowed to touch
                    if _pad_has_no_copper(pad2):
                        continue  # NPTH hole: no copper to short/graze
                    # Skip pads of the SAME footprint: a component's own adjacent
                    # pins are fixed library geometry (a fine-pitch part can have
                    # pad gaps below the routing clearance), never something a
                    # placement/routing step introduces or can fix. Flagging them
                    # would flood grading with pre-existing noise. The cases #234
                    # targets are between DIFFERENT footprints (e.g. a cap nudged
                    # onto an IC pad).
                    if pad2.component_ref == pad1.component_ref:
                        continue
                    net2_matches = matching_net_ids is None or pad2_net in matching_net_ids
                    if not net1_matches and not net2_matches:
                        continue
                    pair_key = (min(id(pad1), id(pad2)), max(id(pad1), id(pad2)))
                    if pair_key in pad_pad_checked:
                        continue
                    pad_pad_checked.add(pair_key)
                    # both pads' overrides are folded in by _pad_pair_cl
                    _eff = _pad_pair_cl(pad1, pad2_net, other_pad=pad2)
                    has_violation, overlap, closest_pt = check_pad_pad_overlap(
                        pad1, pad2, _eff, routing_layers, clearance_margin)
                    if has_violation:
                        n1 = pcb_data.nets.get(pad_net, None)
                        n2 = pcb_data.nets.get(pad2_net, None)
                        n1s = n1.name if n1 else f"net_{pad_net}"
                        n2s = n2.name if n2 else f"net_{pad2_net}"
                        violations.append(_mark_required({
                            'type': 'pad-pad',
                            'net1': n1s,
                            'net2': n2s,
                            # A pad with no net cannot electrically short a net;
                            # keep the clearance violation but not the SHORT tag.
                            'no_net': pad_net == 0 or pad2_net == 0,
                            'layer': layer,
                            'overlap_mm': overlap,
                            'pad_ref': f"{pad1.component_ref}.{pad1.pad_number}",
                            'pad_ref2': f"{pad2.component_ref}.{pad2.pad_number}",
                            'pad_loc': (pad1.global_x, pad1.global_y),
                            'pad_loc2': (pad2.global_x, pad2.global_y),
                            'closest_pt': closest_pt,
                        }, _eff))

    # Dummy variables for compatibility with remaining code
    via_net_ids = list(vias_by_net.keys())
    matching_via_nets = matching_net_ids
    matching_seg_net_set = matching_net_ids
    matching_pad_nets = matching_net_ids

    # Check hole-to-hole clearance (via drill to via drill)
    if hole_to_hole_clearance > 0:
        if not quiet:
            print("Checking via drill hole-to-hole clearances...")
        # Via drill hole-to-hole: vectorized over all via pairs. For each via i we
        # compute the center distance to every via j>i at once; the violation test
        # (overlap = drill_i/2 + drill_j/2 + clearance - dist > tolerance) and the
        # reported overlap mirror check_via_drill_overlap exactly. The default
        # clearance_margin there is 0.05, so tolerance = clearance * 0.05.
        all_vias = list(pcb_data.vias)
        nv = len(all_vias)
        if nv >= 2:
            vx = np.array([v.x for v in all_vias], dtype=np.float64)
            vy = np.array([v.y for v in all_vias], dtype=np.float64)
            vdrill = np.array([v.drill for v in all_vias], dtype=np.float64)
            if matching_via_nets is None:
                vmatch = None
            else:
                vmatch = np.array([v.net_id in matching_via_nets for v in all_vias], dtype=bool)
            tolerance = hole_to_hole_clearance * 0.05
            for i in range(nv - 1):
                required = vdrill[i] / 2 + vdrill[i + 1:] / 2 + hole_to_hole_clearance
                actual = np.sqrt((vx[i] - vx[i + 1:]) ** 2 + (vy[i] - vy[i + 1:]) ** 2)
                overlap = required - actual
                viol = overlap > tolerance
                if vmatch is not None:
                    # Skip pairs where neither net matches the filter.
                    viol &= vmatch[i] | vmatch[i + 1:]
                for k in np.nonzero(viol)[0].tolist():
                    via1 = all_vias[i]
                    via2 = all_vias[i + 1 + k]
                    net1_name = pcb_data.nets.get(via1.net_id, None)
                    net2_name = pcb_data.nets.get(via2.net_id, None)
                    net1_str = net1_name.name if net1_name else f"net_{via1.net_id}"
                    net2_str = net2_name.name if net2_name else f"net_{via2.net_id}"
                    violations.append({
                        'type': 'via-drill-hole',
                        'net1': net1_str,
                        'net2': net2_str,
                        'overlap_mm': float(overlap[k]),
                        'loc1': (via1.x, via1.y),
                        'loc2': (via2.x, via2.y),
                    })

        # Check via drill to pad drill (through-hole pads)
        # NOTE: Hole-to-hole clearance applies regardless of net (manufacturing constraint)
        if not quiet:
            print("Checking via drill to pad drill clearances...")
        # Flatten through-hole pads (SMD pads have no drill) keeping their net key,
        # then vectorize each via against all of them. Iteration order (via outer,
        # pad inner in pads_by_net order) and the overlap formula match the scalar
        # check_pad_drill_via_overlap so the violation list is identical.
        th_pads = []
        th_pad_nets = []
        for pad_net, pads in pads_by_net.items():
            for pad in pads:
                if pad.drill <= 0:
                    continue  # SMD pad
                th_pads.append(pad)
                th_pad_nets.append(pad_net)
        if th_pads and all_vias:
            # Each pad hole as its drill CAPSULE (slot-aware, see
            # pad_drill_capsule): distance is via-centre to the capsule axis,
            # radius is the slot's SHORT dimension. Round drills degenerate to
            # the old centre-to-centre circle test.
            from kicad_parser import pad_drill_capsule
            _caps = [pad_drill_capsule(p) for p in th_pads]
            p1x = np.array([c[0][0] for c in _caps], dtype=np.float64)
            p1y = np.array([c[0][1] for c in _caps], dtype=np.float64)
            p2x = np.array([c[1][0] for c in _caps], dtype=np.float64)
            p2y = np.array([c[1][1] for c in _caps], dtype=np.float64)
            prad = np.array([c[2] for c in _caps], dtype=np.float64)
            cdx = p2x - p1x
            cdy = p2y - p1y
            clen2 = cdx * cdx + cdy * cdy
            safe_clen2 = np.where(clen2 > 0, clen2, 1.0)
            if matching_pad_nets is None:
                pmatch = None
            else:
                pmatch = np.array([pn in matching_pad_nets for pn in th_pad_nets], dtype=bool)
            tolerance = hole_to_hole_clearance * 0.05
            for via in all_vias:
                via_matches = matching_via_nets is None or via.net_id in matching_via_nets
                required = prad + via.drill / 2 + hole_to_hole_clearance
                t = np.clip(((via.x - p1x) * cdx + (via.y - p1y) * cdy) / safe_clen2, 0.0, 1.0)
                actual = np.sqrt((via.x - (p1x + t * cdx)) ** 2 + (via.y - (p1y + t * cdy)) ** 2)
                overlap = required - actual
                viol = overlap > tolerance
                # Skip (via, pad) where neither net matches the filter. When the via
                # matches, every pad is checked; otherwise only filter-matching pads.
                if not via_matches and pmatch is not None:
                    viol &= pmatch
                for k in np.nonzero(viol)[0].tolist():
                    pad = th_pads[k]
                    pad_net = th_pad_nets[k]
                    via_net_name = pcb_data.nets.get(via.net_id, None)
                    pad_net_name = pcb_data.nets.get(pad_net, None)
                    via_net_str = via_net_name.name if via_net_name else f"net_{via.net_id}"
                    pad_net_str = pad_net_name.name if pad_net_name else f"net_{pad_net}"
                    same_net = pad_net == via.net_id
                    violations.append({
                        'type': 'pad-drill-via-drill-same-net' if same_net else 'pad-drill-via-drill',
                        'net1': pad_net_str,
                        'net2': via_net_str,
                        'overlap_mm': float(overlap[k]),
                        'pad_loc': (pad.global_x, pad.global_y),
                        'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                        'via_loc': (via.x, via.y),
                    })

    # Check copper-to-hole clearance: a TRACK too close to an NPTH (no-copper) drill
    # hole of a DIFFERENT net (issue #233). The drill removes the copper that crosses
    # it -- a real fab short the via-drill-only hole check above misses (e.g. a track
    # routed straight across a connector mounting hole). PTH pads and vias carry
    # copper, so their track clearance is already covered by the pad-segment /
    # via-segment checks (with the real pad shape, not a round-drill approximation),
    # and KiCad likewise reports only the NPTH cases. Mirrors KiCad's hole_clearance.
    if not quiet:
        print("Checking copper-to-hole (track <-> NPTH drill) clearances...")
    # Each hole is its drill CAPSULE ((x1,y1),(x2,y2),r): a slot drill's real
    # shape. Round drills degenerate to a zero-length capsule (the old circle).
    from kicad_parser import pad_drill_capsule
    # JLC "NPTH to Track" fab floor (never below the graded clearance), raised
    # to the board's own `min_hole_clearance` when it declares a tighter-than-
    # -default requirement. Without that third term this check graded every
    # board at a HARDCODED 0.20 and never opened the project at all, so copper
    # sitting in a board's authored copper-to-hole band read clean -- measured
    # on neo6502: three NPTH holes at 0.2126/0.2263/0.2263 mm against an
    # authored min_hole_clearance of 0.25, clean at 0.20, and clean BEFORE any
    # ratchet as well, because the key was never consulted in the first place.
    npth_clr = max(clearance, defaults.NPTH_TO_TRACK_CLEARANCE, hole_clearance)
    # Each entry: (p1, p2, r, net_id, ref, required_clr, copper_exempt).
    # NPTH (no-copper) pad holes graded at the fab floor -- or the pad's own
    # clearance OVERRIDE when larger (KiCad's hole_clearance honors it; #326
    # residual, ghoul's zero-ring switch NPTHs carry 0.3). copper_exempt=None.
    # PLUS plated pad holes whose pad carries an override: KiCad grades those
    # net-INDEPENDENTLY (same-net included), but exempts copper that actually
    # touches the pad -- copper_exempt=(cx, cy, copper half-extent) carries
    # the landing-disc approximation of that connected-copper exemption.
    holes = []
    for pad_net, pads in pads_by_net.items():
        for pad in pads:
            if pad.drill <= 0:
                continue
            lc = getattr(pad, 'local_clearance', 0.0) or 0.0
            if _pad_has_no_copper(pad):
                hp1, hp2, hr = pad_drill_capsule(pad)
                holes.append((hp1, hp2, hr, pad_net,
                              f"{pad.component_ref}.{pad.pad_number}",
                              max(npth_clr, lc), None))
            elif max(pad.size_x, pad.size_y) < pad.drill:
                # #441: a PLATED pad whose copper ring does NOT span its drill
                # (vfo_ctrl's U4 "MH": 0.001mm copper over a 2.5mm drill) leaves
                # the hole exposed -- a track that crosses it is cut by the drill,
                # net-independently, exactly like an NPTH mounting hole. Grade it
                # the same way (copper-to-drill floor), exempting only the tiny
                # copper speck so the pad's own micro-ring doesn't self-flag.
                # Parity with add_drill_hole_obstacles's ring-uncovered case.
                hp1, hp2, hr = pad_drill_capsule(pad)
                holes.append((hp1, hp2, hr, pad_net,
                              f"{pad.component_ref}.{pad.pad_number}",
                              max(npth_clr, lc),
                              (pad.global_x, pad.global_y,
                               max(pad.size_x, pad.size_y) / 2.0)))
            elif lc > 0:
                hp1, hp2, hr = pad_drill_capsule(pad)
                holes.append((hp1, hp2, hr, pad_net,
                              f"{pad.component_ref}.{pad.pad_number}",
                              lc, (pad.global_x, pad.global_y,
                                   max(pad.size_x, pad.size_y) / 2.0)))
    segs = list(pcb_data.segments)
    _seg_arrays = None  # (sx1, sy1, sx2, sy2, dx, dy, safe_len2, sw); also reused by the slot-edge check
    if holes and segs:
        sx1 = np.array([s.start_x for s in segs], dtype=np.float64)
        sy1 = np.array([s.start_y for s in segs], dtype=np.float64)
        sx2 = np.array([s.end_x for s in segs], dtype=np.float64)
        sy2 = np.array([s.end_y for s in segs], dtype=np.float64)
        sw = np.array([s.width for s in segs], dtype=np.float64)
        snet = np.array([s.net_id for s in segs])
        dx = sx2 - sx1
        dy = sy2 - sy1
        seglen2 = dx * dx + dy * dy
        safe_len2 = np.where(seglen2 > 0, seglen2, 1.0)
        _seg_arrays = (sx1, sy1, sx2, sy2, dx, dy, safe_len2, sw)
        if matching_net_ids is not None:
            seg_match = np.array([n in matching_net_ids for n in snet], dtype=bool)
        def _pts_to_tracks(hx, hy):
            # point -> per-track closest distance, vectorized over all tracks
            t = np.clip(((hx - sx1) * dx + (hy - sy1) * dy) / safe_len2, 0.0, 1.0)
            cxp = sx1 + t * dx
            cyp = sy1 + t * dy
            return np.sqrt((hx - cxp) ** 2 + (hy - cyp) ** 2)

        for (h1x, h1y), (h2x, h2y), hr, hnet, ref, req_clr, copper_exempt in holes:
            # Capsule(hole)-to-segment distance, vectorized over all tracks. A
            # drill is a through-hole, so any track layer crossing it conflicts
            # (no layer filter). An NPTH hole's own-net track legitimately
            # connects to it -> skip; a plated override hole is graded
            # net-independently, exempting only copper that touches the pad.
            dist = _np_capsule_to_tracks(h1x, h1y, h2x, h2y, *_seg_arrays[:7])
            overlap = (hr + sw / 2.0 + req_clr) - dist
            tolerance = _grade_tol(req_clr, clearance_margin)
            viol = overlap > tolerance
            if copper_exempt is None:
                # NPTH: no copper to connect to; keep the own-net track skip.
                viol &= (snet != hnet)
            else:
                # Plated override hole: net-independent for FOREIGN copper, but
                # KiCad exempts a track of the pad's OWN net -- it lands on the
                # pad and connects there, so hole_clearance never flags it
                # (#442: ecp5_mini GND track -> GND pad hole, check_drc 56 vs
                # kicad-cli 0). Same-net exemption mirrors the NPTH branch above.
                viol &= (snet != hnet)
                # Foreign copper that still physically overlaps the pad copper
                # is a pad-segment SHORT reported there, not a hole graze --
                # exempt it here to avoid a double count. Touch test
                # approximates the pad by its bounding disc.
                ecx, ecy, er = copper_exempt
                viol &= _pts_to_tracks(ecx, ecy) > (er + sw / 2.0)
            if matching_net_ids is not None:
                viol &= seg_match | (hnet in matching_net_ids)
            for k in np.nonzero(viol)[0].tolist():
                seg = segs[k]
                hole_net_name = pcb_data.nets.get(hnet, None)
                seg_net_name = pcb_data.nets.get(seg.net_id, None)
                hole_net_str = hole_net_name.name if hole_net_name else f"net_{hnet}"
                seg_net_str = seg_net_name.name if seg_net_name else f"net_{seg.net_id}"
                v = {
                    'type': 'track-hole',
                    'net1': hole_net_str,        # the drill's net (0 = NPTH)
                    'net2': seg_net_str,         # the crossing track's net
                    'hole_ref': ref or 'via',
                    'layer': seg.layer,
                    'overlap_mm': float(overlap[k]),
                    'hole_loc': ((h1x + h2x) / 2.0, (h1y + h2y) / 2.0),
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                }
                # Attribute an above-default requirement (pad clearance
                # override), mirroring _mark_required / KiCad's wording.
                if req_clr > (npth_clr if copper_exempt is None else clearance) + 1e-9:
                    v['required_mm'] = req_clr
                violations.append(v)

    # VIA arm of the same copper-to-hole rule (#505). The pass above walks
    # TRACKS only; a via near a hole was covered solely by the drill-to-drill
    # check (pad-drill-via-drill, at hole_to_hole_clearance), which measures the
    # via's DRILL. KiCad's hole_clearance holds the via's COPPER off the hole
    # wall and honors the pad's clearance override, so an NPTH mounting hole
    # with a local_clearance override went entirely unseen here -- pinci shipped
    # 5 such items (0.9/1.3mm overrides, vias 0.76-1.13mm away) that check_drc
    # reported as clean while kicad-cli flagged every one.
    if holes and pcb_data.vias:
        for (h1x, h1y), (h2x, h2y), hr, hnet, ref, req_clr, copper_exempt in holes:
            # NPTH only (copper_exempt is None), symmetric with the router's
            # via keep-out. A PLATED pad's own copper already blocks vias and is
            # reported by the pad-via checks, so grading its barrel here would
            # double-count the same physical conflict.
            if copper_exempt is not None:
                continue
            # Grade at KICAD's requirement, not the project's NPTH fab floor.
            # The hole tuple carries max(npth_clr, local_clearance): the floor is
            # a conservative ROUTING policy for tracks (NPTH_TO_TRACK_CLEARANCE),
            # not a KiCad DRC rule, so grading vias against it invents items
            # kicad-cli never reports (crkbd: 7 phantoms at 0.016-0.023mm, whose
            # NPTH pads carry no override at all -- exactly the 0.20-vs-0.127
            # gap). Above the floor the value can only have come from the pad
            # override, which KiCad does honor.
            kicad_req = req_clr if req_clr > npth_clr + 1e-9 else clearance
            for via in pcb_data.vias:
                # Own-net copper legitimately lands on the pad (mirrors the
                # track arm's snet != hnet exemption, #442).
                if via.net_id == hnet:
                    continue
                dist = point_to_segment_distance(via.x, via.y, h1x, h1y, h2x, h2y)
                overlap = (hr + via.size / 2.0 + kicad_req) - dist
                if overlap <= _grade_tol(kicad_req, clearance_margin):
                    continue
                hole_net = pcb_data.nets.get(hnet, None)
                via_net = pcb_data.nets.get(via.net_id, None)
                v = {
                    'type': 'via-hole',
                    'net1': hole_net.name if hole_net else f"net_{hnet}",
                    'net2': via_net.name if via_net else f"net_{via.net_id}",
                    'hole_ref': ref or 'via',
                    'overlap_mm': float(overlap),
                    'hole_loc': ((h1x + h2x) / 2.0, (h1y + h2y) / 2.0),
                    'via_loc': (via.x, via.y),
                }
                if kicad_req > clearance + 1e-9:
                    v['required_mm'] = kicad_req
                violations.append(v)

    # Check board edge clearances. Measure to the real Edge.Cuts outline (outer
    # ring + interior cutouts) when the parser found one, so copper routed into a
    # cutout/slot/notch -- which sits inside the bounding box -- is caught (issue
    # #236). Fall back to the rectangular bounding box otherwise.
    board_bounds = pcb_data.board_info.board_bounds
    # Honor the board's own copper_edge_clearance DRC severity (#427): when the
    # project sets it to 'ignore', KiCad runs NO board-edge check (its oracle
    # reports zero), so grading it here only manufactures phantom
    # SEGMENT-BOARD-EDGE items the board deliberately suppressed. Skip to match.
    edge_ignored = respect_edge_severity and edge_clearance_severity(pcb_file) == 'ignore'
    if board_bounds and effective_board_edge_clearance > 0 and edge_ignored and not quiet:
        print("Skipping board edge clearances "
              "(project sets copper_edge_clearance severity to 'ignore')...")
    if board_bounds and effective_board_edge_clearance > 0 and not edge_ignored:
        edge_rings, edge_outer, edge_cutouts = board_edge_geometry(pcb_data.board_info)
        use_poly = bool(edge_rings)
        if not quiet:
            print("Checking board edge clearances "
                  f"({'real Edge.Cuts outline' if use_poly else 'bounding box'})...")

        # Near-edge pad copper: a track whose edge-violating portion lies INSIDE an
        # (edge-exempt) pad adds no NEW edge-violating copper -- the pad copper was
        # already there, and pads are edge-exempt by design (--check-pad-edge off: a
        # placed part cannot be moved off the outline). ottercast_audio R7.2: a track
        # lands on a pad placed 0.39mm from a 0.5mm-rule edge; the pad is exempt, so
        # the track running into it must be too. Build the near-edge pad set once.
        _edge_pads = []
        _ep_edge = {}  # id(pad) -> pad copper's own min distance to the outline
        # Grid-quantization allowance (~grid_step/2 for the default 0.05 routing
        # grid): a track routed right ALONGSIDE an edge-exempt pad snaps to grid
        # nodes, so a sampled centre point can land ~grid_step/2 shy of literally
        # overlapping the pad copper (ottercast R7.2: a +3V3 tap sample sits 0.085mm
        # from a pad whose half-width reach is 0.0635mm -- a 0.021mm quantization
        # sliver). Treat the track as touching the pad within this tolerance. Only
        # loosens the TOUCH test; the pad-closer-to-edge guard below stays strict, so
        # a track poking genuinely CLOSER to the edge than the pad is still a real graze.
        _edge_touch_quant = 0.025
        if use_poly and edge_rings:
            for _pd in (p for fp in pcb_data.footprints.values() for p in fp.pads):
                if getattr(_pd, 'pad_type', '') == 'np_thru_hole' or not (_pd.size_x and _pd.size_y):
                    continue  # NPTH has no copper
                _ext = max(_pd.size_x, _pd.size_y) / 2.0 + effective_board_edge_clearance + 0.05
                if _point_to_rings_distance(_pd.global_x, _pd.global_y, edge_rings) <= _ext:
                    _edge_pads.append(_pd)
                    _ep_edge[id(_pd)] = min((_point_to_rings_distance(px, py, edge_rings)
                                             for px, py in _pad_perimeter_points(_pd)),
                                            default=_point_to_rings_distance(
                                                _pd.global_x, _pd.global_y, edge_rings))

        def _seg_edge_all_in_pads(seg):
            """True iff every board-edge-violating point of ``seg`` is COVERED by a
            near-edge (edge-exempt) pad: the track copper TOUCHES the pad copper (the
            centreline is within a half-width of it) AND that point is no closer to the
            outline than the pad's own copper -- so the pad already establishes the
            near-edge copper there and the track adds no new edge exposure (ottercast
            R7.2: a +3V3 tap runs into / alongside a pad placed 0.39mm from a 0.5mm
            edge). A point CLOSER to the edge than every touching pad is a REAL graze."""
            if not _edge_pads:
                return False
            half = seg.width / 2.0
            req = effective_board_edge_clearance + half
            L = math.hypot(seg.end_x - seg.start_x, seg.end_y - seg.start_y)
            steps = max(2, int(L / 0.05) + 1)
            saw_viol = False
            for i in range(steps + 1):
                t = i / steps
                x = seg.start_x + t * (seg.end_x - seg.start_x)
                y = seg.start_y + t * (seg.end_y - seg.start_y)
                pt_edge = _point_to_rings_distance(x, y, edge_rings)
                if pt_edge < req:
                    saw_viol = True
                    covered = any(point_to_pad_distance(x, y, _pd) <= half + _edge_touch_quant
                                  and _ep_edge[id(_pd)] <= pt_edge - half + 1e-6
                                  for _pd in _edge_pads)
                    if not covered:
                        return False  # closer to the edge than any touching pad -> real
            return saw_viol

        # Check segments
        for seg in pcb_data.segments:
            seg_matches = matching_seg_net_set is None or seg.net_id in matching_seg_net_set
            if matching_seg_net_set is not None and not seg_matches:
                continue
            # The pad-covered EXEMPTION is a geometric fact (a track running into
            # an edge-exempt pad adds no new edge copper), independent of the
            # grid-quantization margin: a covered segment must PUBLISH as accepted
            # -- so kicad_drc_compare subtracts the matching kicad
            # copper_edge_clearance -- whether its overlap is 8um or 100um. Decide
            # coverage on the STRICT (margin-0) geometry first; only NON-exempt
            # grazes are then margin-gated into a real failure (ottercast R7.2: two
            # +3V3 taps graze 0.008/0.020mm, below the 0.025mm margin, but are
            # pad-covered -- they were silently margin-dropped and left their kicad
            # findings orphaned as false-negative alarms).
            if use_poly:
                s_viol, s_overlap, s_edge = check_segment_board_edge_poly(
                    seg, edge_rings, edge_outer, edge_cutouts,
                    effective_board_edge_clearance, 0.0)
            else:
                s_viol, s_overlap, s_edge = check_segment_board_edge(
                    seg, board_bounds, effective_board_edge_clearance, 0.0)
            if not s_viol:
                continue  # no sub-rule edge graze at all
            net_name = pcb_data.nets.get(seg.net_id, None)
            net_str = net_name.name if net_name else f"net_{seg.net_id}"
            if s_edge != "off-board" and use_poly and _seg_edge_all_in_pads(seg):
                _accepted_edge.append({
                    'type': 'segment-board-edge', 'net1': net_str, 'edge': s_edge,
                    'layer': seg.layer, 'overlap_mm': s_overlap,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    'accepted': 'edge-exempt-pad',
                })
                continue
            # not exempt -> real only if it clears the grid-quantization margin.
            # Derived from the STRICT result already computed above (identical
            # distances, only the tolerance differs) -- re-running the check at
            # the margin doubled the full outline-ring scan per segment.
            if s_edge == "off-board" or s_overlap > _grade_tol(effective_board_edge_clearance, clearance_margin):
                violations.append({
                    'type': 'segment-board-edge', 'net1': net_str, 'edge': s_edge,
                    'layer': seg.layer, 'overlap_mm': s_overlap,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                })
            else:
                # Strictly positive but sub-margin: grid-quantization noise
                # (ghoul 0.8um, dilemma 2-4um at the 0.20 floor). PUBLISH as
                # accepted -- kicad-cli grades the exact metric and reports
                # these, so kicad_drc_compare must subtract its finding rather
                # than alarm it as a kicad-only divergence (#448, same
                # publish-don't-drop contract as the pad-covered class above).
                _accepted_edge.append({
                    'type': 'segment-board-edge', 'net1': net_str, 'edge': s_edge,
                    'layer': seg.layer, 'overlap_mm': s_overlap,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    'accepted': 'quantization-margin',
                })

        # Check vias
        for via in pcb_data.vias:
            via_matches = matching_via_nets is None or via.net_id in matching_via_nets
            if matching_via_nets is not None and not via_matches:
                continue
            # One STRICT check; the margined verdict is derived from it (same
            # distances, different tolerance -- avoids a second ring scan).
            if use_poly:
                s_viol, s_overlap, s_edge = check_via_board_edge_poly(
                    via, edge_rings, edge_outer, edge_cutouts,
                    effective_board_edge_clearance, 0.0)
            else:
                s_viol, s_overlap, s_edge = check_via_board_edge(
                    via, board_bounds, effective_board_edge_clearance, 0.0)
            has_violation = s_viol and (
                s_edge == "off-board"
                or s_overlap > _grade_tol(effective_board_edge_clearance, clearance_margin))
            if s_viol:
                net_name = pcb_data.nets.get(via.net_id, None)
                net_str = net_name.name if net_name else f"net_{via.net_id}"
            # Via-in-edge-pad exemption (#448, via analog of the ottercast
            # R7.2 segment rule): a via whose barrel lands ON an (edge-exempt)
            # pad's copper, and whose ring stays no closer to the outline than
            # the pad's own copper already is, adds no NEW edge exposure --
            # the last-resort via-in-pad rescue (#189) legitimately taps USB
            # connector pads that overhang the outline (crkbd rJ1 VBUSR).
            if s_viol and s_edge != "off-board" and use_poly and _edge_pads:
                _via_edge_d = _point_to_rings_distance(via.x, via.y, edge_rings) - via.size / 2.0
                if any(point_to_pad_distance(via.x, via.y, _pd) <= via.size / 2.0 + _edge_touch_quant
                       and _ep_edge[id(_pd)] <= _via_edge_d + 1e-6
                       for _pd in _edge_pads):
                    _accepted_edge.append({
                        'type': 'via-board-edge', 'net1': net_str, 'edge': s_edge,
                        'overlap_mm': s_overlap, 'via_loc': (via.x, via.y),
                        'accepted': 'edge-exempt-pad',
                    })
                    continue
            if has_violation:
                violations.append({
                    'type': 'via-board-edge',
                    'net1': net_str,
                    'edge': s_edge,
                    'overlap_mm': s_overlap,
                    'via_loc': (via.x, via.y),
                })
            elif s_viol:
                # sub-margin graze: publish accepted (quantization; see the
                # matching segment branch above)
                _accepted_edge.append({
                    'type': 'via-board-edge', 'net1': net_str, 'edge': s_edge,
                    'overlap_mm': s_overlap, 'via_loc': (via.x, via.y),
                    'accepted': 'quantization-margin',
                })

        # ZONE/pour copper is INTENTIONALLY not edge-graded here (#513 item 18,
        # implemented in c7fb340 and then deliberately reverted). KiCad's filler
        # clips fill at the board outline by the ZONE's clearance only, so when
        # the copper_edge_clearance rule is larger the fill genuinely violates
        # it and kicad-cli flags it (healthypi_sensor's 1.8V pour, rule 0.2 vs
        # zone clearance 0.09) -- but we leave that class to the KiCad oracle
        # (kicad_drc_compare's refilled staged copy) on purpose:
        #  - our own outputs carry NO stored (filled_polygon ...) geometry, so
        #    a stored-fill check never fires on them, and grading zone OUTLINES
        #    instead manufactures phantoms (KiCad refills clip at the outline;
        #    measured: 3 phantom findings on healthypi step9, kicad-cli zero);
        #  - on human/reference boards the fab-floor-pinned grade flagged fills
        #    kicad-cli accepts (14/59 corpus originals, e.g. rc2014's 0.1mm
        #    fill vs its own 0.075 rule), inflating original_routed_violations
        #    baselines with a class kicad never confirms.
        # If this is ever revisited, grade STORED fills only, at the board's
        # OWN copper_edge_clearance (un-pinned), to fire exactly where kicad
        # fires.

        # NPTH SLOT (milled oval) holes are board edge to KiCad (#448): its edge
        # provider grades copper proximity to a slot's hole wall as
        # copper_edge_clearance, while ROUND NPTH drills stay in the
        # hole_clearance / copper-to-hole domain (verified with kicad-cli 10
        # probes on sofle_pico: a track 0.22mm from the SW25 2.8x1.5 slot flags
        # copper_edge_clearance; the same track 0.10mm from a round 3.0mm NPTH
        # flags nothing). Grade tracks and vias against each slot capsule at the
        # same effective edge clearance, with EXACT capsule distance -- these
        # breaches are often a few um (sofle SW25A: 13.5um under the 0.3 rule),
        # so ring sampling error would swallow them.
        _slot_caps = []
        from kicad_parser import pad_drill_capsule as _pdc
        for _fp in pcb_data.footprints.values():
            for _pd in _fp.pads:
                if getattr(_pd, 'pad_type', '') != 'np_thru_hole' or _pd.drill <= 0:
                    continue
                (_s1, _s2, _sr) = _pdc(_pd)
                if math.hypot(_s2[0] - _s1[0], _s2[1] - _s1[1]) <= 1e-9:
                    continue  # round drill: not part of the milled edge
                _slot_caps.append((_s1, _s2, _sr,
                                   f"{_pd.component_ref}.{_pd.pad_number}"))
        if _slot_caps and pcb_data.segments:
            # Reuse the copper-to-hole check's per-segment arrays when it ran
            # (slots are NPTH pads, so they are always in its holes list);
            # build them only if that block was skipped.
            if _seg_arrays is None:
                _bx1 = np.array([s.start_x for s in pcb_data.segments], dtype=np.float64)
                _by1 = np.array([s.start_y for s in pcb_data.segments], dtype=np.float64)
                _bx2 = np.array([s.end_x for s in pcb_data.segments], dtype=np.float64)
                _by2 = np.array([s.end_y for s in pcb_data.segments], dtype=np.float64)
                _bdx, _bdy = _bx2 - _bx1, _by2 - _by1
                _blen2 = _bdx * _bdx + _bdy * _bdy
                _seg_arrays = (_bx1, _by1, _bx2, _by2, _bdx, _bdy,
                               np.where(_blen2 > 0, _blen2, 1.0),
                               np.array([s.width for s in pcb_data.segments], dtype=np.float64))
            _esw = _seg_arrays[7]
            _edge_tol = _grade_tol(effective_board_edge_clearance, clearance_margin)

            for (_h1x, _h1y), (_h2x, _h2y), _hr, _slot_ref in _slot_caps:
                _d = _np_capsule_to_tracks(_h1x, _h1y, _h2x, _h2y, *_seg_arrays[:7])
                _hdx, _hdy = _h2x - _h1x, _h2y - _h1y
                _hlen2 = _hdx * _hdx + _hdy * _hdy
                _ovl = (effective_board_edge_clearance + _esw / 2.0) - (_d - _hr)
                for _k in np.nonzero(_ovl > 1e-9)[0].tolist():
                    seg = pcb_data.segments[_k]
                    if matching_seg_net_set is not None and seg.net_id not in matching_seg_net_set:
                        continue
                    net_name = pcb_data.nets.get(seg.net_id, None)
                    net_str = net_name.name if net_name else f"net_{seg.net_id}"
                    _v = {
                        'type': 'segment-board-edge', 'net1': net_str,
                        'edge': 'npth-slot', 'layer': seg.layer,
                        'overlap_mm': float(_ovl[_k]), 'slot_ref': _slot_ref,
                        'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                    }
                    if _ovl[_k] > _edge_tol:
                        violations.append(_v)
                    else:  # sub-margin: publish accepted (quantization)
                        _v['accepted'] = 'quantization-margin'
                        _accepted_edge.append(_v)
                for via in pcb_data.vias:
                    if matching_via_nets is not None and via.net_id not in matching_via_nets:
                        continue
                    _t = 0.0 if _hlen2 <= 0 else max(0.0, min(1.0, ((via.x - _h1x) * _hdx + (via.y - _h1y) * _hdy) / _hlen2))
                    _vd = math.hypot(via.x - (_h1x + _t * _hdx), via.y - (_h1y + _t * _hdy)) - _hr
                    _vovl = (effective_board_edge_clearance + via.size / 2.0) - _vd
                    if _vovl > 1e-9:
                        net_name = pcb_data.nets.get(via.net_id, None)
                        net_str = net_name.name if net_name else f"net_{via.net_id}"
                        _v = {
                            'type': 'via-board-edge', 'net1': net_str,
                            'edge': 'npth-slot', 'overlap_mm': float(_vovl),
                            'slot_ref': _slot_ref, 'via_loc': (via.x, via.y),
                        }
                        if _vovl > _edge_tol:
                            violations.append(_v)
                        else:
                            _v['accepted'] = 'quantization-margin'
                            _accepted_edge.append(_v)

        # Check pads (issue #236). Off by default: pad-to-edge violations are
        # almost always pre-existing edge-connector pads on the bare board (the
        # router never places pads), so flagging them by default just adds noise
        # to routed-board grading. Enable with --check-pad-edge to catch a
        # placement step that pushed a component off the board / into a cutout.
        if not check_pad_edge and not quiet:
            # SAY that it was not checked. The severity-ignore case one branch
            # up prints a "Skipping..." line and this one printed nothing at
            # all, so "pads are clear of the edge" and "pads were never looked
            # at" were the same output. The default is off because on a ROUTED
            # board the hits are almost always pre-existing edge connectors --
            # but that premise inverts on a placement-repair run, where a part
            # really can be pushed off the outline. One board's copper-free
            # baseline was 93 by default and 95 with the flag, and the two
            # extra violations were on precisely the two parts that run was
            # about to freeze and waive.
            print("Skipping pad-to-board-edge checks (--check-pad-edge is off; "
                  "pads are edge-exempt by default because on a routed board "
                  "the hits are usually pre-existing edge connectors). Pass it "
                  "on a board whose PLACEMENT may have moved a part off the "
                  "outline.")
        if check_pad_edge:
            if not quiet:
                # The OFF branch above announces itself and the ON branch used
                # to print nothing, so a log showed a line when the check was
                # skipped and silence when it ran. That is backwards for
                # anyone reading the log later to find out whether the
                # top-priority placement defect was looked for at all.
                print("Checking pad-to-board-edge clearances "
                      "(--check-pad-edge is on)...")
            for pad_net, pads in pads_by_net.items():
                if matching_pad_nets is not None and pad_net not in matching_pad_nets:
                    continue
                for pad in pads:
                    if _pad_has_no_copper(pad):
                        continue  # No copper on this pad (pure NPTH)
                    has_violation, overlap, edge = check_pad_board_edge(
                        pad, edge_rings, edge_outer, edge_cutouts,
                        effective_board_edge_clearance, board_bounds, clearance_margin)
                    if has_violation:
                        net_name = pcb_data.nets.get(pad_net, None)
                        net_str = net_name.name if net_name else f"net_{pad_net}"
                        violations.append({
                            'type': 'pad-board-edge',
                            'net1': net_str,
                            'edge': edge,
                            'overlap_mm': overlap,
                            'pad_ref': f"{pad.component_ref}.{pad.pad_number}",
                            'pad_loc': (pad.global_x, pad.global_y),
                        })

    # Check track widths and via/hole sizes against the fab floor (issue #176).
    # Unlike the clearance checks these are per-object (one net), so the net filter
    # applies to that single net.
    if check_sizes:
        if not quiet:
            print("Checking track widths and via/hole sizes...")
        for seg in pcb_data.segments:
            if matching_net_ids is not None and seg.net_id not in matching_net_ids:
                continue
            _tf = _track_floor(seg)
            too_thin, shortfall = check_track_width(seg, _tf, size_margin)
            if too_thin:
                net_name = pcb_data.nets.get(seg.net_id, None)
                net_str = net_name.name if net_name else f"net_{seg.net_id}"
                violations.append({
                    'type': 'track-width',
                    'net1': net_str,
                    'layer': seg.layer,
                    'width': seg.width,
                    'min_width': _tf,
                    'shortfall_mm': shortfall,
                    'seg_loc': (seg.start_x, seg.start_y, seg.end_x, seg.end_y),
                })
        for via in pcb_data.vias:
            if matching_net_ids is not None and via.net_id not in matching_net_ids:
                continue
            _vd, _vh = _via_floors(via)
            dia_bad, drill_bad, dia_short, drill_short = check_via_size(
                via, _vd, _vh, size_margin)
            if dia_bad or drill_bad:
                net_name = pcb_data.nets.get(via.net_id, None)
                net_str = net_name.name if net_name else f"net_{via.net_id}"
                if dia_bad:
                    violations.append({
                        'type': 'via-size',
                        'net1': net_str,
                        'size': via.size,
                        'min_size': _vd,
                        'shortfall_mm': dia_short,
                        'via_loc': (via.x, via.y),
                    })
                if drill_bad:
                    violations.append({
                        'type': 'via-drill-size',
                        'net1': net_str,
                        'drill': via.drill,
                        'min_drill': _vh,
                        'shortfall_mm': drill_short,
                        'via_loc': (via.x, via.y),
                    })

    # Same-net COPPER overlaps are not DRC failures: same-net copper is allowed
    # to overlap (KiCad's own DRC permits it -- it only enforces clearance between
    # DIFFERENT nets). This covers both same-net segment crossings AND same-net
    # via-to-via copper clearance (two vias on the same net may touch -- e.g. a
    # plane stitching via next to a tap via). They are at most cosmetic clutter,
    # so report them as warnings and keep them out of the violation count / exit
    # status. (Same-net DRILL hole-to-hole checks -- 'via-drill-hole',
    # 'pad-drill-via-drill-same-net' -- stay violations: drill spacing is a real
    # fab constraint independent of net.)
    # Same-net SOFT JOINTS ('segment-endpoint-gap') are a COUNTED violation
    # (#320 step 3, promoted after #318/#322 drove the corpus to zero): a
    # dangling free end held only by cap overlap is a fragile near-open --
    # electrically connected today, an open after etch tolerance/rework. The
    # repair pipeline (close_soft_joints + the neck/strict gates) now prevents
    # or bridges every corpus instance, so a surviving one is a real defect
    # the run must fail on.
    #
    # EXCEPT the sub-coincidence band: a gap at or below COINCIDENCE_TOL
    # (0.02mm, connectivity.py -- THE strict tolerance) is quantization-level
    # contact that every gate and cleanup pass deliberately treats as
    # CONNECTED (so nothing will ever "fix" it), and the caps overlap by far
    # more than any etch tolerance. Counting those would fail boards on gaps
    # the pipeline defines as joined; report them as a warning instead
    # (kuchen /USBH_DN: 0.010mm gap, 0.102mm cap overlap).
    from connectivity import COINCIDENCE_TOL as _COINC_TOL
    _samenet_copper = ('segment-crossing-same-net', 'via-via-same-net')
    seg_warns = [v for v in violations if v['type'] == 'segment-crossing-same-net']
    viavia_warns = [v for v in violations if v['type'] == 'via-via-same-net']
    subcoinc_warns = [v for v in violations if v['type'] == 'segment-endpoint-gap'
                      and v.get('gap_mm', 1.0) <= _COINC_TOL + 1e-9]
    warnings = ([v for v in violations if v['type'] in _samenet_copper]
                + subcoinc_warns)
    _warn_ids = {id(v) for v in warnings}
    violations = [v for v in violations if id(v) not in _warn_ids]

    def _warn_note():
        if warnings:
            print(f"\nWARNINGS ({len(warnings)}, not DRC failures):")
            if subcoinc_warns:
                print(f"  sub-coincidence endpoint gap: {len(subcoinc_warns)} "
                      f"(<= {_COINC_TOL}mm -- quantization-level; treated as "
                      f"connected by routing and cleanup)")
            if seg_warns:
                print(f"  same-net self-crossing: {len(seg_warns)} (same-net copper overlap; "
                      f"permitted by KiCad DRC)")
            if viavia_warns:
                print(f"  same-net via-via: {len(viavia_warns)} (same-net copper overlap; "
                      f"permitted by KiCad DRC -- drill hole-to-hole checked separately)")

    # Report violations. #383: print_summary=False lets an IN-PROCESS caller
    # (kicad_drc_compare.run_check_drc, run under a ThreadPoolExecutor) get the
    # violations back WITHOUT this quiet-summary print -- the old caller wrapped
    # the call in a process-global redirect_stdout, which under concurrency
    # swallowed other worker threads' result lines into its buffer.
    if quiet:
        if violations:
            if print_summary:
                print(f"FAILED ({len(violations)} violations)")
        else:
            if print_summary:
                print("OK" + (f" ({len(warnings)} same-net copper warning(s))" if warnings else ""))
            return violations + _accepted_edge

    # Print detailed results (always for non-quiet, or when violations in quiet mode)
    if not quiet or violations:
        print("\n" + "=" * 60 if not quiet else "=" * 60)
        if violations:
            print(f"FOUND {len(violations)} DRC VIOLATIONS:\n")

            # Group by type
            by_type = {}
            for v in violations:
                t = v['type']
                if t not in by_type:
                    by_type[t] = []
                by_type[t].append(v)

            # Per-type print cap. max_print <= 0 means print every violation
            # (issue #93: a fixed cap silently dropped most of a long list).
            limit = len(violations) if max_print is not None and max_print <= 0 else max_print
            for vtype, vlist in by_type.items():
                # CONTACT count per type. `[SHORT]` is printed at exactly one
                # place in this file, inside the pad-pad branch, so it cannot
                # name a track touching a pad -- two boards differing by a
                # power-rail short reported identical totals AND identical
                # pad-pad short counts, because every differing contact was
                # pad-segment. Only a per-type contact figure separates them,
                # and the JSON already carries it as `contacts_by_type`.
                _nc = sum(1 for v in vlist
                          if isinstance(v.get('overlap_mm'), (int, float))
                          and v['overlap_mm'] >= (
                              v['required_mm']
                              if isinstance(v.get('required_mm'), (int, float))
                              else clearance))
                _ct = f" -- {_nc} in CONTACT" if _nc else ""
                print(f"\n{vtype.upper()} violations ({len(vlist)}){_ct}:")
                print("-" * 40)
                for v in vlist[:limit]:  # Show first `limit` of each type
                    if vtype in ('segment-segment', 'segment-segment-track-rule'):
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        if v.get('track_rule'):
                            print(f"    Track rule: '{v['track_rule']}' (floor-governed pair)")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'via-segment':
                        print(f"  Via:{v['net1']} <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-via':
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                        print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")
                    elif vtype in ('segment-crossing', 'segment-crossing-same-net'):
                        print(f"  {v['net1']} <-> {v['net2']}")
                        print(f"    Layer: {v['layer']}, Cross at: ({v['cross_point'][0]:.3f},{v['cross_point'][1]:.3f})")
                        print(f"    Seg1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})-({v['loc1'][2]:.2f},{v['loc1'][3]:.2f})")
                        print(f"    Seg2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})-({v['loc2'][2]:.2f},{v['loc2'][3]:.2f})")
                    elif vtype == 'segment-endpoint-gap':
                        print(f"  {v['net1']} (same-net soft joint)")
                        print(f"    Layer: {v['layer']}, endpoint gap: {v['gap_mm']:.3f}mm "
                              f"(cap overlap only {v['overlap_mm']:.3f}mm)")
                        print(f"    Ends: ({v['loc1'][0]:.3f},{v['loc1'][1]:.3f}) <-> "
                              f"({v['loc2'][0]:.3f},{v['loc2'][1]:.3f})")
                    elif vtype == 'pad-segment':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Seg:{v['net2']}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'pad-via':
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Via:{v['net2']}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'pad-pad':
                        short = (" [SHORT]" if v['overlap_mm'] >= clearance
                                 and not v.get('no_net') else "")
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Pad:{v['net2']} ({v['pad_ref2']}){short}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad1: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Pad2: ({v['pad_loc2'][0]:.2f},{v['pad_loc2'][1]:.2f})")
                    elif vtype == 'via-drill-hole':
                        print(f"  Via:{v['net1']} <-> Via:{v['net2']} (drill hole clearance)")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via1: ({v['loc1'][0]:.2f},{v['loc1'][1]:.2f})")
                        print(f"    Via2: ({v['loc2'][0]:.2f},{v['loc2'][1]:.2f})")
                    elif vtype in ('pad-drill-via-drill', 'pad-drill-via-drill-same-net'):
                        same_net_msg = " [SAME NET]" if vtype.endswith('same-net') else ""
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) <-> Via:{v['net2']} (drill hole clearance){same_net_msg}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'track-hole':
                        print(f"  Hole:{v['net1']} ({v['hole_ref']}) <-> Track:{v['net2']} (copper-to-hole)")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Hole: ({v['hole_loc'][0]:.2f},{v['hole_loc'][1]:.2f})")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-hole':
                        print(f"  Hole:{v['net1']} ({v['hole_ref']}) <-> Via:{v['net2']} (copper-to-hole)")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Hole: ({v['hole_loc'][0]:.2f},{v['hole_loc'][1]:.2f})")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'segment-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  {v['net1']} {where}")
                        print(f"    Layer: {v['layer']}, Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  Via:{v['net1']} {where}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'pad-board-edge':
                        where = _edge_phrase(v['edge'])
                        print(f"  Pad:{v['net1']} ({v['pad_ref']}) {where}")
                        print(f"    Overlap: {v['overlap_mm']:.3f}mm")
                        print(f"    Pad: ({v['pad_loc'][0]:.2f},{v['pad_loc'][1]:.2f})")
                    elif vtype == 'track-width':
                        print(f"  {v['net1']} track too thin for fab")
                        print(f"    Layer: {v['layer']}, Width: {v['width']:.4f}mm "
                              f"< min {v['min_width']:.4f}mm (short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Seg: ({v['seg_loc'][0]:.2f},{v['seg_loc'][1]:.2f})-({v['seg_loc'][2]:.2f},{v['seg_loc'][3]:.2f})")
                    elif vtype == 'via-size':
                        print(f"  Via:{v['net1']} diameter too small for fab")
                        print(f"    Size: {v['size']:.4f}mm < min {v['min_size']:.4f}mm "
                              f"(short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    elif vtype == 'via-drill-size':
                        print(f"  Via:{v['net1']} drill hole too small for fab")
                        print(f"    Drill: {v['drill']:.4f}mm < min {v['min_drill']:.4f}mm "
                              f"(short {v['shortfall_mm']:.4f}mm)")
                        print(f"    Via: ({v['via_loc'][0]:.2f},{v['via_loc'][1]:.2f})")
                    # #326: attribute above-global requirements (pad/footprint
                    # local clearance or netclass), mirroring KiCad's wording.
                    if v.get('required_mm'):
                        print(f"    Required clearance: {v['required_mm']:.4f}mm "
                              f"(local/netclass override; global {clearance:.4f}mm)")

                if len(vlist) > limit:
                    print(f"  ... and {len(vlist) - limit} more "
                          f"(use --max-print 0 to show all)")

            # Asserted listing total (run-3 B2): a truncated listing must say
            # so machine-checkably. Consumers assert listed == total (or pass
            # --max-print 0) before quoting specific items -- the run-3 orphan
            # incident read 1 of 3 off a tail and shipped the wrong count.
            listed = sum(min(len(vl), limit) for vl in by_type.values())
            trunc = "" if listed == len(violations) else \
                " TRUNCATED (use --max-print 0 to list all)"
            print(f"\nLISTING: {listed} of {len(violations)} violation(s) shown{trunc}")
        else:
            print("NO DRC VIOLATIONS FOUND!")

        _warn_note()
        print("=" * 60)

    # Write debug lines if requested
    if debug_output and violations:
        write_debug_lines(pcb_file, violations, clearance)

    return violations + _accepted_edge


if __name__ == "__main__":
    import cli_banner; cli_banner.install()  # CMD/EXIT self-echo (run-3 B1)
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    parser = argparse.ArgumentParser(description='Check PCB for DRC violations (clearance errors)')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--clearance', '-c', type=float, default=None,
                        help='Minimum clearance in mm to grade against. If omitted, '
                             'auto-detected from the sibling .kicad_pro Default net '
                             'class (the value the board was routed/graded to); falls '
                             'back to 0.2 if no project clearance is found.')
    parser.add_argument('--hole-to-hole-clearance', type=float, default=defaults.HOLE_TO_HOLE_CLEARANCE,
                        help=f'Minimum drill hole edge-to-edge clearance in mm '
                             f'(default: {defaults.HOLE_TO_HOLE_CLEARANCE}, the fab floor — same as routing)')
    parser.add_argument('--board-edge-clearance', type=float, default=0.0,
                        help='Minimum clearance from board edge in mm (0 = use --clearance value)')
    parser.add_argument('--hole-clearance', type=float, default=0.0,
                        help=f'Minimum COPPER-to-drill-hole clearance in mm '
                             f'(0 = auto: the project\'s min_hole_clearance, else '
                             f'the {defaults.NPTH_TO_TRACK_CLEARANCE} fab floor). '
                             f'Raises the floor, never lowers it.')
    parser.add_argument('--clearance-margin', type=float, default=0.05,
                        help='Fraction of clearance to use as tolerance (default: 0.05 = 5%%). Violations smaller than clearance*margin are ignored.')
    parser.add_argument('--nets', '-n', nargs='+', default=None,
                        help='Optional net name patterns to focus on (fnmatch wildcards supported, e.g., "*lvds*")')
    parser.add_argument('--debug-lines', '-d', action='store_true',
                        help='Write debug lines to User.7 layer showing violation locations')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='Only print a summary line unless there are violations')
    parser.add_argument('--max-print', type=int, default=20,
                        help='Max violations to list per type before "... and N more" '
                             '(0 = print all). Default 20.')
    parser.add_argument('--no-size-checks', action='store_true',
                        help='Skip the track-width and via/hole-size fab-floor checks')
    parser.add_argument('--min-track-width', type=float, default=None,
                        help='Minimum manufacturable track width in mm '
                             '(default: JLC fab floor for the board layer count)')
    parser.add_argument('--min-via-diameter', type=float, default=None,
                        help='Minimum via outer diameter in mm (default: fab floor)')
    parser.add_argument('--min-via-drill', type=float, default=None,
                        help='Minimum via drill diameter in mm (default: fab floor)')
    parser.add_argument('--size-margin', type=float, default=0.0,
                        help='Absolute tolerance in mm for the size checks (default: 0)')
    parser.add_argument('--check-pad-edge', action='store_true',
                        help='Also check pad-to-board-edge clearance (issue #236). '
                             'Off by default: pad-edge violations are almost always '
                             'pre-existing edge-connector pads, not router-introduced.')
    parser.add_argument('--json', metavar='FILE', default=None,
                        help='also write the result as JSON: the graded floors '
                             'with their source, the non-accepted violation '
                             'count, the per-type breakdown and every item. '
                             'The file is COMPLETE regardless of --max-print, '
                             'so a consumer never quotes a count off a '
                             'truncated listing.')
    parser.add_argument('--render', metavar='DIR', default=None,
                        help='write question-scoped crop panels of the violation '
                             'clusters to DIR (one PNG per spatial cluster, red '
                             'rings at each violation, count/types/rect in the '
                             'caption). The picture shows WHERE; the numbers '
                             'above say how much.')

    from fab_tiers import add_fab_tier_args, fab_tier_from_args, set_default_fab_tier
    add_fab_tier_args(parser)
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))

    # Grade at the clearance the board was actually routed to. When -c is not
    # given, read the sibling .kicad_pro Default net-class clearance -- which the
    # routers lower to the smallest clearance used in ANY step (incl. fine-pitch
    # tap escalation). Grading stricter than that invents phantom violations on
    # legitimately tight copper; grading looser hides real ones. (issue follow-up
    # to the repair_planes fine-tap grading confusion.)
    _explicit_clearance = args.clearance is not None   # args.clearance is overwritten below
    if args.clearance is None:
        args.clearance = 0.2
        found = False
        try:
            import os, json
            from fix_kicad_drc_settings import find_project, project_copper_clearance
            pro = find_project(args.pcb)
            if os.path.isfile(pro):
                with open(pro) as f:
                    pc = project_copper_clearance(json.load(f))
                if pc:
                    args.clearance = pc
                    found = True
                    if not args.quiet:
                        print(f"Grading at clearance {pc:.4g} mm "
                              f"(from {os.path.basename(pro)} Default net class)")
        except Exception as e:
            print(f"  (could not read project clearance, using 0.2 mm: {e})")
        if not found and not args.quiet:
            print("Grading at clearance 0.2 mm (no project clearance found; "
                  "pass -c to override)")
            print(f"  WARNING: {os.path.basename(args.pcb)} has NO sibling .kicad_pro. "
                  f"Opening it in KiCad will auto-create a project with DEFAULT "
                  f"constraints and report hundreds of phantom annular/track/hole "
                  f"violations on a fine-pitch board (#295). Generate one with:\n"
                  f"    python3 py_router/fix_kicad_drc_settings.py {args.pcb}")
    elif not args.quiet:
        # Run-3 B3 (and run-2 T3): the graded clearance used to be echoed ONLY
        # on the auto-derived branch, so an explicit -c left no trace in the
        # log and board_score's graded_at parsed to null exactly when the
        # caller was most explicit. Say it always, with its source.
        print(f"Grading at clearance {args.clearance:.4g} mm (--clearance)")

    # ...and say when the board declares NO floor of its own, on BOTH branches
    # (run-12 Tier 1.3). The missing-project warning above fires only when -c
    # was omitted -- yet CLAUDE.md tells a caller to pass the routed clearance
    # explicitly, which is exactly the case where the board's own silence went
    # unrecorded. Measured on tigard (no .kicad_pro): every floor accessor
    # returns None, a whole baseline was graded against fallbacks, and
    # `grep -icE "no sibling|no .kicad_pro|no project"` over the log returned 0.
    # Report-only: no floor and no exit code changes here. The flag is computed
    # even under -q, because the JSON below carries it too.
    _board_declares_no_floor = False
    try:
        from list_nets import board_floor_declaration
        _decl = board_floor_declaration(args.pcb)
        _board_declares_no_floor = bool(_decl['declares_nothing'])
        if _board_declares_no_floor and not args.quiet:
            print(f"  NOTE: {os.path.basename(args.pcb)} declares NO net class "
                  f"and NO board constraint (no sibling .kicad_pro, no "
                  f"(net_class) block). Every floor here is a FALLBACK, not "
                  f"this board's own: clearance {args.clearance:.4g} mm"
                  + (" (--clearance)" if _explicit_clearance
                     else " (check_drc default)")
                  + f", hole-to-hole {args.hole_to_hole_clearance:.4g} mm, "
                  f"board-edge {args.board_edge_clearance:.4g} mm. Whether "
                  f"they match what the copper was routed to is unverified "
                  f"HERE -- read the route step's --clearance from "
                  f"redo_commands.sh.")
    except Exception as _e:
        if not args.quiet:
            print(f"  (board floor declaration not read: {_e})")

    # Issue #326: per-netclass clearances -- KiCad grades every pair at the
    # max of the two items' netclass values, so read the board's classes
    # (explicit assignments + wildcard patterns) and grade the same way.
    # Issue #338: KiCad grades copper-to-edge at the board's
    # min_copper_edge_clearance; honor it unless --board-edge-clearance is
    # explicitly larger.
    net_clearances = None
    try:
        from list_nets import read_design_rules, net_clearance_map
        _rules = read_design_rules(args.pcb)
        if _rules.get('classes'):
            # Pass the detected file-format version: KiCad 10 dropped the
            # numbered net table, so the version-less extract_nets() call
            # returned ZERO nets on those boards and cross-class grading was
            # silently OFF (cparti step7b: KiCad GUI 92 violations, this
            # checker 0 -- the #344 numeric-net-matcher class of bug).
            from kicad_parser import extract_nets, detect_kicad_version
            with open(args.pcb, encoding='utf-8', errors='replace') as _f:
                _content = _f.read()
            _net_objs, _ = extract_nets(_content, detect_kicad_version(_content))
            del _content
            net_clearances = net_clearance_map(
                args.pcb, [n.name for n in _net_objs.values()],
                rules=_rules) or None
        # #603: a board minimum that overrides an EXPLICIT CLI value is
        # announced on stderr even under --quiet. Silently substituting is how
        # polykit_x_inputboard's `--hole-to-hole-clearance 0.2` (taken from
        # list_nets' then-wrong floor line) came back graded at 0.25 with
        # nothing in the output saying so -- the grader could not tell the
        # requested floor from the applied one. stderr keeps stdout
        # machine-readable for the callers that parse it.
        def _pin_up(attr, board_val, source, label):
            cur = getattr(args, attr)
            if board_val <= cur:
                return
            # Explicitness must come from the COMMAND LINE, not from comparing
            # against the default: --hole-to-hole-clearance 0.2 IS the default
            # value, and that is exactly the case in the field report.
            flag = '--' + attr.replace('_', '-')
            explicit = any(a == flag or a.startswith(flag + '=')
                           for a in sys.argv[1:])
            setattr(args, attr, board_val)
            msg = f"{label} {board_val:.4g} mm (from project {source})"
            if explicit:
                print(f"NOTE: {label} CLAMPED UP to {board_val:.4g} mm by the "
                      f"board's own {source} -- the requested {cur:.4g} mm is "
                      f"below a DRC-enforced minimum and cannot be graded at. "
                      f"Route at {board_val:.4g} too, or the grade will not "
                      f"match what was routed.", file=sys.stderr)
            elif not args.quiet:
                print(msg)

        _pin_up('board_edge_clearance',
                float(_rules.get('constraints', {})
                      .get('min_copper_edge_clearance') or 0.0),
                'min_copper_edge_clearance', 'Board-edge clearance')
        # #439: board-derive the hole-to-hole floor too (symmetry with edge above
        # and with the router, which pins it from min_hole_to_hole). Without this a
        # board declaring min_hole_to_hole > the 0.2 default is graded too loose and
        # a real hole-to-hole violation between 0.2 and the board value is missed.
        _pin_up('hole_to_hole_clearance',
                float(_rules.get('constraints', {})
                      .get('min_hole_to_hole') or 0.0),
                'min_hole_to_hole', 'Hole-to-hole clearance')
        # COPPER-to-hole, the third of the same family and the one that was
        # missing (#617). The two blocks above board-derive their floors; this
        # check did not, and graded every board at the hardcoded 0.20
        # NPTH_TO_TRACK_CLEARANCE instead -- so a board declaring
        # min_hole_clearance 0.25 had its authored 0.20-0.25 band graded clean
        # (neo6502: 3 NPTH holes, tightest 0.2126 mm). Routed through _pin_up
        # so it inherits #603's explicit-clamp announcement like the others.
        _pin_up('hole_clearance',
                float(_rules.get('constraints', {})
                      .get('min_hole_clearance') or 0.0),
                'min_hole_clearance', 'Copper-to-hole clearance')
    except Exception as e:
        if not args.quiet:
            print(f"  (netclass/edge rules not read: {e})")

    violations = run_drc(args.pcb, args.clearance, args.nets, args.debug_lines, args.quiet,
                         args.hole_to_hole_clearance, args.board_edge_clearance,
                         args.hole_clearance,
                         args.clearance_margin, max_print=args.max_print,
                         min_track_width=args.min_track_width,
                         min_via_diameter=args.min_via_diameter,
                         min_via_drill=args.min_via_drill,
                         check_sizes=not args.no_size_checks,
                         size_margin=args.size_margin,
                         check_pad_edge=args.check_pad_edge,
                         net_clearances=net_clearances)
    if args.render and any(not v.get('accepted') for v in violations):
        render_violation_panels(args.pcb, violations, args.render)
    if args.json:
        # A machine-readable result, because the copper-free placement gate is
        # consumed by a driver that REFUSES to proceed without it -- and a gate
        # whose evidence file cannot be produced is satisfiable only by
        # fabricating it. Count 'accepted' items separately: they are published
        # for other graders but are not failures, exactly as the exit status
        # below treats them.
        import collections as _c
        import json as _json
        _real = [v for v in violations if not v.get('accepted')]

        def _clearance_for(v):
            """The clearance THIS violation was graded against.

            Per-net clearances mean the graded floor is not one number, so a
            violation carrying its own `required_mm` wins; otherwise the run's
            clearance applies. Same relation the pad-pad `[SHORT]` tag uses.
            """
            r = v.get('required_mm')
            return r if isinstance(r, (int, float)) else args.clearance

        _doc = {
            'schema': 1,
            'tool': 'check_drc.py',
            'board': os.path.abspath(args.pcb),
            'graded_at': {
                'clearance': args.clearance,
                'clearance_margin': args.clearance_margin,
                'hole_to_hole_clearance': args.hole_to_hole_clearance,
                'board_edge_clearance': args.board_edge_clearance,
                'per_net_clearances': bool(net_clearances),
                'size_checks': not args.no_size_checks,
                # run-12 Tier 1.3: True when the BOARD declared no net class
                # and no constraint, so every floor above is this tool's
                # fallback rather than the board's own. A reader comparing
                # `graded_at` across boards cannot otherwise tell the two apart.
                'board_declares_no_floor': _board_declares_no_floor,
            },
            'violations': len(_real),
            'accepted': len(violations) - len(_real),
            'by_type': dict(_c.Counter(v.get('type') for v in _real)),
            # CONTACT, per type. `overlap_mm >= clearance` means the two pieces
            # of copper physically reach each other -- required_dist is
            # (width/2 + clearance) and overlap is required_dist minus the
            # EDGE-TO-EDGE distance, so the relation holds for every type, not
            # just pad-pad.
            #
            # Two boards once differed by a +1V2-to-signal short and reported
            # 38 violations with byte-identical `by_type`. The pad-pad SHORT
            # count was identical too (10 vs 10) -- every differing contact was
            # `pad-segment`, whose TOTAL was 8 on both. So neither the total nor
            # the obvious refinement could separate them, and the `[SHORT]` tag
            # is emitted at exactly one place in this file, inside the pad-pad
            # branch, and is structurally incapable of naming a track contact.
            # Only a PER-TYPE contact count does it.
            'contacts_by_type': dict(_c.Counter(
                v.get('type') for v in _real
                if isinstance(v.get('overlap_mm'), (int, float))
                and v['overlap_mm'] >= _clearance_for(v))),
            'items': [dict(item,
                           short=(isinstance(item.get('overlap_mm'), (int, float))
                                  and item['overlap_mm'] >= _clearance_for(item)))
                      for item in violations],
        }
        with open(args.json, 'w', encoding='utf-8') as _fh:
            _json.dump(_doc, _fh, indent=1, default=str, sort_keys=True)
        print(f"  JSON -> {args.json}")
    # 'accepted' items (e.g. a track covered by an edge-exempt pad) are published in
    # the return for other graders but are NOT failures -- exclude from exit status.
    sys.exit(1 if any(not v.get('accepted') for v in violations) else 0)

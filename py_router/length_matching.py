"""
Length matching for PCB routes using trombone-style meanders.

Adds length to shorter routes by inserting perpendicular zigzag patterns
at the longest straight segment.
"""
from __future__ import annotations

import fnmatch
import math
import re
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass

from kicad_parser import Segment, PCBData
from routing_config import GridRouteConfig
from routing_utils import segment_length
from net_queries import calculate_route_length, expand_pad_layers
from geometry_utils import (
    point_to_segment_distance,
    segments_intersect,
    segment_to_segment_distance,
)

# Meander geometry constants
CHAMFER_SIZE = 0.1  # mm - 45-degree chamfer size at meander corners
MIN_AMPLITUDE = 0.2  # mm - minimum useful meander amplitude
MIN_SEGMENT_LENGTH = 0.001  # mm - minimum meaningful segment length
COLINEAR_DOT_THRESHOLD = 0.99  # dot product threshold for colinearity check
POSITION_TOLERANCE = 0.01  # mm - tolerance for position comparisons
CORNER_BLOAT_FACTOR = 0.42  # sqrt(2) - 1, extra copper extension at 90-degree corners
SPATIAL_CELL_SIZE = 2.0  # mm - spatial index cell size


def _pad_bounding_radius(pad) -> float:
    """Conservative bounding-circle radius of a pad's copper (#526).

    The meander clearance checks modelled every pad as a circle of radius
    max(size_x, size_y)/2 -- but a RECT pad's corner reaches hypot(sx, sy)/2,
    up to sqrt(2) beyond that. A time-match arm approaching a roundrect
    corner-on passed the check while physically grazing the corner 59um deep
    (a13 ddr-a3 vs GND pad C210.2: model 0.57, true corner reach 0.789).
    Circle AND OVAL pads are exact at max/2: an oval is a stadium (a rect with
    semicircular ends), so its farthest copper is the end cap on the long axis --
    it has no corner to reach. Giving it the half-diagonal inflated every oval by
    up to sqrt(2), which is the shape of essentially every through-hole pad
    (measured: all 1225 ovals in kicad_files/, mean +274um, worst +966um), and it
    contradicted the convention this repo applies everywhere else -- see
    check_drc.py, obstacle_map.py, obstacle_cache.py, and especially
    diff_pair_routing.py's `short if pad.shape in ('circle','oval') else
    short * sqrt(2)`, which is this same corner correction.

    Every other shape gets the half-diagonal -- exact for rect, slightly
    conservative for roundrect (its corners are rounded by roundrect_rratio),
    which is the correct direction for a placement check.

    Known gap: a `custom` pad's copper lives in Pad.polygons, not size_x/size_y,
    so this UNDER-bounds it. Pre-existing and out of scope here, but this helper
    is where a fix belongs."""
    if getattr(pad, 'shape', '') in ('circle', 'oval'):
        return max(pad.size_x or 0, pad.size_y or 0) / 2.0
    return math.hypot(pad.size_x or 0, pad.size_y or 0) / 2.0


def resolve_meander_chamfer(
    config: Optional[GridRouteConfig],
    net_id: Optional[int] = None,
    layer: Optional[str] = None,
    own_width: float = 0.0,
) -> float:
    """Chamfer (= half the riser pitch) for single-ended meander arms (#501).

    Adjacent risers sit 2*chamfer centre-to-centre, so the fixed CHAMFER_SIZE
    packed arms 0.2mm apart regardless of track width. Same-net arms are
    excluded from every clearance checker by design, so the pitch arithmetic
    here is the only arm-spacing guarantee: pitch = config.meander_spacing *
    the net's actual routed width on this layer (own_width covers segments
    widened beyond their class).
    """
    if config is None:
        return CHAMFER_SIZE
    width = max(getattr(config, 'track_width', 0.0), own_width)
    if net_id is not None and layer is not None:
        width = max(width, config.get_net_track_width(net_id, layer))
    spacing = getattr(config, 'meander_spacing', 2.0)
    if not spacing or spacing <= 0:
        spacing = 2.0
    return max(CHAMFER_SIZE, spacing * width / 2.0)


def pair_leg_metric(result, metric_func, segments=None, vias=None):
    """Single-leg metric for a diff-pair result (#520).

    A pair result's ``new_segments``/``new_vias`` carry BOTH legs' copper, but
    its ``route_length`` and every group-matching target are SINGLE-leg totals
    (diff_pair_loop stamps ``max(p_total, n_total)`` or centerline+stubs).
    Feeding the whole segment list to a metric therefore measures ~2x the
    target scale, and the undershoot/scale loops compare incommensurate
    numbers. Measure each leg's own copper and take the max (the pair's
    limiting leg). Falls back to the whole-list metric when the result does
    not carry leg net ids (hybrid/fallback shapes)."""
    segs = result.get('new_segments', []) if segments is None else segments
    vs = (result.get('new_vias', []) if vias is None else vias) or []
    p_id = result.get('p_net_id')
    n_id = result.get('n_net_id')
    if p_id is None or n_id is None:
        return metric_func(segs, vs)
    return max(
        metric_func([s for s in segs if s.net_id == p_id],
                    [v for v in vs if v.net_id == p_id]),
        metric_func([s for s in segs if s.net_id == n_id],
                    [v for v in vs if v.net_id == n_id]))


def multipoint_span_metric(result, metric_func):
    """Metric of a pair's MATCHING SPAN. For a multipoint pair (a chain of
    coupled 2-point legs) the span is the LONGEST leg -- the longest MST edge,
    the same span the meander pass lengthens. Falls through to
    pair_leg_metric for 2-point results (no leg_results)."""
    legs = result.get('leg_results') if result.get('is_multipoint') else None
    if not legs:
        return pair_leg_metric(result, metric_func)
    return max(pair_leg_metric(lr, metric_func) for lr in legs)


class ClearanceIndex:
    """
    Spatial index for efficient clearance checking.

    Divides the board into cells and stores references to segments/vias/pads
    in each cell they overlap. This allows efficient queries for items within
    a specific region rather than checking all items.
    """

    def __init__(self, cell_size: float = SPATIAL_CELL_SIZE):
        self.cell_size = cell_size
        self.segment_cells: Dict[Tuple[int, int], List] = {}  # cell -> list of (seg, layer)
        self.via_cells: Dict[Tuple[int, int], List] = {}  # cell -> list of vias
        self.pad_cells: Dict[Tuple[int, int], List] = {}  # cell -> list of (pad, expanded_layers)
        self._pad_layer_cache: Dict[int, List[str]] = {}  # id(pad) -> expanded layers

    def _cell_key(self, x: float, y: float) -> Tuple[int, int]:
        """Convert (x, y) coordinates to cell key."""
        return (int(x / self.cell_size), int(y / self.cell_size))

    def _cells_for_segment(self, x1: float, y1: float, x2: float, y2: float, margin: float = 0) -> List[Tuple[int, int]]:
        """Get all cells that a segment (with margin) might touch."""
        min_x = min(x1, x2) - margin
        max_x = max(x1, x2) + margin
        min_y = min(y1, y2) - margin
        max_y = max(y1, y2) + margin

        min_cx, min_cy = self._cell_key(min_x, min_y)
        max_cx, max_cy = self._cell_key(max_x, max_y)

        cells = []
        for cx in range(min_cx, max_cx + 1):
            for cy in range(min_cy, max_cy + 1):
                cells.append((cx, cy))
        return cells

    def _cells_for_point(self, x: float, y: float, radius: float) -> List[Tuple[int, int]]:
        """Get all cells within radius of a point."""
        min_cx, min_cy = self._cell_key(x - radius, y - radius)
        max_cx, max_cy = self._cell_key(x + radius, y + radius)

        cells = []
        for cx in range(min_cx, max_cx + 1):
            for cy in range(min_cy, max_cy + 1):
                cells.append((cx, cy))
        return cells

    def build(self, pcb_data: PCBData, config: GridRouteConfig,
              extra_segments: List[Segment] = None, extra_vias: List = None):
        """
        Build the spatial index from PCB data.

        Args:
            pcb_data: PCB data with segments, vias, pads
            config: Routing configuration (for clearance margin in cell assignment)
            extra_segments: Additional segments to include
            extra_vias: Additional vias to include
        """
        # Clearance margin for cell assignment - ensures we find all potentially conflicting items
        margin = config.track_width + config.clearance + config.meander_amplitude

        # Index segments
        for seg in pcb_data.segments:
            cells = self._cells_for_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y, margin)
            for cell in cells:
                if cell not in self.segment_cells:
                    self.segment_cells[cell] = []
                self.segment_cells[cell].append(seg)

        if extra_segments:
            for seg in extra_segments:
                cells = self._cells_for_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y, margin)
                for cell in cells:
                    if cell not in self.segment_cells:
                        self.segment_cells[cell] = []
                    self.segment_cells[cell].append(seg)

        # Index vias
        via_margin = config.via_size / 2 + margin
        for via in pcb_data.vias:
            cells = self._cells_for_point(via.x, via.y, via_margin)
            for cell in cells:
                if cell not in self.via_cells:
                    self.via_cells[cell] = []
                self.via_cells[cell].append(via)

        if extra_vias:
            for via in extra_vias:
                cells = self._cells_for_point(via.x, via.y, via_margin)
                for cell in cells:
                    if cell not in self.via_cells:
                        self.via_cells[cell] = []
                    self.via_cells[cell].append(via)

        # Index pads and pre-compute expanded layers
        for pad_net_id, pad_list in pcb_data.pads_by_net.items():
            for pad in pad_list:
                pad_radius = _pad_bounding_radius(pad) + margin
                cells = self._cells_for_point(pad.global_x, pad.global_y, pad_radius)

                # Pre-compute expanded layers once per pad
                expanded = expand_pad_layers(pad.layers, config.layers)
                self._pad_layer_cache[id(pad)] = expanded

                for cell in cells:
                    if cell not in self.pad_cells:
                        self.pad_cells[cell] = []
                    self.pad_cells[cell].append((pad, expanded))

    def add_segments(self, segments: List[Segment], margin: float):
        """Incrementally add segments to the index after initial build."""
        for seg in segments:
            cells = self._cells_for_segment(seg.start_x, seg.start_y, seg.end_x, seg.end_y, margin)
            for cell in cells:
                if cell not in self.segment_cells:
                    self.segment_cells[cell] = []
                self.segment_cells[cell].append(seg)

    def query_segments(self, x1: float, y1: float, x2: float, y2: float, margin: float) -> List:
        """Get segments potentially within margin of a line segment."""
        cells = self._cells_for_segment(x1, y1, x2, y2, margin)
        seen = set()
        result = []
        for cell in cells:
            for seg in self.segment_cells.get(cell, []):
                seg_id = id(seg)
                if seg_id not in seen:
                    seen.add(seg_id)
                    result.append(seg)
        return result

    def query_vias(self, x1: float, y1: float, x2: float, y2: float, margin: float) -> List:
        """Get vias potentially within margin of a line segment."""
        cells = self._cells_for_segment(x1, y1, x2, y2, margin)
        seen = set()
        result = []
        for cell in cells:
            for via in self.via_cells.get(cell, []):
                via_id = id(via)
                if via_id not in seen:
                    seen.add(via_id)
                    result.append(via)
        return result

    def query_pads(self, x1: float, y1: float, x2: float, y2: float, margin: float) -> List[Tuple]:
        """Get pads (with expanded layers) potentially within margin of a line segment."""
        cells = self._cells_for_segment(x1, y1, x2, y2, margin)
        seen = set()
        result = []
        for cell in cells:
            for pad_tuple in self.pad_cells.get(cell, []):
                pad_id = id(pad_tuple[0])
                if pad_id not in seen:
                    seen.add(pad_id)
                    result.append(pad_tuple)
        return result


def _board_edge_geom(pcb_data):
    """(rings, outer_outline, cutouts) of the real Edge.Cuts geometry, cached on
    the PCBData object (amplitude probing runs per meander bump, so the rings are
    fetched many times per board). Empty rings = no usable outline; callers fall
    back to the board_bounds bbox test."""
    if pcb_data is None or getattr(pcb_data, 'board_info', None) is None:
        return [], None, []
    cached = getattr(pcb_data, '_meander_edge_geom', None)
    if cached is None:
        from check_drc import board_edge_geometry
        cached = board_edge_geometry(pcb_data.board_info)
        pcb_data._meander_edge_geom = cached
    return cached


def _bump_edge_clear(bump_segs, edge_geom, board_bounds, edge_margin) -> bool:
    """True when every bump segment stays ON the board and >= edge_margin from
    the real board outline AND every interior cutout (#370 B1, same class as the
    #256 octolinear fix): the old bbox test was blind to cutouts and non-
    rectangular outlines, so bumps could print over a switch window or past a
    curved edge. Bbox fallback when the board has no usable outline rings."""
    edge_rings, edge_outer, edge_cutouts = edge_geom
    if edge_rings:
        from check_drc import _point_on_board, _segment_to_rings_distance
        for bx1, by1, bx2, by2 in bump_segs:
            if not (_point_on_board(bx1, by1, edge_outer, edge_cutouts) and
                    _point_on_board(bx2, by2, edge_outer, edge_cutouts)):
                return False
            if _segment_to_rings_distance(bx1, by1, bx2, by2, edge_rings) < edge_margin:
                return False
        return True
    if board_bounds is not None:
        b_min_x = min(min(bx1, bx2) for bx1, by1, bx2, by2 in bump_segs)
        b_max_x = max(max(bx1, bx2) for bx1, by1, bx2, by2 in bump_segs)
        b_min_y = min(min(by1, by2) for bx1, by1, bx2, by2 in bump_segs)
        b_max_y = max(max(by1, by2) for bx1, by1, bx2, by2 in bump_segs)
        return not (b_min_x < board_bounds[0] + edge_margin or
                    b_max_x > board_bounds[2] - edge_margin or
                    b_min_y < board_bounds[1] + edge_margin or
                    b_max_y > board_bounds[3] - edge_margin)
    return True


# Spatial-query slack for foreign copper wider than the config defaults (#370
# B1): the per-object checks below grow the keep-out by (width - track_width)/2
# (or (via.size - via_size)/2), so the cell queries must reach at least that much
# further or an extra-wide trunk could be missed by the index scan entirely.
# 2.0mm covers a 4mm-wide strap; the index build margin adds more on top.
_FOREIGN_WIDTH_SLACK = 2.0


def get_bump_segments(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    amplitude: float,
    chamfer: float = 0.1,
    is_first_bump: bool = True
) -> List[Tuple[float, float, float, float]]:
    """
    Calculate the segments that would form a meander bump.

    Args:
        is_first_bump: If True, includes entry chamfer. If False, bump connects
                       directly from previous bump (no entry chamfer needed).

    Returns list of (x1, y1, x2, y2) tuples for each segment.
    """
    riser_height = amplitude - 2 * chamfer
    if riser_height < 0.1:
        riser_height = 0.1

    segments = []
    x, y = cx, cy

    # Entry 45° chamfer (only for first bump)
    if is_first_bump:
        nx = x + ux * chamfer + px * chamfer * direction
        ny = y + uy * chamfer + py * chamfer * direction
        segments.append((x, y, nx, ny))
        x, y = nx, ny

    # Riser away from centerline
    nx = x + px * riser_height * direction
    ny = y + py * riser_height * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Top chamfer 1
    nx = x + ux * chamfer + px * chamfer * direction
    ny = y + uy * chamfer + py * chamfer * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Top chamfer 2
    nx = x + ux * chamfer - px * chamfer * direction
    ny = y + uy * chamfer - py * chamfer * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # Riser back toward centerline
    nx = x - px * riser_height * direction
    ny = y - py * riser_height * direction
    segments.append((x, y, nx, ny))
    x, y = nx, ny

    # No exit chamfer - next bump connects directly
    # (Exit chamfer is only added at the very end of all meanders)

    return segments


def get_safe_amplitude_at_point(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    max_amplitude: float,
    min_amplitude: float,
    layer: str,
    pcb_data: PCBData,
    net_id: int,
    config: GridRouteConfig,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    is_first_bump: bool = True,
    paired_net_id: int = None,
    clearance_index: 'ClearanceIndex' = None,
    own_width: float = 0.0,
    chamfer: float = None
) -> float:
    """
    Find the maximum safe amplitude for a meander bump at a specific point.

    Args:
        cx, cy: Center point of the bump on the trace
        ux, uy: Unit vector along the trace direction
        px, py: Perpendicular unit vector
        direction: 1 for positive perpendicular, -1 for negative
        max_amplitude: Maximum amplitude to try
        min_amplitude: Minimum useful amplitude
        layer: Layer of the trace
        pcb_data: PCB data with all segments and vias
        net_id: Net ID of the route (to exclude self)
        config: Routing configuration
        extra_segments: Additional segments to check against (e.g., from other nets in same length-match pass)
        extra_vias: Additional vias to check against (e.g., from other nets in same length-match pass)
        paired_net_id: Optional paired net ID to also exclude (for intra-pair diff pair matching)
        clearance_index: Pre-built spatial index for efficient collision detection (optional)
        own_width: Actual width of the segment being meandered (mm). The meander arms
            inherit this width, so it drives the own-width term of the keep-out. Pass
            the segment's real width so a segment widened beyond its netclass (e.g. a
            hand-placed wide trunk) is honored, matching obstacle_cache's max(net_w,
            seg.width). 0 = fall back to the net's configured width only.
        chamfer: Chamfer size the caller will emit bumps with (#501). Pass the
            generator's resolved value so the checked geometry matches the
            emitted geometry; None = resolve from config/net width here.

    Returns:
        Safe amplitude, or 0 if no safe amplitude found
    """
    # Add margin to account for segment merging/optimization when writing output
    # The routing uses grid-snapped segments, but output merges them into longer
    # segments that may have slightly different coordinates (up to half a grid step)
    meander_clearance_margin = config.grid_step / 2
    # This net may be routed wider than the base track_width (impedance-controlled
    # --impedance layer widths, or a power net width). The meander arms carry the
    # net's ACTUAL width, so size the keep-out from it (#175). max() keeps the
    # common case (net_w == track_width) byte-identical; only wider nets grow their
    # keep-out. own_width is the actual width of the meandered segment, so a segment
    # widened beyond its netclass is honored too (mirrors obstacle_cache's
    # max(net_w, seg.width)). The neighbor half-width below starts from the
    # generic track_width/2 and is grown per-object to the FOREIGN copper's real
    # width/via size (#370 B1) -- max()'d with the config values, so the common
    # case stays byte-identical.
    net_w = max(config.track_width, config.get_net_track_width(net_id, layer), own_width)
    net_half = net_w / 2
    # Add corner margin for track width bloat at 45-degree chamfered corners
    corner_margin = net_half * CORNER_BLOAT_FACTOR
    required_clearance = net_half + config.track_width / 2 + config.clearance + meander_clearance_margin + corner_margin
    via_clearance = config.via_size / 2 + net_half + config.clearance + meander_clearance_margin + corner_margin
    paired_clearance = net_half + config.track_width / 2 + config.clearance
    if chamfer is None:
        chamfer = resolve_meander_chamfer(config, net_id, layer, own_width)

    # Board-edge keep-out (meanders overran the board outline). The A* router avoids
    # the edge via add_board_edge_obstacles, but this geometric meander pass never
    # did, so bumps near the edge printed past it. A bump arm carries the net's
    # width, so its centerline must stay edge_clearance + net_half inside the REAL
    # outline/cutout geometry (#370 B1; bbox fallback when no rings parsed).
    # board_edge_clearance overrides config.clearance when set, matching
    # add_board_edge_obstacles.
    board_bounds = getattr(pcb_data.board_info, 'board_bounds', None) if pcb_data and pcb_data.board_info else None
    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    edge_margin = edge_clearance + net_half
    edge_geom = _board_edge_geom(pcb_data)

    # Binary search for safe amplitude
    # Start with max_amplitude and reduce if there are conflicts
    test_amplitudes = [max_amplitude]
    # Add intermediate values for binary search
    amp = max_amplitude
    while amp > min_amplitude:
        amp *= 0.7
        if amp >= min_amplitude:
            test_amplitudes.append(amp)
    test_amplitudes.append(min_amplitude)

    for test_amp in test_amplitudes:
        # Generate bump segments for this amplitude
        bump_segs = get_bump_segments(cx, cy, ux, uy, px, py, direction, test_amp, chamfer, is_first_bump)

        conflict_found = False

        # Calculate bump bounding box for spatial queries
        bump_min_x = min(min(bx1, bx2) for bx1, by1, bx2, by2 in bump_segs)
        bump_max_x = max(max(bx1, bx2) for bx1, by1, bx2, by2 in bump_segs)
        bump_min_y = min(min(by1, by2) for bx1, by1, bx2, by2 in bump_segs)
        bump_max_y = max(max(by1, by2) for bx1, by1, bx2, by2 in bump_segs)

        # Reject amplitudes whose bump would print past the board edge or over an
        # interior cutout (real Edge.Cuts geometry; bbox fallback, #370 B1).
        if not _bump_edge_clear(bump_segs, edge_geom, board_bounds, edge_margin):
            continue  # try a smaller amplitude that stays inside the outline

        # Use spatial index if available, otherwise fall back to full iteration
        if clearance_index is not None:
            # Query segments in the bump region
            nearby_segments = clearance_index.query_segments(
                bump_min_x, bump_min_y, bump_max_x, bump_max_y,
                required_clearance + _FOREIGN_WIDTH_SLACK
            )

            # Check each bump segment against nearby segments on the same layer
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break

                for other_seg in nearby_segments:
                    # Layer check first (most likely to skip)
                    if other_seg.layer != layer:
                        continue
                    if other_seg.net_id == net_id:
                        continue

                    # For paired net (intra-pair meanders), use minimum clearance
                    if other_seg.net_id == paired_net_id:
                        check_clearance = paired_clearance
                    else:
                        check_clearance = required_clearance
                    # Foreign copper at its REAL width (#370 B1): a wide power
                    # trunk's edge extends (width - track_width)/2 beyond the
                    # generic keep-out. max() keeps the common case identical.
                    check_clearance += max(0.0, (other_seg.width - config.track_width) / 2.0)

                    # Check segment-to-segment distance
                    dist = segment_to_segment_distance(
                        bx1, by1, bx2, by2,
                        other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                    )

                    if dist < check_clearance:
                        conflict_found = True
                        break

            # Check bump segments against nearby vias
            if not conflict_found:
                nearby_vias = clearance_index.query_vias(
                    bump_min_x, bump_min_y, bump_max_x, bump_max_y,
                    via_clearance + _FOREIGN_WIDTH_SLACK
                )

                for bx1, by1, bx2, by2 in bump_segs:
                    if conflict_found:
                        break

                    for via in nearby_vias:
                        if via.net_id == net_id:
                            continue

                        # Distance from via center to bump segment; the FOREIGN
                        # via's real size when larger than the default (#370 B1).
                        dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                        if dist < via_clearance + max(0.0, ((getattr(via, 'size', 0) or 0) - config.via_size) / 2.0):
                            conflict_found = True
                            break

            # Check bump segments against nearby pads (on same layer)
            if not conflict_found:
                pad_clearance = net_half + config.clearance + corner_margin
                nearby_pads = clearance_index.query_pads(
                    bump_min_x, bump_min_y, bump_max_x, bump_max_y, pad_clearance + 2.0
                )

                for bx1, by1, bx2, by2 in bump_segs:
                    if conflict_found:
                        break
                    for pad, expanded_pad_layers in nearby_pads:
                        if layer not in expanded_pad_layers:
                            continue
                        # Treat pad as circle with radius = max(size_x, size_y)/2
                        pad_radius = _pad_bounding_radius(pad)
                        dist = point_to_segment_distance(pad.global_x, pad.global_y, bx1, by1, bx2, by2)
                        if dist < pad_radius + pad_clearance:
                            conflict_found = True
                            break

        else:
            # Fallback: iterate all segments (slower, but works without index)
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break

                for other_seg in pcb_data.segments:
                    # Layer check first (most likely to skip)
                    if other_seg.layer != layer:
                        continue
                    if other_seg.net_id == net_id:
                        continue

                    # For paired net (intra-pair meanders), use minimum clearance
                    if other_seg.net_id == paired_net_id:
                        check_clearance = paired_clearance
                    else:
                        check_clearance = required_clearance
                    # Foreign copper at its REAL width (#370 B1).
                    check_clearance += max(0.0, (other_seg.width - config.track_width) / 2.0)

                    # Quick distance check
                    seg_center_x = (other_seg.start_x + other_seg.end_x) / 2
                    seg_center_y = (other_seg.start_y + other_seg.end_y) / 2
                    seg_half_len = math.sqrt((other_seg.end_x - other_seg.start_x)**2 +
                                             (other_seg.end_y - other_seg.start_y)**2) / 2
                    bump_center_x = (bx1 + bx2) / 2
                    bump_center_y = (by1 + by2) / 2
                    rough_dist = math.sqrt((bump_center_x - seg_center_x)**2 + (bump_center_y - seg_center_y)**2)

                    if rough_dist > test_amp + seg_half_len + check_clearance + 1.0:
                        continue

                    # Check segment-to-segment distance
                    dist = segment_to_segment_distance(
                        bx1, by1, bx2, by2,
                        other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                    )

                    if dist < check_clearance:
                        conflict_found = True
                        break

                # Check extra_segments too
                if not conflict_found and extra_segments:
                    for other_seg in extra_segments:
                        if other_seg.layer != layer:
                            continue
                        if other_seg.net_id == net_id:
                            continue

                        if other_seg.net_id == paired_net_id:
                            check_clearance = paired_clearance
                        else:
                            check_clearance = required_clearance
                        # Foreign copper at its REAL width (#370 B1).
                        check_clearance += max(0.0, (other_seg.width - config.track_width) / 2.0)

                        dist = segment_to_segment_distance(
                            bx1, by1, bx2, by2,
                            other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                        )

                        if dist < check_clearance:
                            conflict_found = True
                            break

            # Check bump segments against vias
            if not conflict_found:
                for bx1, by1, bx2, by2 in bump_segs:
                    if conflict_found:
                        break

                    for via in pcb_data.vias:
                        if via.net_id == net_id:
                            continue

                        dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                        # The FOREIGN via's real size when larger (#370 B1).
                        if dist < via_clearance + max(0.0, ((getattr(via, 'size', 0) or 0) - config.via_size) / 2.0):
                            conflict_found = True
                            break

                    # Check extra_vias too
                    if not conflict_found and extra_vias:
                        for via in extra_vias:
                            if via.net_id == net_id:
                                continue

                            dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                            if dist < via_clearance + max(0.0, ((getattr(via, 'size', 0) or 0) - config.via_size) / 2.0):
                                conflict_found = True
                                break

            # Check bump segments against pads (on same layer)
            if not conflict_found:
                pad_clearance = net_half + config.clearance + corner_margin
                for bx1, by1, bx2, by2 in bump_segs:
                    if conflict_found:
                        break
                    for pad_net_id, pad_list in pcb_data.pads_by_net.items():
                        if pad_net_id == net_id:
                            continue
                        if conflict_found:
                            break
                        for pad in pad_list:
                            expanded_pad_layers = expand_pad_layers(pad.layers, config.layers)
                            if layer not in expanded_pad_layers:
                                continue
                            pad_radius = _pad_bounding_radius(pad)
                            dist = point_to_segment_distance(pad.global_x, pad.global_y, bx1, by1, bx2, by2)
                            if dist < pad_radius + pad_clearance:
                                conflict_found = True
                                break

        if not conflict_found:
            return test_amp

    # All amplitudes had conflicts
    return 0


def segments_are_colinear(seg1: Segment, seg2: Segment, tolerance: float = POSITION_TOLERANCE) -> bool:
    """
    Check if two segments are colinear (same direction and connected).

    Args:
        seg1: First segment
        seg2: Second segment (should start where seg1 ends)
        tolerance: Position tolerance in mm

    Returns:
        True if segments are colinear and connected
    """
    # Check if seg2 starts where seg1 ends
    if abs(seg1.end_x - seg2.start_x) > tolerance or abs(seg1.end_y - seg2.start_y) > tolerance:
        return False

    # Check if same layer
    if seg1.layer != seg2.layer:
        return False

    # Calculate direction vectors
    dx1 = seg1.end_x - seg1.start_x
    dy1 = seg1.end_y - seg1.start_y
    dx2 = seg2.end_x - seg2.start_x
    dy2 = seg2.end_y - seg2.start_y

    # Normalize
    len1 = math.sqrt(dx1*dx1 + dy1*dy1)
    len2 = math.sqrt(dx2*dx2 + dy2*dy2)

    if len1 < MIN_SEGMENT_LENGTH or len2 < MIN_SEGMENT_LENGTH:
        return False

    dx1, dy1 = dx1/len1, dy1/len1
    dx2, dy2 = dx2/len2, dy2/len2

    # Check if same direction (dot product close to 1)
    dot = dx1*dx2 + dy1*dy2
    return dot > COLINEAR_DOT_THRESHOLD


def generate_trombone_meander(
    segment: Segment,
    extra_length: float,
    amplitude: float,
    track_width: float,
    pcb_data: PCBData = None,
    config: GridRouteConfig = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0,
    paired_net_id: int = None,
    clearance_index: 'ClearanceIndex' = None
) -> Tuple[List[Segment], int]:
    """
    Generate trombone-style meander segments to replace a straight segment.

    The meander creates simple up-down bumps with 45° chamfered corners:

    Original:  ────────────────────────────────────>

    Meander:   ──╮╭╮╭╮╭──>
                 ││││││
                 ╰╯╰╯╰╯

    Each "bump" consists of:
    - 45° corner going perpendicular
    - Straight segment perpendicular to trace (the "riser" going up)
    - 45° corner at top (U-turn)
    - Straight segment back down (the "riser" going down)
    - 45° corner returning to trace

    Args:
        segment: Original segment to replace
        extra_length: Additional length to add (mm)
        amplitude: Maximum height of meander perpendicular to trace (mm)
        track_width: Track width for new segments (mm)
        pcb_data: PCB data for per-bump clearance checking (optional)
        config: Routing configuration for clearance checking (optional)
        extra_segments: Additional segments to check against (e.g., from other nets already processed)
        extra_vias: Additional vias to check against (e.g., from other nets already processed)
        min_bumps: Minimum number of bumps to generate (0 = use extra_length to determine)
        paired_net_id: Optional paired net ID to also exclude (for intra-pair diff pair matching)

    Returns:
        Tuple of (segments, bump_count) - the meander segments and number of bumps added
    """
    if extra_length <= 0 and min_bumps <= 0:
        return [segment], 0

    # Calculate segment direction
    dx = segment.end_x - segment.start_x
    dy = segment.end_y - segment.start_y
    seg_len = math.sqrt(dx * dx + dy * dy)

    if seg_len < MIN_SEGMENT_LENGTH:
        return [segment], 0

    # Unit vectors along and perpendicular to segment
    ux = dx / seg_len
    uy = dy / seg_len
    # Perpendicular (90° counterclockwise)
    px = -uy
    py = ux

    # 45° chamfer size = half the riser pitch: scale it with the net's routed
    # width so adjacent arms keep their spacing at any width (#501)
    chamfer = resolve_meander_chamfer(config, segment.net_id, segment.layer, segment.width)
    min_amplitude = MIN_AMPLITUDE  # mm - minimum useful amplitude

    # Horizontal distance consumed by one bump (just the chamfers' horizontal components)
    bump_width = 4 * chamfer

    # Estimate number of bumps we can fit
    max_bumps = int(seg_len * 0.9 / bump_width)
    if max_bumps < 1:
        return [segment], 0

    # Calculate how much extra length we need per bump on average
    target_extra_per_bump = extra_length / max_bumps

    # Each bump adds approximately 2 * (amplitude - 2*chamfer) extra length
    # So target_amplitude = target_extra_per_bump / 2 + 2*chamfer
    # But we cap at the configured amplitude

    # Build segment list
    new_segments = []

    # Current position - start at segment start
    cx = segment.start_x
    cy = segment.start_y

    # Track how much extra length we've added
    total_extra_added = 0.0

    # Generate meander bumps
    direction = 1  # Alternates: 1 = up (positive perpendicular), -1 = down
    prev_bump_direction = None  # Track previous bump direction for same-direction spacing
    blocked_direction = None  # Track which direction is completely blocked (safe_amp=0)
    # Which perpendicular side (+1/-1) the meander baseline currently sits at,
    # or None when the cursor is on the original centerline. After a bump the
    # cursor rests chamfer (0.1mm) off the centerline; copper emitted there
    # WITHOUT a clearance check (the skip-forward walk, the lead-out) can graze
    # the very neighbor that blocked the bumps (#449: sata_sniffer skip trains
    # printed 0.1mm from an adjacent DDR net's trunk -> 151 shorting_items).
    offset_dir = None
    bump_count = 0

    # Leave some margin at start and end
    margin = bump_width

    # Straight lead-in
    if margin > POSITION_TOLERANCE:
        end_x = cx + ux * margin
        end_y = cy + uy * margin
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=end_x, end_y=end_y,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = end_x, end_y

    # Keep adding bumps until we've added enough length or run out of space
    # If min_bumps > 0: generate exactly min_bumps (for amplitude scaling - don't let extra_length add more)
    # If min_bumps == 0: use extra_length to determine bump count
    def should_continue():
        if min_bumps > 0:
            return bump_count < min_bumps
        else:
            return total_extra_added < extra_length

    while should_continue():
        # Check if we have room for another bump
        dist_to_end = math.sqrt((segment.end_x - cx)**2 + (segment.end_y - cy)**2)
        if dist_to_end < bump_width + margin:
            break

        # Determine amplitude for this bump
        bump_amplitude = amplitude

        # A bump starting from the centerline (the very first, or the first
        # after a skip returned the cursor to the centerline) needs an entry
        # chamfer; bumps chained from the ±chamfer baseline don't.
        is_first = (offset_dir is None)
        has_entry_chamfer = is_first

        # Calculate the ACTUAL bump start position (after any same-direction spacing)
        # This is where the clearance check should be done
        check_cx, check_cy = cx, cy

        # For same-direction bumps, we need to account for the spacing chamfers
        # that will be added before the bump. The bump will start 2*chamfer further along.
        needs_same_dir_spacing = (prev_bump_direction is not None and prev_bump_direction == direction)
        if needs_same_dir_spacing:
            # The bump will start after exit chamfer + entry chamfer = 2*chamfer forward
            check_cx = cx + ux * 2 * chamfer
            check_cy = cy + uy * 2 * chamfer

        # If we have clearance checking, find safe amplitude at the ACTUAL bump position
        if pcb_data is not None and config is not None:
            safe_amp = get_safe_amplitude_at_point(
                check_cx, check_cy, ux, uy, px, py, direction, amplitude, min_amplitude,
                segment.layer, pcb_data, segment.net_id, config, extra_segments, extra_vias, is_first,
                paired_net_id=paired_net_id, clearance_index=clearance_index,
                own_width=segment.width, chamfer=chamfer
            )
            if safe_amp < min_amplitude:
                # Try the other direction
                # For the other direction, same-direction spacing might not be needed
                other_dir = -direction
                other_needs_spacing = (prev_bump_direction is not None and prev_bump_direction == other_dir)
                if other_needs_spacing:
                    other_check_cx = cx + ux * 2 * chamfer
                    other_check_cy = cy + uy * 2 * chamfer
                else:
                    other_check_cx, other_check_cy = cx, cy

                safe_amp_other = get_safe_amplitude_at_point(
                    other_check_cx, other_check_cy, ux, uy, px, py, other_dir, amplitude, min_amplitude,
                    segment.layer, pcb_data, segment.net_id, config, extra_segments, extra_vias, is_first,
                    paired_net_id=paired_net_id, clearance_index=clearance_index,
                    own_width=segment.width, chamfer=chamfer
                )
                if safe_amp_other >= min_amplitude:
                    # Mark the original direction as blocked if safe_amp was 0
                    # But NOT for intra-pair matching (paired_net_id set) - the paired track
                    # blocks full bumps but exit chamfers back to centerline are always valid
                    # since the original track position is already properly spaced
                    if safe_amp == 0 and paired_net_id is None:
                        blocked_direction = direction
                    direction = other_dir
                    safe_amp = safe_amp_other
                    needs_same_dir_spacing = other_needs_spacing
                    check_cx, check_cy = other_check_cx, other_check_cy
                else:
                    # No room for a bump here. If the cursor sits at the
                    # ±chamfer meander baseline, chamfer back to the original
                    # centerline FIRST: the skip segments below are emitted
                    # without any clearance check, and walking them at the
                    # offset lays copper 0.1mm off the DRC-clean route -- into
                    # the clearance band of the neighbor that blocked the
                    # bumps (#449). The centerline is the original routed
                    # corridor, so skipping along it is safe by construction.
                    if offset_dir is not None:
                        new_x = cx + ux * chamfer - px * chamfer * offset_dir
                        new_y = cy + uy * chamfer - py * chamfer * offset_dir
                        new_segments.append(Segment(
                            start_x=cx, start_y=cy,
                            end_x=new_x, end_y=new_y,
                            width=segment.width, layer=segment.layer, net_id=segment.net_id
                        ))
                        cx, cy = new_x, new_y
                        offset_dir = None
                        prev_bump_direction = None  # next bump re-enters with its own chamfer
                        continue
                    # On the centerline: skip forward with a straight segment
                    skip_dist = 0.2
                    new_x = cx + ux * skip_dist
                    new_y = cy + uy * skip_dist
                    new_segments.append(Segment(
                        start_x=cx, start_y=cy,
                        end_x=new_x, end_y=new_y,
                        width=segment.width, layer=segment.layer, net_id=segment.net_id
                    ))
                    cx, cy = new_x, new_y
                    continue

            bump_amplitude = min(amplitude, safe_amp)

        riser_height = bump_amplitude - 2 * chamfer
        if riser_height < 0.1:
            riser_height = 0.1
            bump_amplitude = riser_height + 2 * chamfer

        # Calculate extra length added by this bump
        # With entry/exit chamfers: 4 chamfers + 2 risers
        # Without inter-bump chamfers: 2 top chamfers + 2 risers (for middle bumps)
        chamfer_diag = chamfer * math.sqrt(2)

        if has_entry_chamfer:
            # Full bump with entry chamfer
            bump_path_length = 3 * chamfer_diag + 2 * riser_height  # entry + 2 top chamfers + risers
        else:
            # No entry chamfer - connect directly from previous bump's end
            bump_path_length = 2 * chamfer_diag + 2 * riser_height  # just 2 top chamfers + risers

        # Horizontal distance consumed (for checking if we have room)
        if has_entry_chamfer:
            this_bump_width = 3 * chamfer  # entry chamfer + 2 top chamfers
        else:
            this_bump_width = 2 * chamfer  # just 2 top chamfers

        extra_this_bump = bump_path_length - this_bump_width

        # Generate this bump

        # Add chamfers between same-direction bumps to prevent risers from touching
        # This happens when clearance checking forces all bumps to one side (e.g., intra-pair matching)
        if needs_same_dir_spacing:
            # After a bump, we're at +chamfer offset from centerline.
            # Add exit chamfer (to centerline) + entry chamfer (back to offset) to separate bumps

            # Check if chamfers would go into blocked direction - use flat segments instead
            exit_goes_blocked = (blocked_direction is not None and -prev_bump_direction == blocked_direction)

            if exit_goes_blocked:
                # Can't return to centerline (blocked direction), so stay at current offset
                # Use a single flat segment for separation - we're already at the correct offset
                # for the next bump since both bumps go in the same (unblocked) direction
                nx = cx + ux * 2 * chamfer  # Double length to maintain spacing
                ny = cy + uy * 2 * chamfer
                new_segments.append(Segment(
                    start_x=cx, start_y=cy,
                    end_x=nx, end_y=ny,
                    width=segment.width, layer=segment.layer, net_id=segment.net_id
                ))
                cx, cy = nx, ny
            else:
                # Normal case: exit chamfer returns to centerline, entry goes to new offset
                # Exit chamfer (return to centerline)
                nx = cx + ux * chamfer - px * chamfer * prev_bump_direction
                ny = cy + uy * chamfer - py * chamfer * prev_bump_direction
                new_segments.append(Segment(
                    start_x=cx, start_y=cy,
                    end_x=nx, end_y=ny,
                    width=segment.width, layer=segment.layer, net_id=segment.net_id
                ))
                cx, cy = nx, ny

                # Entry chamfer (go to offset for next bump's riser)
                nx = cx + ux * chamfer + px * chamfer * direction
                ny = cy + uy * chamfer + py * chamfer * direction
                new_segments.append(Segment(
                    start_x=cx, start_y=cy,
                    end_x=nx, end_y=ny,
                    width=segment.width, layer=segment.layer, net_id=segment.net_id
                ))
                cx, cy = nx, ny

        # Entry chamfer (only for bumps starting from the centerline)
        if has_entry_chamfer:
            offset_dir = direction  # baseline moves to ±chamfer off centerline
            # Direction switching already ensured we're going in the unblocked direction
            nx = cx + ux * chamfer + px * chamfer * direction
            ny = cy + uy * chamfer + py * chamfer * direction
            new_segments.append(Segment(
                start_x=cx, start_y=cy,
                end_x=nx, end_y=ny,
                width=segment.width, layer=segment.layer, net_id=segment.net_id
            ))
            cx, cy = nx, ny

        # Riser going away from centerline
        nx = cx + px * riser_height * direction
        ny = cy + py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Top 45° chamfer 1 (continuing away from centerline + forward)
        nx = cx + ux * chamfer + px * chamfer * direction
        ny = cy + uy * chamfer + py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Top 45° chamfer 2 (now returning toward centerline + forward)
        nx = cx + ux * chamfer - px * chamfer * direction
        ny = cy + uy * chamfer - py * chamfer * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # Riser going back toward centerline
        nx = cx - px * riser_height * direction
        ny = cy - py * riser_height * direction
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

        # No exit chamfer - the next bump connects directly here
        # (Exit chamfer will be added after the loop for the last bump)

        # Track progress
        total_extra_added += extra_this_bump
        bump_count += 1
        prev_bump_direction = direction  # Remember direction for same-direction spacing check

        # Alternate direction for next bump
        direction *= -1

    # Add exit chamfer to return to centerline. offset_dir tracks which side
    # the baseline sits at (None = a skip already returned us to centerline).
    # Always exit to the centerline: the old "blocked direction" flat kept the
    # tail at the offset, sending the unchecked lead-out 0.1mm off the routed
    # corridor (#449) -- returning TO the original centerline is safe even when
    # full bumps toward that side don't fit.
    if bump_count > 0 and offset_dir is not None:
        nx = cx + ux * chamfer - px * chamfer * offset_dir
        ny = cy + uy * chamfer - py * chamfer * offset_dir
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=nx, end_y=ny,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))
        cx, cy = nx, ny

    # Straight lead-out to segment end
    # Calculate remaining distance to end
    remaining_x = segment.end_x - cx
    remaining_y = segment.end_y - cy
    remaining_dist = math.sqrt(remaining_x**2 + remaining_y**2)

    if remaining_dist > POSITION_TOLERANCE:
        new_segments.append(Segment(
            start_x=cx, start_y=cy,
            end_x=segment.end_x, end_y=segment.end_y,
            width=segment.width, layer=segment.layer, net_id=segment.net_id
        ))

    return new_segments, bump_count


def get_segment_centerline_range(
    segments: List[Segment],
    start_idx: int,
    end_idx: int,
    main_ux: float,
    main_uy: float,
    origin_x: float,
    origin_y: float
) -> Tuple[float, float]:
    """
    Get the centerline position range for a segment run.

    Args:
        segments: All segments
        start_idx, end_idx: Run indices
        main_ux, main_uy: Main direction unit vector
        origin_x, origin_y: Origin point for projection

    Returns:
        (start_pos, end_pos) along centerline
    """
    first_seg = segments[start_idx]
    last_seg = segments[end_idx]

    start_proj = (first_seg.start_x - origin_x) * main_ux + (first_seg.start_y - origin_y) * main_uy
    end_proj = (last_seg.end_x - origin_x) * main_ux + (last_seg.end_y - origin_y) * main_uy

    return (min(start_proj, end_proj), max(start_proj, end_proj))


def ranges_overlap(range1: Tuple[float, float], range2: Tuple[float, float]) -> bool:
    """Check if two ranges overlap."""
    return range1[0] < range2[1] and range2[0] < range1[1]


def find_all_straight_runs(segments: List[Segment], min_length: float = 1.0) -> List[Tuple[int, int, float]]:
    """
    Find all straight runs of colinear segments, sorted by length descending.

    Args:
        segments: List of route segments
        min_length: Minimum run length to consider (mm)

    Returns:
        List of (start_idx, end_idx, length) tuples, sorted by length descending
    """
    if not segments:
        return []

    runs = []
    i = 0
    while i < len(segments):
        run_start = i
        run_length = segment_length(segments[i])

        j = i + 1
        while j < len(segments) and segments_are_colinear(segments[j-1], segments[j]):
            run_length += segment_length(segments[j])
            j += 1

        if run_length >= min_length:
            runs.append((run_start, j - 1, run_length))

        i = j

    # Sort by length descending
    runs.sort(key=lambda x: x[2], reverse=True)
    return runs


def apply_meanders_to_route(
    segments: List[Segment],
    extra_length: float,
    config: GridRouteConfig,
    pcb_data: PCBData = None,
    net_id: int = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0,
    amplitude_override: float = None,
    paired_net_id: int = None,
    excluded_centerline_ranges: List[Tuple[float, float]] = None,
    clearance_index: 'ClearanceIndex' = None
) -> Tuple[List[Segment], int]:
    """
    Apply meanders to a route to add extra length.

    Args:
        segments: Original route segments
        extra_length: Length to add (mm)
        config: Routing configuration
        pcb_data: PCB data for clearance checking (optional)
        net_id: Net ID of this route (optional, for clearance checking)
        extra_segments: Additional segments to check against (e.g., from already-processed nets)
        extra_vias: Additional vias to check against (e.g., from already-processed nets)
        min_bumps: Minimum number of bumps to generate, as a TOTAL across all
            straight runs the meanders spill over (#501)
        amplitude_override: Override amplitude (for scaling down to hit target length)
        paired_net_id: Optional paired net ID to also exclude (for intra-pair diff pair matching)
        excluded_centerline_ranges: List of (start, end) position ranges to avoid (e.g., inter-pair meanders)
        clearance_index: Pre-built spatial index for clearance checking (optional, built if not provided)

    Returns:
        Tuple of (modified segment list, total bump_count across all runs)
    """
    if extra_length <= 0 or not segments:
        return segments, 0

    # Use override amplitude if provided
    amplitude = amplitude_override if amplitude_override is not None else config.meander_amplitude

    # Use provided index or build one for efficient clearance checking
    if clearance_index is None and pcb_data is not None and config is not None:
        clearance_index = ClearanceIndex()
        clearance_index.build(pcb_data, config, extra_segments, extra_vias)

    # Calculate main direction for centerline projection (if we have excluded ranges)
    main_ux, main_uy, origin_x, origin_y = 0, 0, 0, 0
    if excluded_centerline_ranges:
        first_seg = segments[0]
        last_seg = segments[-1]
        main_dx = last_seg.end_x - first_seg.start_x
        main_dy = last_seg.end_y - first_seg.start_y
        main_len = math.sqrt(main_dx**2 + main_dy**2)
        if main_len > 0.001:
            main_ux = main_dx / main_len
            main_uy = main_dy / main_len
            origin_x = first_seg.start_x
            origin_y = first_seg.start_y

    min_length = amplitude * 2  # Reduced minimum for more options

    # Spill across multiple straight runs (#501): with the arm pitch scaled to
    # the net's width, one run holds fewer bumps, so a single run often cannot
    # satisfy extra_length (or min_bumps) alone. Each round meanders the best
    # available run, then re-finds straight runs on the MODIFIED list -- the
    # meandered region is no longer colinear so it will not be re-picked, but
    # its lead-in/lead-out and every untouched run remain candidates. min_bumps
    # is a TOTAL across runs (remaining = requested - placed so far).
    current_segments = segments
    original_length = sum(segment_length(s) for s in segments)
    total_bumps = 0
    added_so_far = 0.0
    max_spill_rounds = 10  # cap: a round that stalls breaks out below anyway

    for round_idx in range(max_spill_rounds):
        remaining_bumps = (min_bumps - total_bumps) if min_bumps > 0 else 0
        remaining_extra = extra_length - added_so_far
        if min_bumps > 0:
            if remaining_bumps <= 0:
                break
        elif remaining_extra <= POSITION_TOLERANCE:
            break

        runs = find_all_straight_runs(current_segments, min_length=min_length)
        if not runs:
            if round_idx == 0:
                print(f"    Warning: No suitable straight run found for meanders (need >= {min_length:.1f}mm)")
                return segments, 0
            break

        # Track run status for verbose output
        run_status = []  # List of (length, status): 'inter-meander', 'no-bumps', 'used', 'too-short'
        round_bumps = 0

        for start_idx, end_idx, run_length in runs:
            # Check if this run is long enough for meanders
            if run_length < amplitude * 2:
                run_status.append((run_length, 'too-short'))
                continue

            # Check if this run overlaps with excluded centerline ranges (e.g., inter-pair meanders)
            if excluded_centerline_ranges and (main_ux != 0 or main_uy != 0):
                run_range = get_segment_centerline_range(
                    current_segments, start_idx, end_idx, main_ux, main_uy, origin_x, origin_y
                )
                overlaps = any(ranges_overlap(run_range, excluded) for excluded in excluded_centerline_ranges)
                if overlaps:
                    run_status.append((run_length, 'inter-meander'))
                    continue

            first_seg = current_segments[start_idx]
            last_seg = current_segments[end_idx]

            # The run must hold lead-in + one bump + end margin at THIS net's
            # width-scaled pitch (#501), or the generator returns zero bumps.
            chamfer = resolve_meander_chamfer(config, first_seg.net_id, first_seg.layer, first_seg.width)
            if run_length < 3 * (4 * chamfer):
                run_status.append((run_length, 'too-short'))
                continue

            # Create a merged segment from the run
            merged_seg = Segment(
                start_x=first_seg.start_x,
                start_y=first_seg.start_y,
                end_x=last_seg.end_x,
                end_y=last_seg.end_y,
                width=first_seg.width,
                layer=first_seg.layer,
                net_id=first_seg.net_id
            )

            # Generate meanders - per-bump clearance checking will adjust amplitudes
            meander_segs, bump_count = generate_trombone_meander(
                merged_seg,
                remaining_extra,
                amplitude,
                config.track_width,
                pcb_data=pcb_data,
                config=config,
                extra_segments=extra_segments,
                extra_vias=extra_vias,
                min_bumps=remaining_bumps,
                paired_net_id=paired_net_id,
                clearance_index=clearance_index
            )

            if bump_count == 0:
                # Per-bump clearance checking rejected all positions
                run_status.append((run_length, 'no-bumps'))
                continue

            run_status.append((run_length, 'used'))

            # Replace original segment run with meanders
            current_segments = current_segments[:start_idx] + meander_segs + current_segments[end_idx + 1:]
            round_bumps = bump_count
            break

        # Print verbose status when there were rejections before success, or
        # when no run in this round worked at all
        if config and config.verbose and run_status and (round_bumps == 0 or len(run_status) > 1):
            status_str = ', '.join(f"{length:.2f}({status})" for length, status in run_status)
            print(f"      Straight runs: {status_str}")

        if round_bumps == 0:
            break  # no run can take another bump; keep what earlier rounds placed

        total_bumps += round_bumps
        new_added = sum(segment_length(s) for s in current_segments) - original_length
        if min_bumps <= 0 and new_added - added_so_far < POSITION_TOLERANCE:
            added_so_far = new_added
            break  # progress guard: bumps placed but ~zero length gained
        added_so_far = new_added

    if total_bumps == 0:
        print(f"    Warning: No straight run with clearance found for meanders")
        return segments, 0

    return current_segments, total_bumps


def _apply_meanders_to_net_with_iteration(
    result: dict,
    target_metric: float,
    current_metric: float,
    tolerance: float,
    config: GridRouteConfig,
    pcb_data: PCBData,
    already_processed_segments: List[Segment],
    already_processed_vias: List,
    group_clearance_index,
    metric_func,
    extra_length_func,
) -> Tuple[dict, List[Segment], int, float]:
    """
    Apply meanders to a single net with bump iteration and amplitude scaling.

    This is a shared helper for both length and time matching. The difference
    is controlled by the metric_func (calculates current value) and
    extra_length_func (converts metric deficit to physical length to add).

    Args:
        result: The routing result for this net
        target_metric: Target value to match (length in mm or time in ps)
        current_metric: Current value before meanders
        tolerance: Acceptable variance in metric units
        config: Routing configuration
        pcb_data: PCB data for clearance checking
        already_processed_segments: Segments from other nets for clearance
        already_processed_vias: Vias from other nets for clearance
        group_clearance_index: Spatial index for clearance checking
        metric_func: Function(segments, vias) -> current metric value
        extra_length_func: Function(metric_deficit) -> extra_length in mm

    Returns:
        Tuple of (modified_result_or_segments, final_segments, bump_count, final_metric)
    """
    from net_queries import calculate_route_length

    delta_metric = target_metric - current_metric
    extra_length = extra_length_func(delta_metric)

    is_diff_pair = result.get('is_diff_pair', False)

    if is_diff_pair:
        # Multipoint pair (#520): the pair is a chain of coupled 2-point legs
        # and its route_length is the LONGEST leg (the longest MST edge). That
        # leg is itself a complete 2-point coupled result, so the centerline
        # meander machinery below operates on the LEG; the splice at the end
        # folds the meandered copper back into the chain bookkeeping.
        mp_legs = result.get('leg_results') if result.get('is_multipoint') else None
        if mp_legs:
            span = max(mp_legs, key=lambda lr: pair_leg_metric(lr, metric_func))
        else:
            span = result

        # #520: measure ONE leg (max of P/N), not the sum of both legs, and
        # carry the entry offset (stub adders baked into current_metric that a
        # copper-only metric cannot see) forward so new_metric starts at
        # current_metric and the undershoot/scale loops act on commensurate
        # deltas against the single-leg target.
        entry_leg = pair_leg_metric(span, metric_func)
        stub_offset = current_metric - entry_leg

        def pair_metric(res):
            return pair_leg_metric(res, metric_func) + stub_offset

        # Differential pair: apply meanders to centerline
        modified_result, bump_count = apply_meanders_to_diff_pair(
            span, extra_length, config, pcb_data,
            extra_segments=already_processed_segments,
            extra_vias=already_processed_vias
        )
        new_metric = pair_metric(modified_result)

        # Step 2: If we undershot, add more bumps
        max_bump_iterations = 20
        iteration = 0
        while new_metric < target_metric - tolerance and iteration < max_bump_iterations:
            iteration += 1
            bump_count += 1
            metric_deficit = target_metric - new_metric
            extra_length = extra_length_func(metric_deficit)

            modified_result, actual_bumps = apply_meanders_to_diff_pair(
                span, extra_length, config, pcb_data,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count
            )
            prev_metric = new_metric
            new_metric = pair_metric(modified_result)

            if actual_bumps < bump_count or new_metric <= prev_metric:
                print(f"      Can't fit more bumps (got {actual_bumps}, need {bump_count})")
                bump_count = actual_bumps
                break

        # Step 3: Scale amplitude to hit target
        best_result = modified_result.copy()
        best_metric = new_metric
        scaled_amplitude = config.meander_amplitude
        for scale_iter in range(5):
            if abs(new_metric - target_metric) <= tolerance:
                break
            actual_added = new_metric - current_metric
            if actual_added <= 0 or bump_count == 0:
                break

            scale = delta_metric / actual_added
            scaled_amplitude = scaled_amplitude * scale
            if scaled_amplitude < 0.2:
                scaled_amplitude = 0.2

            trial_result, trial_bumps = apply_meanders_to_diff_pair(
                span, extra_length, config, pcb_data,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count,
                amplitude_override=scaled_amplitude
            )
            trial_metric = pair_metric(trial_result)

            # Keep the CLOSEST result to the target, not the largest (#520):
            # bump-quantized overshoot (a wide multipoint span can land many mm
            # over) must scale DOWN, which ">=" could never accept.
            if trial_bumps > 0 and abs(trial_metric - target_metric) < abs(best_metric - target_metric):
                modified_result = trial_result
                new_metric = trial_metric
                best_result = trial_result.copy()
                best_metric = trial_metric
            else:
                modified_result = best_result
                new_metric = best_metric
                break

        if mp_legs:
            # Splice the meandered span back into the chain. The write list
            # and rip-up custody carry the per-LEG dicts (#369 A2), so the
            # meandered copper must land IN the leg dict, not just in the
            # merged bookkeeping dict; the merged lists are then rebuilt from
            # the legs, preserving any extra copper merged carries beyond the
            # legs (relocation fanout segments). Old-identity sets are taken
            # BEFORE the leg update so the extras are computed against the
            # pre-meander lists.
            leg_seg_ids = {id(s) for lr in mp_legs for s in lr.get('new_segments', [])}
            leg_via_ids = {id(v) for lr in mp_legs for v in lr.get('new_vias', [])}
            extra_segs = [s for s in result.get('new_segments', []) if id(s) not in leg_seg_ids]
            extra_vs = [v for v in result.get('new_vias', []) if id(v) not in leg_via_ids]
            span.update(modified_result)
            merged_mod = {
                'new_segments': [s for lr in mp_legs for s in lr.get('new_segments', [])] + extra_segs,
                'new_vias': [v for lr in mp_legs for v in lr.get('new_vias', [])] + extra_vs,
                'route_length': new_metric,
            }
            return merged_mod, merged_mod['new_segments'], bump_count, new_metric

        return modified_result, modified_result.get('new_segments', []), bump_count, new_metric

    else:
        # Single-ended net
        net_id = None
        original_segments = result['new_segments']
        stub_length = result.get('stub_length', 0.0)
        if original_segments:
            net_id = original_segments[0].net_id

        new_segments, bump_count = apply_meanders_to_route(
            original_segments, extra_length, config,
            pcb_data=pcb_data, net_id=net_id,
            extra_segments=already_processed_segments,
            extra_vias=already_processed_vias,
            clearance_index=group_clearance_index
        )
        new_vias = result.get('new_vias', [])
        new_metric = metric_func(new_segments, new_vias)

        # Step 2: Add more bumps if undershoot
        max_bump_iterations = 20
        iteration = 0
        while new_metric < target_metric - tolerance and iteration < max_bump_iterations:
            iteration += 1
            bump_count += 1
            metric_deficit = target_metric - new_metric
            extra_length = extra_length_func(metric_deficit)

            new_segments, actual_bumps = apply_meanders_to_route(
                original_segments, extra_length, config,
                pcb_data=pcb_data, net_id=net_id,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count,
                clearance_index=group_clearance_index
            )
            prev_metric = new_metric
            new_metric = metric_func(new_segments, new_vias)

            if actual_bumps < bump_count or new_metric <= prev_metric:
                print(f"      Can't fit more bumps (got {actual_bumps}, need {bump_count})")
                bump_count = actual_bumps
                break

        # Step 3: Scale amplitude
        best_segments = new_segments
        best_metric = new_metric
        scaled_amplitude = config.meander_amplitude
        for scale_iter in range(5):
            if abs(new_metric - target_metric) <= tolerance:
                break
            actual_added = new_metric - current_metric
            if actual_added <= 0 or bump_count == 0:
                break

            scale = delta_metric / actual_added
            scaled_amplitude = scaled_amplitude * scale
            if scaled_amplitude < 0.2:
                scaled_amplitude = 0.2

            trial_segments, trial_bumps = apply_meanders_to_route(
                original_segments, extra_length, config,
                pcb_data=pcb_data, net_id=net_id,
                extra_segments=already_processed_segments,
                extra_vias=already_processed_vias,
                min_bumps=bump_count,
                amplitude_override=scaled_amplitude,
                clearance_index=group_clearance_index
            )
            trial_metric = metric_func(trial_segments, new_vias)

            # Keep the CLOSEST result to the target, not the largest (#520).
            if trial_bumps > 0 and abs(trial_metric - target_metric) < abs(best_metric - target_metric):
                new_segments = trial_segments
                new_metric = trial_metric
                best_segments = trial_segments
                best_metric = trial_metric
            else:
                new_segments = best_segments
                new_metric = best_metric
                break

        return new_segments, new_segments, bump_count, new_metric


def _seed_group_members_from_board(
    net_names: List[str],
    have_names,
    pcb_data: PCBData,
    in_run_names=(),
) -> Dict[str, dict]:
    """Pseudo-results for group members with NO in-run routing result, measured
    from the copper already on the board (#489 §7).

    Matching was same-run only: a group whose members the planner routed in
    DIFFERENT chain steps was skipped entirely with "fewer than 2 routed nets",
    even though the earlier step's copper is sitting in pcb_data. Seeding those
    members lets them set the group TARGET, which is the whole point -- this
    run's nets then meander up to the net routed earlier.

    The returned dicts are deliberately NOT inserted into net_results and carry
    'from_board': True so the meander loop skips them: their copper is already
    written, so "adding length" to them would emit a duplicate copy.

    `in_run_names` are names that DO have an in-run result, just not one this
    pass can lengthen -- a pair routed by the direct-hybrid escape carries no
    `route_length`, so it lands here despite having been routed seconds ago
    (#766). Such a member is seeded the same way but flagged `routed_this_run`,
    because telling the user to "route the whole group in one step" when they
    just did is advice that costs them a re-run to disprove.
    """
    from net_queries import calculate_route_length

    if pcb_data is None:
        return {}
    wanted = {}
    for nid, net in pcb_data.nets.items():
        if net.name and net.name in net_names and net.name not in have_names:
            wanted[nid] = net.name
    if not wanted:
        return {}

    segs_by: Dict[int, List[Segment]] = {}
    vias_by: Dict[int, List] = {}
    for s in pcb_data.segments:
        if s.net_id in wanted:
            segs_by.setdefault(s.net_id, []).append(s)
    for v in pcb_data.vias:
        if v.net_id in wanted:
            vias_by.setdefault(v.net_id, []).append(v)

    seeded = {}
    for nid, name in wanted.items():
        segs = segs_by.get(nid, [])
        if not segs:
            continue  # no copper on the board either -- genuinely unrouted
        vias = vias_by.get(nid, [])
        seeded[name] = {
            'new_segments': segs,
            'new_vias': vias,
            'route_length': calculate_route_length(segs, vias, pcb_data),
            'from_board': True,
            'routed_this_run': name in in_run_names,
        }
    return seeded


def apply_length_matching_to_group(
    net_results: Dict[str, dict],
    net_names: List[str],
    config: GridRouteConfig,
    pcb_data: PCBData = None,
    prev_group_segments: List[Segment] = None,
    prev_group_vias: List = None
) -> Dict[str, dict]:
    """
    Apply length matching to a group of nets.

    Handles both single-ended nets and differential pairs. For diff pairs,
    meanders are applied to the centerline and P/N paths are regenerated.

    Args:
        net_results: Dict mapping net name to routing result
        net_names: Net names in this matching group
        config: Routing configuration
        pcb_data: PCB data for clearance checking (optional)
        prev_group_segments: Segments from previously processed groups (for cross-group clearance)
        prev_group_vias: Vias from previously processed groups (for cross-group clearance)

    Returns:
        Modified net_results with meanders added
    """
    from net_queries import calculate_route_length

    # Find nets in this group that were successfully routed
    group_results = {}
    processed_result_ids = set()

    for name in net_names:
        if name in net_results:
            result = net_results[name]
            if result and not result.get('failed') and 'route_length' in result:
                result_id = id(result)
                if result_id not in processed_result_ids:
                    group_results[name] = result
                    processed_result_ids.add(result_id)

    # Members routed in an EARLIER chain step have no in-run result; measure
    # them from board copper so they can still set the target (#489 §7).
    # The have-set must cover every ALIAS of an in-run result (#520): a pair's
    # shared result dict is registered under BOTH member names, group_results
    # dedups to the first, and seeding on group_results keys then "measured"
    # the other member from the just-synced pcb_data -- claiming this run's
    # own copper as earlier-step copper and skewing the group bookkeeping.
    have_names = {name for name in net_names
                  if net_results.get(name)
                  and not net_results[name].get('failed')
                  and 'route_length' in net_results[name]}
    board_seeded = _seed_group_members_from_board(
        net_names, have_names, pcb_data,
        in_run_names={n for n in net_names if net_results.get(n)})
    group_results.update(board_seeded)

    if len(group_results) < 2:
        print(f"  Length matching group: fewer than 2 routed nets, skipping")
        return net_results
    if len(board_seeded) == len(group_results):
        # #766: "not meanderable by this pass" is not the same claim as "not
        # routed this run", and saying the latter sends the caller off to
        # re-run a step that just ran. A hybrid-escape pair carries no
        # centerline, so it lands here having been routed seconds ago.
        _tr = sorted(n for n, r in board_seeded.items() if r.get('routed_this_run'))
        if _tr:
            print(f"  Length matching group: nothing here can be meandered by this pass "
                  f"-- {len(_tr)} member(s) WERE routed this run but carry no centerline "
                  f"(hybrid-escape pair, #766): {', '.join(_tr)}. Re-running will not "
                  f"help; use --diff-pair-intra-match for P/N skew, or give the pair "
                  f"room to route coupled.")
        else:
            print(f"  Length matching group: no nets routed this run, skipping")
        return net_results
    if board_seeded:
        _tr = sorted(n for n, r in board_seeded.items() if r.get('routed_this_run'))
        _earlier = sorted(n for n in board_seeded if n not in _tr)
        if _earlier:
            print(f"  Length matching group: {len(_earlier)} member(s) measured "
                  f"from existing board copper (routed in an earlier step): "
                  f"{', '.join(_earlier)}")
        if _tr:
            print(f"  Length matching group: {len(_tr)} member(s) routed THIS run but "
                  f"not meanderable here (no centerline -- hybrid-escape pair, #766): "
                  f"{', '.join(_tr)}")

    # #521: a matched group's copper is an invariant later chain steps cannot
    # reproduce -- mark every member (including pair-member aliases and
    # board-seeded members) protected so downstream rip steps skip them.
    from protected_nets import note_protection_candidates
    note_protection_candidates({n: 'length-matched'
                                for n in (set(group_results) | have_names)})

    # Identify diff pairs vs single-ended
    diff_pair_count = sum(1 for r in group_results.values() if r.get('is_diff_pair'))
    single_count = len(group_results) - diff_pair_count

    # Find target length
    target_length = max(r['route_length'] for r in group_results.values())
    print(f"  Length matching group: {len(group_results)} nets ({diff_pair_count} diff pairs, {single_count} single-ended), target={target_length:.2f}mm")

    # Pre-collect segments/vias for clearance checking
    already_processed_segments: List[Segment] = list(prev_group_segments) if prev_group_segments else []
    already_processed_vias: List = list(prev_group_vias) if prev_group_vias else []

    for result in group_results.values():
        # Board-seeded members are already in pcb_data (the index reads it), so
        # adding their copper here would only duplicate it.
        if result.get('from_board'):
            continue
        if result.get('new_segments'):
            already_processed_segments.extend(result['new_segments'])
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])

    # Build spatial index
    group_clearance_index = None
    index_margin = 0
    if pcb_data is not None and config is not None:
        group_clearance_index = ClearanceIndex()
        group_clearance_index.build(pcb_data, config, already_processed_segments, already_processed_vias)
        index_margin = config.track_width + config.clearance + config.meander_amplitude

    # Define metric functions for length matching
    def length_metric(segments, vias):
        return calculate_route_length(segments, vias, pcb_data)

    def length_to_length(delta):
        return delta  # Identity for length matching

    # Apply meanders to shorter routes
    for net_name, result in group_results.items():
        current_length = result['route_length']
        delta = target_length - current_length

        if result.get('from_board'):
            # Copper already written in an earlier step: it sets the target, it
            # cannot be lengthened here.
            print(f"    {net_name}: {current_length:.2f}mm (existing board copper, not modified)")
            if delta > config.length_match_tolerance:
                if result.get('routed_this_run'):
                    print(f"    WARNING: {net_name} is {delta:.2f}mm SHORT of the group target "
                          f"and was ROUTED THIS RUN, but not in a form this pass can meander "
                          f"(no centerline -- e.g. a hybrid-escape pair, #766) -- the group is "
                          f"NOT fully matched. Re-running will not help; give the pair room to "
                          f"route coupled, or match it with --diff-pair-intra-match")
                else:
                    print(f"    WARNING: {net_name} is {delta:.2f}mm SHORT of the group target "
                          f"but its copper was written in an earlier step -- the group is NOT "
                          f"fully matched (route the whole group in one step to meander it)")
            continue

        if delta <= config.length_match_tolerance:
            suffix = " (diff pair)" if result.get('is_diff_pair') else ""
            print(f"    {net_name}{suffix}: {current_length:.2f}mm (OK, within tolerance)")
            continue

        suffix = " (diff pair)" if result.get('is_diff_pair') else ""
        print(f"    {net_name}{suffix}: {current_length:.2f}mm -> adding {delta:.2f}mm")

        modified, new_segments, bump_count, new_length = _apply_meanders_to_net_with_iteration(
            result, target_length, current_length, config.length_match_tolerance,
            config, pcb_data, already_processed_segments, already_processed_vias,
            group_clearance_index, length_metric, length_to_length
        )

        print(f"      {net_name}: new length = {new_length:.2f}mm ({bump_count} bumps)")

        if result.get('is_diff_pair'):
            result.update(modified)
        else:
            result['new_segments'] = new_segments
            result['route_length'] = new_length

        already_processed_segments.extend(new_segments)
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])

        if group_clearance_index is not None:
            group_clearance_index.add_segments(new_segments, index_margin)

    return net_results


def get_route_primary_layer(segments: List[Segment]) -> str:
    """
    Determine the primary layer for a route (where most length is).

    Used by time matching to determine which layer's ps/mm to use
    when calculating extra length needed to add a target amount of time.

    Args:
        segments: List of Segment objects

    Returns:
        Layer name where most of the route length is (defaults to 'F.Cu')
    """
    layer_lengths: Dict[str, float] = {}
    for seg in segments:
        length = segment_length(seg)
        layer_lengths[seg.layer] = layer_lengths.get(seg.layer, 0.0) + length

    if not layer_lengths:
        return 'F.Cu'

    return max(layer_lengths, key=layer_lengths.get)


def apply_time_matching_to_group(
    net_results: Dict[str, dict],
    net_names: List[str],
    config: GridRouteConfig,
    pcb_data: PCBData = None,
    prev_group_segments: List[Segment] = None,
    prev_group_vias: List = None
) -> Dict[str, dict]:
    """
    Apply time matching to a group of nets.

    Similar to apply_length_matching_to_group but matches propagation time
    instead of physical length. This accounts for different signal speeds
    on different layers due to varying effective dielectric constants.

    Args:
        net_results: Dict mapping net name to routing result
        net_names: Net names in this matching group
        config: Routing configuration
        pcb_data: PCB data for time calculations and clearance checking
        prev_group_segments: Segments from previously processed groups
        prev_group_vias: Vias from previously processed groups

    Returns:
        Modified net_results with meanders added
    """
    from impedance import calculate_route_propagation_time_ps, get_layer_ps_per_mm
    from net_queries import calculate_route_length

    # Find nets in this group that were successfully routed
    group_results = {}
    processed_result_ids = set()

    for name in net_names:
        if name in net_results:
            result = net_results[name]
            if result and not result.get('failed') and 'route_length' in result:
                result_id = id(result)
                if result_id not in processed_result_ids:
                    group_results[name] = result
                    processed_result_ids.add(result_id)

    # Members routed in an EARLIER chain step: measured from board copper so
    # they can set the target (#489 §7). Never meandered -- already written.
    # Alias-aware have-set, same as the length path (#520).
    have_names = {name for name in net_names
                  if net_results.get(name)
                  and not net_results[name].get('failed')
                  and 'route_length' in net_results[name]}
    board_seeded = _seed_group_members_from_board(
        net_names, have_names, pcb_data,
        in_run_names={n for n in net_names if net_results.get(n)})
    group_results.update(board_seeded)

    if len(group_results) < 2:
        print(f"  Time matching group: fewer than 2 routed nets, skipping")
        return net_results
    if len(board_seeded) == len(group_results):
        print(f"  Time matching group: no nets routed this run, skipping")
        return net_results
    if board_seeded:
        print(f"  Time matching group: {len(board_seeded)} member(s) measured "
              f"from existing board copper (routed in an earlier step): "
              f"{', '.join(sorted(board_seeded))}")

    # #521: matched members are protected downstream (see the length path).
    from protected_nets import note_protection_candidates
    note_protection_candidates({n: 'time-matched'
                                for n in (set(group_results) | have_names)})

    # Identify diff pairs vs single-ended
    diff_pair_count = sum(1 for r in group_results.values() if r.get('is_diff_pair'))
    single_count = len(group_results) - diff_pair_count

    # Calculate propagation time for each net and find primary layers
    net_times: Dict[str, float] = {}
    net_primary_layers: Dict[str, str] = {}

    for net_name, result in group_results.items():
        segments = result.get('new_segments', [])
        # A pair result's segment list holds BOTH legs' copper; time the
        # limiting leg, not the sum (#520) -- and for a multipoint pair, the
        # longest coupled leg (the matching span). Single-ended results fall
        # through unchanged (no leg net ids).
        net_times[net_name] = multipoint_span_metric(
            result, lambda s, v: calculate_route_propagation_time_ps(s, v, pcb_data))
        net_primary_layers[net_name] = get_route_primary_layer(segments)

    target_time = max(net_times.values())
    print(f"  Time matching group: {len(group_results)} nets ({diff_pair_count} diff pairs, {single_count} single-ended), target={target_time:.2f}ps")

    # Pre-collect segments/vias for clearance checking
    already_processed_segments: List[Segment] = list(prev_group_segments) if prev_group_segments else []
    already_processed_vias: List = list(prev_group_vias) if prev_group_vias else []

    for result in group_results.values():
        # Board-seeded members already live in pcb_data (see the length path).
        if result.get('from_board'):
            continue
        if result.get('new_segments'):
            already_processed_segments.extend(result['new_segments'])
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])

    # Build spatial index
    group_clearance_index = None
    index_margin = 0
    if pcb_data is not None and config is not None:
        group_clearance_index = ClearanceIndex()
        group_clearance_index.build(pcb_data, config, already_processed_segments, already_processed_vias)
        index_margin = config.track_width + config.clearance + config.meander_amplitude

    # Apply meanders to shorter-time routes
    for net_name, result in group_results.items():
        current_time = net_times[net_name]
        delta_time = target_time - current_time

        if result.get('from_board'):
            print(f"    {net_name}: {current_time:.2f}ps (existing board copper, not modified)")
            if delta_time > config.time_match_tolerance:
                if result.get('routed_this_run'):
                    print(f"    WARNING: {net_name} is {delta_time:.2f}ps SHORT of the group target "
                          f"and was ROUTED THIS RUN, but not in a form this pass can meander "
                          f"(no centerline -- e.g. a hybrid-escape pair, #766) -- the group is "
                          f"NOT fully matched. Re-running will not help; give the pair room to "
                          f"route coupled, or match it with --diff-pair-intra-match")
                else:
                    print(f"    WARNING: {net_name} is {delta_time:.2f}ps SHORT of the group target "
                          f"but its copper was written in an earlier step -- the group is NOT "
                          f"fully matched (route the whole group in one step to meander it)")
            continue

        if delta_time <= config.time_match_tolerance:
            suffix = " (diff pair)" if result.get('is_diff_pair') else ""
            print(f"    {net_name}{suffix}: {current_time:.2f}ps (OK, within tolerance)")
            continue

        primary_layer = net_primary_layers[net_name]
        ps_per_mm = get_layer_ps_per_mm(pcb_data, primary_layer)
        extra_length = delta_time / ps_per_mm

        suffix = " (diff pair)" if result.get('is_diff_pair') else ""
        print(f"    {net_name}{suffix}: {current_time:.2f}ps -> adding {delta_time:.2f}ps ({extra_length:.3f}mm on {primary_layer})")

        # Create metric functions that capture pcb_data
        def time_metric(segments, vias):
            return calculate_route_propagation_time_ps(segments, vias, pcb_data)

        def time_to_length(delta_ps):
            return delta_ps / ps_per_mm

        modified, new_segments, bump_count, new_time = _apply_meanders_to_net_with_iteration(
            result, target_time, current_time, config.time_match_tolerance,
            config, pcb_data, already_processed_segments, already_processed_vias,
            group_clearance_index, time_metric, time_to_length
        )

        print(f"      {net_name}: new time = {new_time:.2f}ps ({bump_count} bumps)")

        if result.get('is_diff_pair'):
            result.update(modified)
        else:
            result['new_segments'] = new_segments
            stub_length = result.get('stub_length', 0.0)
            result['route_length'] = calculate_route_length(new_segments) + stub_length

        already_processed_segments.extend(new_segments)
        if result.get('new_vias'):
            already_processed_vias.extend(result['new_vias'])

        if group_clearance_index is not None:
            group_clearance_index.add_segments(new_segments, index_margin)

    return net_results


def match_net_pattern(net_name: str, pattern: str) -> bool:
    """
    Check if net name matches a pattern.

    Patterns support:
    - * : matches any characters
    - [0-9] : matches digit range

    Args:
        net_name: Actual net name
        pattern: Pattern with wildcards

    Returns:
        True if matches
    """
    # Convert pattern to regex
    # First, handle [0-9] style ranges
    regex_pattern = pattern

    # Escape special regex chars except * and []
    # Process backslash first to avoid re-escaping
    regex_pattern = regex_pattern.replace('\\', '\\\\')
    for char in '.^$+?{}|()':
        regex_pattern = regex_pattern.replace(char, '\\' + char)

    # Convert * to .*
    regex_pattern = regex_pattern.replace('*', '.*')

    # Anchor the pattern
    regex_pattern = '^' + regex_pattern + '$'

    try:
        return bool(re.match(regex_pattern, net_name))
    except re.error:
        # Fall back to simple fnmatch
        return fnmatch.fnmatch(net_name, pattern)


def find_nets_matching_patterns(all_net_names: List[str], patterns: List[str]) -> List[str]:
    """
    Find all net names that match any of the given patterns.

    Args:
        all_net_names: All available net names
        patterns: List of patterns to match

    Returns:
        List of matching net names
    """
    matching = []
    for net_name in all_net_names:
        for pattern in patterns:
            if match_net_pattern(net_name, pattern):
                matching.append(net_name)
                break
    return matching


def auto_group_ddr4_nets(net_names: List[str]) -> List[List[str]]:
    """
    Automatically group DDR4 nets by byte lane for length matching.

    Groups:
    - DQ0-7 + DQS0/DQS0_N (byte lane 0)
    - DQ8-15 + DQS1/DQS1_N (byte lane 1)
    - etc.
    - CA/CMD/ADDR as separate group

    Args:
        net_names: List of net names to group

    Returns:
        List of groups, each group is a list of net names
    """
    groups = {}
    ungrouped = []

    for net_name in net_names:
        # Try to identify DQ nets (e.g., DQ0, DQ15, DQ0_A, etc.)
        dq_match = re.search(r'DQ(\d+)', net_name, re.IGNORECASE)
        if dq_match:
            dq_num = int(dq_match.group(1))
            byte_lane = dq_num // 8
            group_key = f"byte_lane_{byte_lane}"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        # Try to identify DQS nets (e.g., DQS0, DQS0_N, DQSP0, DQSN0)
        dqs_match = re.search(r'DQS[PN]?(\d+)', net_name, re.IGNORECASE)
        if dqs_match:
            dqs_num = int(dqs_match.group(1))
            group_key = f"byte_lane_{dqs_num}"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        # Try to identify CA/CMD/ADDR nets
        if re.search(r'(CA\d+|CMD|ADDR|A\d+|BA\d+|BG\d+|CK|CS|ODT|CKE|RAS|CAS|WE)', net_name, re.IGNORECASE):
            group_key = "command_address"
            if group_key not in groups:
                groups[group_key] = []
            groups[group_key].append(net_name)
            continue

        ungrouped.append(net_name)

    # Convert to list of lists
    result = list(groups.values())

    # Log grouping
    print(f"  DDR4 auto-grouping found {len(groups)} groups:")
    for key, nets in groups.items():
        print(f"    {key}: {len(nets)} nets")
    if ungrouped:
        print(f"    ungrouped: {len(ungrouped)} nets")

    return result


# ============================================================================
# Differential Pair Length Matching
# ============================================================================

def find_straight_runs_in_path(path: List[Tuple[int, int, int]],
                                 coord,
                                 min_length: float = 1.0) -> List[Tuple[int, int, float]]:
    """
    Find all straight runs in a centerline path (grid coordinates).

    Args:
        path: List of (gx, gy, layer) grid coordinates
        coord: GridCoord converter for grid<->float
        min_length: Minimum run length to consider (mm)

    Returns:
        List of (start_idx, end_idx, length) tuples, sorted by length descending
    """
    if len(path) < 2:
        return []

    runs = []
    i = 0

    while i < len(path) - 1:
        run_start = i
        run_length = 0.0

        # Get initial direction
        x1, y1 = coord.to_float(path[i][0], path[i][1])
        x2, y2 = coord.to_float(path[i + 1][0], path[i + 1][1])
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.sqrt(dx * dx + dy * dy)

        if seg_len < MIN_SEGMENT_LENGTH:
            i += 1
            continue

        run_length += seg_len
        ux, uy = dx / seg_len, dy / seg_len

        # Extend run while segments are colinear and on same layer
        j = i + 1
        while j < len(path) - 1:
            # Check same layer
            if path[j][2] != path[run_start][2]:
                break

            # Get next segment direction
            x1, y1 = coord.to_float(path[j][0], path[j][1])
            x2, y2 = coord.to_float(path[j + 1][0], path[j + 1][1])
            dx = x2 - x1
            dy = y2 - y1
            next_len = math.sqrt(dx * dx + dy * dy)

            if next_len < MIN_SEGMENT_LENGTH:
                j += 1
                continue

            next_ux, next_uy = dx / next_len, dy / next_len

            # Check if colinear (dot product close to 1)
            dot = ux * next_ux + uy * next_uy
            if dot < COLINEAR_DOT_THRESHOLD:
                break

            run_length += next_len
            j += 1

        if run_length >= min_length:
            runs.append((run_start, j, run_length))

        i = j

    # Sort by length descending
    runs.sort(key=lambda x: x[2], reverse=True)
    return runs


def generate_centerline_meander(
    path: List[Tuple[int, int, int]],
    start_idx: int,
    end_idx: int,
    extra_length: float,
    amplitude: float,
    coord,
    config: GridRouteConfig,
    spacing_mm: float,
    pcb_data: PCBData = None,
    p_net_id: int = None,
    n_net_id: int = None,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0
) -> Tuple[List[Tuple[float, float, int]], int]:
    """
    Generate meanders in a centerline path (returns float coordinates).

    Key differences from generate_trombone_meander:
    - Works with (gx, gy, layer) path format
    - Clearance checking accounts for full diff pair width (2 * spacing_mm)
    - Returns modified centerline path in float coordinates

    Args:
        path: Original centerline path in grid coords
        start_idx, end_idx: Indices of the straight run to meander
        extra_length: Target extra length to add (mm)
        amplitude: Maximum amplitude for meanders
        coord: GridCoord converter
        config: Routing configuration
        spacing_mm: P/N offset from centerline (for clearance calculation)
        pcb_data: PCB data for clearance checking
        p_net_id, n_net_id: Net IDs to exclude from clearance check
        extra_segments: Additional segments to check against
        extra_vias: Additional vias to check against
        min_bumps: Minimum number of bumps to generate

    Returns:
        (new_path_float, bump_count) - new centerline path in float coords with meanders
    """
    if extra_length <= 0 and min_bumps <= 0:
        # Just convert to float and return
        return [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                for p in path], 0

    # Convert path to float coordinates
    float_path = [(coord.to_float(p[0], p[1])[0], coord.to_float(p[0], p[1])[1], p[2])
                  for p in path]

    # Get start and end points of the straight run
    start_pt = float_path[start_idx]
    end_pt = float_path[end_idx]
    layer = start_pt[2]

    # Calculate direction
    dx = end_pt[0] - start_pt[0]
    dy = end_pt[1] - start_pt[1]
    seg_len = math.sqrt(dx * dx + dy * dy)

    if seg_len < MIN_SEGMENT_LENGTH:
        return float_path, 0

    ux = dx / seg_len
    uy = dy / seg_len
    px = -uy  # Perpendicular
    py = ux

    # Meander parameters - chamfer must be larger than P/N half-spacing to avoid crossings
    chamfer = max(0.1, spacing_mm * config.diff_chamfer_extra)
    min_amplitude = MIN_AMPLITUDE
    bump_width = 6 * chamfer  # wide entry (2) + wide top chamfers (4)

    # Account for full diff pair width in clearance
    # The meander bump needs clearance for both P and N tracks. Use the pair's
    # actual routed width (impedance/power), not the base track_width (#175).
    # (Informational only; the real per-bump keep-out is computed in
    # get_safe_amplitude_for_diff_pair.)
    layer_names = config.layers if isinstance(getattr(config, 'layers', None), list) else []
    layer_name = layer_names[layer] if isinstance(layer, int) and layer < len(layer_names) else str(layer)
    pair_net_w = max(config.track_width,
                     config.get_net_track_width(p_net_id, layer_name) if p_net_id is not None else 0.0,
                     config.get_net_track_width(n_net_id, layer_name) if n_net_id is not None else 0.0)
    diff_pair_half_width = spacing_mm + pair_net_w / 2

    # Build new path with meanders
    new_path = list(float_path[:start_idx])

    # Current position
    cx, cy = start_pt[0], start_pt[1]

    # Always include start_pt when there's path before the straight run.
    # This ensures proper 45-degree transitions from the previous segment.
    # For layer changes, this also ensures create_parallel_path_from_float sees proper layer transition.
    if start_idx > 0:
        new_path.append((cx, cy, layer))

    # Meander generation
    direction = 1
    # Which perpendicular side (+1/-1) the meander baseline currently sits at,
    # or None when the cursor is on the original centerline (see #449: copper
    # emitted at the offset without a clearance check can graze the neighbor
    # that blocked the bumps).
    offset_dir = None
    bump_count = 0
    total_extra_added = 0.0

    # Leave margin at start and end
    margin = bump_width

    # Add lead-in
    if margin > POSITION_TOLERANCE:
        cx += ux * margin
        cy += uy * margin
        new_path.append((cx, cy, layer))

    def should_continue():
        if min_bumps > 0:
            return bump_count < min_bumps
        else:
            return total_extra_added < extra_length

    max_skips = 100  # Prevent infinite loop
    skip_count = 0

    while should_continue():
        # Check room for another bump
        dist_to_end = math.sqrt((end_pt[0] - cx)**2 + (end_pt[1] - cy)**2)
        if dist_to_end < bump_width + margin:
            break

        # Find safe amplitude at this position. A bump starting from the
        # centerline (first, or first after a skip returned there) needs an
        # entry chamfer; bumps chained from the ±chamfer baseline don't.
        bump_amplitude = amplitude
        is_first = (offset_dir is None)

        if pcb_data is not None:
            # Adjust clearance for diff pair width
            safe_amp = get_safe_amplitude_for_diff_pair(
                cx, cy, ux, uy, px, py, direction, amplitude, min_amplitude,
                layer, pcb_data, p_net_id, n_net_id, config, spacing_mm,
                extra_segments, extra_vias, is_first
            )
            if safe_amp < min_amplitude:
                # Try other direction
                safe_amp_other = get_safe_amplitude_for_diff_pair(
                    cx, cy, ux, uy, px, py, -direction, amplitude, min_amplitude,
                    layer, pcb_data, p_net_id, n_net_id, config, spacing_mm,
                    extra_segments, extra_vias, is_first
                )
                if safe_amp_other >= min_amplitude:
                    direction = -direction
                    safe_amp = safe_amp_other
                else:
                    # No room for a bump here. Chamfer back to the original
                    # centerline first when sitting at the ±chamfer baseline:
                    # the skip points below are emitted without any clearance
                    # check, and walking them at the offset lays the pair
                    # off the DRC-clean corridor, into the clearance band of
                    # the neighbor that blocked the bumps (#449).
                    if offset_dir is not None:
                        cx += ux * 2 * chamfer - px * chamfer * offset_dir
                        cy += uy * 2 * chamfer - py * chamfer * offset_dir
                        new_path.append((cx, cy, layer))
                        offset_dir = None
                        continue
                    # Skip forward along the centerline
                    skip_count += 1
                    if skip_count > max_skips:
                        break
                    cx += ux * 0.2
                    cy += uy * 0.2
                    new_path.append((cx, cy, layer))
                    continue

            bump_amplitude = min(amplitude, safe_amp)

        riser_height = bump_amplitude - 2 * chamfer
        if riser_height < 0.1:
            riser_height = 0.1
            bump_amplitude = riser_height + 2 * chamfer

        # Calculate extra length for this bump
        # All chamfers are wider (2:1 ratio) for P/N track spacing
        chamfer_diag_wide = chamfer * math.sqrt(5)  # wider chamfer (2:1)
        has_entry_chamfer = is_first

        if has_entry_chamfer:
            # wide entry chamfer + 2 wide top chamfers + risers
            bump_path_length = 3 * chamfer_diag_wide + 2 * riser_height
            this_bump_width = 2 * chamfer + 4 * chamfer  # entry (2) + top chamfers (4)
        else:
            # 2 wide top chamfers + risers
            bump_path_length = 2 * chamfer_diag_wide + 2 * riser_height
            this_bump_width = 4 * chamfer  # top chamfers only

        extra_this_bump = bump_path_length - this_bump_width

        # Generate bump points
        if has_entry_chamfer:
            offset_dir = direction  # baseline moves to ±chamfer off centerline
            # Entry chamfer - wider (2x forward) to give P/N tracks room
            cx += ux * 2 * chamfer + px * chamfer * direction
            cy += uy * 2 * chamfer + py * chamfer * direction
            new_path.append((cx, cy, layer))

        # Riser up
        cx += px * riser_height * direction
        cy += py * riser_height * direction
        new_path.append((cx, cy, layer))

        # Top chamfer 1 - wider (2x forward) for P/N track spacing
        cx += ux * 2 * chamfer + px * chamfer * direction
        cy += uy * 2 * chamfer + py * chamfer * direction
        new_path.append((cx, cy, layer))

        # Top chamfer 2 - wider (2x forward) for P/N track spacing
        cx += ux * 2 * chamfer - px * chamfer * direction
        cy += uy * 2 * chamfer - py * chamfer * direction
        new_path.append((cx, cy, layer))

        # Riser down
        cx -= px * riser_height * direction
        cy -= py * riser_height * direction
        new_path.append((cx, cy, layer))

        total_extra_added += extra_this_bump
        bump_count += 1
        direction *= -1

    # Exit chamfer - wider (2x forward) to match entry (skipped when a skip
    # already returned the cursor to the centerline)
    if bump_count > 0 and offset_dir is not None:
        cx += ux * 2 * chamfer - px * chamfer * offset_dir
        cy += uy * 2 * chamfer - py * chamfer * offset_dir
        new_path.append((cx, cy, layer))

    # Lead-out to end: use wider chamfer (2:1) to smoothly transition back to path
    if bump_count > 0:
        # Calculate perpendicular distance from current point to end_pt
        dx_to_end = end_pt[0] - cx
        dy_to_end = end_pt[1] - cy

        # Project onto perpendicular direction
        perp_dist = dx_to_end * px + dy_to_end * py

        # If we need to move perpendicular to reach end level, use wider chamfer
        if abs(perp_dist) > POSITION_TOLERANCE:
            # Move forward by 2x the perpendicular distance (2:1 ratio)
            chamfer_x = cx + ux * 2 * abs(perp_dist) + px * perp_dist
            chamfer_y = cy + uy * 2 * abs(perp_dist) + py * perp_dist
            new_path.append((chamfer_x, chamfer_y, layer))
            cx, cy = chamfer_x, chamfer_y

    new_path.append((end_pt[0], end_pt[1], layer))

    # Add remaining path points
    new_path.extend(float_path[end_idx + 1:])

    return new_path, bump_count


def get_safe_amplitude_for_diff_pair(
    cx: float, cy: float,
    ux: float, uy: float,
    px: float, py: float,
    direction: int,
    max_amplitude: float,
    min_amplitude: float,
    layer: int,
    pcb_data: PCBData,
    p_net_id: int,
    n_net_id: int,
    config: GridRouteConfig,
    spacing_mm: float,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    is_first_bump: bool = True
) -> float:
    """
    Find safe amplitude for a diff pair meander bump.

    Accounts for the full diff pair width when checking clearance.
    The centerline meander at amplitude A means P/N tracks extend to A + spacing_mm.
    """
    # Extra clearance for diff pair width
    # The P/N tracks are at ±spacing_mm from centerline
    # So the outer edge of the meander is at amplitude + spacing_mm + track_width/2
    diff_pair_extra = spacing_mm

    # Add margin for segment merging
    meander_clearance_margin = config.grid_step / 2

    # Get layer name for comparison
    # config.layers is a list of layer names
    layer_names = config.layers if hasattr(config, 'layers') and isinstance(config.layers, list) else []
    layer_name = layer_names[layer] if layer < len(layer_names) else str(layer)

    # The pair may be routed wider than the base track_width (impedance control /
    # power width). The meandered arms carry the pair's ACTUAL width, so drive the
    # own-width term of the keep-out from it (#175). max() keeps the common case
    # (net_w == track_width) byte-identical; only wider pairs grow their keep-out.
    # spacing_mm already reflects the routed P/N offset. The neighbor half-width
    # starts from the generic track_width/2 and is grown per-object to the
    # FOREIGN copper's real width / via size (#370 B1).
    net_w = max(config.track_width,
                config.get_net_track_width(p_net_id, layer_name),
                config.get_net_track_width(n_net_id, layer_name))
    net_half = net_w / 2
    # Add corner margin for track width bloat at 45-degree chamfered corners
    corner_margin = net_half * CORNER_BLOAT_FACTOR
    required_clearance = net_half + config.track_width / 2 + config.clearance + meander_clearance_margin + diff_pair_extra + corner_margin
    via_clearance = config.via_size / 2 + net_half + config.clearance + meander_clearance_margin + diff_pair_extra + corner_margin
    chamfer = CHAMFER_SIZE

    # Board-edge keep-out: bumps are generated at the pair CENTERLINE, but the P/N
    # copper extends spacing_mm + net_half beyond it, so that outer edge must stay
    # edge_clearance inside the board outline (mirrors the single-ended path).
    board_bounds = getattr(pcb_data.board_info, 'board_bounds', None) if pcb_data and pcb_data.board_info else None
    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    edge_margin = edge_clearance + diff_pair_extra + net_half
    edge_geom = _board_edge_geom(pcb_data)

    # Combine all segments and vias
    all_segments = list(pcb_data.segments)
    if extra_segments:
        all_segments.extend(extra_segments)

    all_vias = list(pcb_data.vias)
    if extra_vias:
        all_vias.extend(extra_vias)

    # Binary search for safe amplitude
    test_amplitudes = [max_amplitude]
    amp = max_amplitude
    while amp > min_amplitude:
        amp *= 0.7
        if amp >= min_amplitude:
            test_amplitudes.append(amp)
    test_amplitudes.append(min_amplitude)

    for test_amp in test_amplitudes:
        # Generate bump segments for this amplitude (at centerline level)
        bump_segs = get_bump_segments(cx, cy, ux, uy, px, py, direction, test_amp, chamfer, is_first_bump)

        conflict_found = False

        # Reject amplitudes whose P/N copper would print past the board edge or
        # over an interior cutout (real Edge.Cuts geometry; bbox fallback, #370 B1).
        if not _bump_edge_clear(bump_segs, edge_geom, board_bounds, edge_margin):
            continue  # try a smaller amplitude that stays inside the outline

        # Check each bump segment
        for bx1, by1, bx2, by2 in bump_segs:
            if conflict_found:
                break

            for other_seg in all_segments:
                # Skip own nets
                if other_seg.net_id == p_net_id or other_seg.net_id == n_net_id:
                    continue
                if other_seg.layer != layer_name:
                    continue

                # Foreign copper at its REAL width (#370 B1).
                check_clearance = required_clearance + max(
                    0.0, (other_seg.width - config.track_width) / 2.0)

                # Quick distance check
                seg_center_x = (other_seg.start_x + other_seg.end_x) / 2
                seg_center_y = (other_seg.start_y + other_seg.end_y) / 2
                seg_half_len = math.sqrt((other_seg.end_x - other_seg.start_x)**2 +
                                         (other_seg.end_y - other_seg.start_y)**2) / 2
                bump_center_x = (bx1 + bx2) / 2
                bump_center_y = (by1 + by2) / 2
                rough_dist = math.sqrt((bump_center_x - seg_center_x)**2 + (bump_center_y - seg_center_y)**2)

                if rough_dist > test_amp + seg_half_len + check_clearance + 1.0:
                    continue

                dist = segment_to_segment_distance(
                    bx1, by1, bx2, by2,
                    other_seg.start_x, other_seg.start_y, other_seg.end_x, other_seg.end_y
                )

                if dist < check_clearance:
                    conflict_found = True
                    break

        # Check vias
        if not conflict_found:
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break

                for via in all_vias:
                    if via.net_id == p_net_id or via.net_id == n_net_id:
                        continue

                    dist = point_to_segment_distance(via.x, via.y, bx1, by1, bx2, by2)

                    # The FOREIGN via's real size when larger (#370 B1).
                    if dist < via_clearance + max(0.0, ((getattr(via, 'size', 0) or 0)
                                                        - config.via_size) / 2.0):
                        conflict_found = True
                        break

        # Check bump segments against pads (on same layer)
        if not conflict_found:
            pad_clearance = net_half + config.clearance + corner_margin + diff_pair_extra
            for bx1, by1, bx2, by2 in bump_segs:
                if conflict_found:
                    break
                for pad_net_id, pad_list in pcb_data.pads_by_net.items():
                    if pad_net_id == p_net_id or pad_net_id == n_net_id:
                        continue
                    if conflict_found:
                        break
                    for pad in pad_list:
                        # Expand wildcard layers like "*.Cu" to actual routing layers
                        expanded_pad_layers = expand_pad_layers(pad.layers, config.layers)
                        if layer_name not in expanded_pad_layers:
                            continue
                        # Treat pad as circle with radius = max(size_x, size_y)/2
                        pad_radius = _pad_bounding_radius(pad)
                        dist = point_to_segment_distance(pad.global_x, pad.global_y, bx1, by1, bx2, by2)
                        if dist < pad_radius + pad_clearance:
                            conflict_found = True
                            break

        if not conflict_found:
            return test_amp

    return 0


def apply_meanders_to_diff_pair(
    result: dict,
    extra_length: float,
    config: GridRouteConfig,
    pcb_data: PCBData,
    extra_segments: List[Segment] = None,
    extra_vias: List = None,
    min_bumps: int = 0,
    amplitude_override: float = None
) -> Tuple[dict, int]:
    """
    Apply meanders to a differential pair by modifying the centerline and regenerating P/N paths.

    Args:
        result: Original diff pair routing result
        extra_length: Length to add to centerline (mm)
        config: Routing configuration
        pcb_data: PCB data for clearance checking
        extra_segments: Additional segments to check against
        extra_vias: Additional vias to check against
        min_bumps: Minimum number of bumps to generate
        amplitude_override: Override amplitude for scaling

    Returns:
        (modified_result, bump_count)
    """
    from routing_config import GridCoord

    # Extract data from result
    centerline_grid = result.get('centerline_path_grid')
    if not centerline_grid or len(centerline_grid) < 2:
        print(f"    Warning: No centerline path available for diff pair meanders")
        return result, 0

    # For routes with vias, we need to be careful - only meander sections that
    # don't affect via positions. Check if route is single-layer.
    first_layer = centerline_grid[0][2]
    is_single_layer = all(p[2] == first_layer for p in centerline_grid)

    p_sign = result.get('p_sign', 1)
    n_sign = result.get('n_sign', -1)
    spacing_mm = result.get('spacing_mm', (config.track_width + config.diff_pair_gap) / 2)
    start_stub_dir = result.get('start_stub_dir')
    end_stub_dir = result.get('end_stub_dir')
    p_net_id = result.get('p_net_id')
    n_net_id = result.get('n_net_id')
    layer_names = result.get('layer_names', [])
    p_start = result.get('p_start')
    n_start = result.get('n_start')
    p_end = result.get('p_end')
    n_end = result.get('n_end')
    src_stub_dir = result.get('src_stub_dir')
    tgt_stub_dir = result.get('tgt_stub_dir')

    coord = GridCoord(config.grid_step)
    amplitude = amplitude_override if amplitude_override is not None else config.meander_amplitude

    # Find straight runs in centerline
    min_length = amplitude * 2
    runs = find_straight_runs_in_path(centerline_grid, coord, min_length)

    if not runs:
        print(f"    Warning: No suitable straight run in diff pair centerline for meanders")
        return result, 0

    # Try each straight run
    for start_idx, end_idx, run_length in runs:
        if run_length < amplitude * 2:
            continue

        # For routes with vias, check if run is on a single layer
        if not is_single_layer:
            run_layer = centerline_grid[start_idx][2]
            # Make sure all points in run are on same layer
            if any(centerline_grid[i][2] != run_layer for i in range(start_idx, end_idx + 1)):
                continue

        # Generate meanders on centerline
        new_centerline_float, bump_count = generate_centerline_meander(
            centerline_grid, start_idx, end_idx,
            extra_length, amplitude, coord, config, spacing_mm,
            pcb_data, p_net_id, n_net_id, extra_segments, extra_vias, min_bumps
        )

        if bump_count == 0:
            continue

        from diff_pair_routing import create_parallel_path_from_float, _float_path_to_geometry, _calculate_parallel_extension, _process_via_positions

        # Unified path: always regenerate full P/N paths from modified centerline
        p_float_path = create_parallel_path_from_float(
            new_centerline_float, sign=p_sign, spacing_mm=spacing_mm,
            start_dir=start_stub_dir, end_dir=end_stub_dir
        )
        n_float_path = create_parallel_path_from_float(
            new_centerline_float, sign=n_sign, spacing_mm=spacing_mm,
            start_dir=start_stub_dir, end_dir=end_stub_dir
        )

        # For multi-layer routes, process via positions to maintain P/N parallelism
        if not is_single_layer:
            simplified_path_grid = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
                                    for x, y, layer in new_centerline_float]
            p_float_path, n_float_path = _process_via_positions(
                simplified_path_grid, p_float_path, n_float_path, coord, config,
                p_sign, n_sign, spacing_mm
            )

        # Recalculate extensions for source and target connectors
        p_src_route = p_float_path[0][:2] if p_float_path else p_start
        n_src_route = n_float_path[0][:2] if n_float_path else n_start
        src_p_ext, src_n_ext = _calculate_parallel_extension(
            p_start, n_start, p_src_route, n_src_route,
            src_stub_dir, p_sign
        )

        p_tgt_route = p_float_path[-1][:2] if p_float_path else p_end
        n_tgt_route = n_float_path[-1][:2] if n_float_path else n_end
        tgt_p_ext, tgt_n_ext = _calculate_parallel_extension(
            p_end, n_end, p_tgt_route, n_tgt_route,
            tgt_stub_dir, p_sign
        )

        # Convert paths to segments with proper connector handling
        new_segments = []
        new_vias = []

        p_segs, p_vias_new, _ = _float_path_to_geometry(
            p_float_path, p_net_id, p_start, p_end, p_sign,
            src_stub_dir, tgt_stub_dir, src_p_ext, tgt_p_ext,
            config, layer_names
        )
        new_segments.extend(p_segs)
        new_vias.extend(p_vias_new)

        n_segs, n_vias_new, _ = _float_path_to_geometry(
            n_float_path, n_net_id, n_start, n_end, n_sign,
            src_stub_dir, tgt_stub_dir, src_n_ext, tgt_n_ext,
            config, layer_names
        )
        new_segments.extend(n_segs)
        new_vias.extend(n_vias_new)

        # Also update the grid version for storage
        new_centerline_grid = [(coord.to_grid(x, y)[0], coord.to_grid(x, y)[1], layer)
                               for x, y, layer in new_centerline_float]

        # Recreate GND vias for multi-layer routes
        if not is_single_layer and config.gnd_via_enabled:
            from diff_pair_routing import _create_gnd_vias
            gnd_net_id = result.get('gnd_net_id')
            gnd_via_dirs = result.get('gnd_via_dirs', [])
            gnd_vias = _create_gnd_vias(
                new_centerline_grid, coord, config, layer_names, spacing_mm, gnd_net_id, gnd_via_dirs
            )
            new_vias.extend(gnd_vias)

        # Calculate actual routed length from P segments (includes connectors and via barrels)
        # This is more accurate than centerline length which misses connector segments
        from net_queries import calculate_route_length
        p_routed_length = calculate_route_length(p_segs, p_vias_new, pcb_data)
        n_routed_length = calculate_route_length(n_segs, n_vias_new, pcb_data)
        avg_routed_length = (p_routed_length + n_routed_length) / 2

        # Get individual stub lengths (for accurate P/N total lengths)
        p_stub_length = result.get('p_stub_length', result.get('stub_length', 0.0))
        n_stub_length = result.get('n_stub_length', result.get('stub_length', 0.0))
        avg_stub_length = (p_stub_length + n_stub_length) / 2

        # Update result
        modified_result = dict(result)
        modified_result['new_segments'] = new_segments
        modified_result['new_vias'] = new_vias
        modified_result['p_routed_length'] = p_routed_length
        modified_result['n_routed_length'] = n_routed_length
        modified_result['centerline_length'] = avg_routed_length  # Use actual routed length
        # Use max(P,N) total for route_length when intra-match is enabled (since it will equalize)
        # Otherwise use average for consistent comparison
        if config.diff_pair_intra_match:
            p_total = p_routed_length + p_stub_length
            n_total = n_routed_length + n_stub_length
            modified_result['route_length'] = max(p_total, n_total)
        else:
            modified_result['route_length'] = avg_routed_length + avg_stub_length
        modified_result['simplified_path'] = new_centerline_float
        modified_result['centerline_path_grid'] = new_centerline_grid

        return modified_result, bump_count

    print(f"    Warning: No straight run with clearance found for diff pair meanders")
    return result, 0


def _lengthen_net_with_meanders(
    segments: List[Segment],
    net_id: int,
    paired_net_id: int,
    vias: List,
    stub_length: float,
    current_length: float,
    target_length: float,
    delta: float,
    config: GridRouteConfig,
    pcb_data: PCBData,
) -> Tuple[List[Segment], float, int]:
    """Add meanders to ONE net's ``segments`` to lengthen it toward
    ``target_length`` (total = routed + ``stub_length``), starting from
    ``current_length`` (the same total before meandering) with a positive
    ``delta`` gap to close.

    Shared by intra-pair P/N matching and end-to-end AC-coupled (XNet) matching
    so the bump-count + amplitude-scaling search lives in exactly one place.
    ``paired_net_id`` is the net's coupled partner on this run (reduced
    clearance). Returns ``(meandered_segments, new_total_length, bump_count)``;
    ``bump_count == 0`` means no meanders could be fit (segments/length returned
    unchanged) and the caller decides how to report it.
    """
    # Meander geometry: entry chamfer + 2 risers + 2 top chamfers + exit chamfer.
    # For n bumps: extra ~= n * (2*amplitude - 2.34*chamfer).
    # Match the generator's width-scaled chamfer (#501) or the bump-count /
    # initial-amplitude estimate here diverges from what actually gets emitted.
    chamfer = resolve_meander_chamfer(
        config,
        net_id,
        segments[0].layer if segments else None,
        max((s.width for s in segments), default=0.0),
    )
    min_amplitude = 0.1  # Minimum useful amplitude (smaller for small deltas)
    max_amplitude = config.meander_amplitude  # Default 1.0mm

    # Pick a bump count + initial amplitude that should hit the delta in one
    # shot, then refine by scaling below.
    max_extra_per_bump = 2 * max_amplitude - 2.34 * chamfer  # ~1.766mm at amplitude=1.0
    num_bumps_needed = max(1, int(math.ceil(delta / max_extra_per_bump)))
    initial_amplitude = (delta / num_bumps_needed + 2.34 * chamfer) / 2
    initial_amplitude = max(min_amplitude, min(initial_amplitude, max_amplitude))

    # Pass paired_net_id to use reduced clearance for the coupled partner track,
    # and min_bumps to get exactly the calculated number of bumps.
    meandered_segments, bump_count = apply_meanders_to_route(
        segments,
        delta,
        config,
        pcb_data=pcb_data,
        net_id=net_id,
        paired_net_id=paired_net_id,
        amplitude_override=initial_amplitude,
        min_bumps=num_bumps_needed
    )

    if bump_count == 0:
        return segments, current_length, 0

    new_routed = calculate_route_length(meandered_segments, vias, pcb_data)
    new_length = new_routed + stub_length

    # Keep track of best result (closest to target)
    best_segments = meandered_segments
    best_length = new_length
    best_bump_count = bump_count

    # Scale amplitude to converge on target (keep the closest result).
    for scale_iter in range(5):
        if abs(new_length - target_length) <= config.length_match_tolerance:
            break

        actual_extra = new_length - current_length
        if actual_extra < MIN_SEGMENT_LENGTH or bump_count == 0:
            break  # No meaningful extra was added

        needed_extra = target_length - current_length
        # amplitude that would give needed_extra with the current bump_count
        target_amplitude = (needed_extra / bump_count + 2.34 * chamfer) / 2
        if target_amplitude < 0.2:
            break  # Amplitude too small, accept current result

        new_meandered_segments, new_bump_count = apply_meanders_to_route(
            segments,
            delta,
            config,
            pcb_data=pcb_data,
            net_id=net_id,
            min_bumps=bump_count,  # Keep same bump count
            amplitude_override=target_amplitude,
            paired_net_id=paired_net_id
        )

        if new_bump_count == 0:
            break  # Scaling failed, use best result so far

        new_routed = calculate_route_length(new_meandered_segments, vias, pcb_data)
        new_length = new_routed + stub_length

        if abs(new_length - target_length) < abs(best_length - target_length):
            best_segments = new_meandered_segments
            best_length = new_length
            best_bump_count = new_bump_count
            bump_count = new_bump_count
        else:
            break  # Not improving, stop

    return best_segments, best_length, best_bump_count


def apply_intra_pair_length_matching(
    result: dict,
    config: GridRouteConfig,
    pcb_data: PCBData,
    p_net_id: int = None,
    n_net_id: int = None,
    status: dict = None
) -> dict:
    """
    Apply length matching within a differential pair by adding meanders to
    the shorter track (P or N) to match the longer one.

    Args:
        result: Diff pair routing result containing new_segments, new_vias, etc.
        config: Routing configuration with length_match_tolerance
        pcb_data: PCB data for clearance checking
        p_net_id/n_net_id: the pair's member net ids (#766). Default None reads
            them off the result, which is what the coupled-route constructors
            stamp -- but NOT every constructor does: the direct-hybrid escape
            (coupled middle + point-to-point terminal legs) returns a bare
            4-key result, and a pair routed that way was skipped in silence.
            The caller knows the pair, so it passes the ids.
        status: optional dict the function fills in place with
            {'status', 'reason', 'delta_mm', 'delta_before_mm'} -- the outcome
            as data, so the caller can report per pair instead of leaving the
            printed log as the only record.

    Returns:
        Modified result dict with adjusted segments
    """
    def _stat(st, reason=None, delta=None, before=None):
        if status is not None:
            status.update({'status': st, 'reason': reason,
                           'delta_mm': None if delta is None else round(delta, 4),
                           'delta_before_mm': None if before is None else round(before, 4)})

    if not result.get('new_segments'):
        _stat('unmatched', 'no-copper-in-result')
        return result

    # Net IDs: caller-supplied (authoritative) else the result's own stamps.
    if p_net_id is None:
        p_net_id = result.get('p_net_id')
    if n_net_id is None:
        n_net_id = result.get('n_net_id')
    if p_net_id is None or n_net_id is None:
        _stat('unmatched', 'no-pair-net-ids')
        return result

    # Separate P and N segments and vias
    all_segments = result['new_segments']
    all_vias = result.get('new_vias', [])

    p_segments = [s for s in all_segments if s.net_id == p_net_id]
    n_segments = [s for s in all_segments if s.net_id == n_net_id]
    p_vias = [v for v in all_vias if v.net_id == p_net_id]
    n_vias = [v for v in all_vias if v.net_id == n_net_id]

    # Calculate routed lengths (new segments only, without stubs)
    p_routed_length = calculate_route_length(p_segments, p_vias, pcb_data)
    n_routed_length = calculate_route_length(n_segments, n_vias, pcb_data)

    # Get pre-calculated stub lengths from routing phase
    p_src_stub = result.get('p_src_stub_length', 0.0)
    p_tgt_stub = result.get('p_tgt_stub_length', 0.0)
    n_src_stub = result.get('n_src_stub_length', 0.0)
    n_tgt_stub = result.get('n_tgt_stub_length', 0.0)

    polarity_fixed = result.get('polarity_fixed', False)

    # Calculate total stub lengths based on polarity swap status
    # If polarity will be swapped, target stubs swap between P and N:
    #   After swap: P gets P_source + N_target, N gets N_source + P_target
    if polarity_fixed:
        p_stub_length = p_src_stub + n_tgt_stub
        n_stub_length = n_src_stub + p_tgt_stub
        if config.verbose:
            print(f"      Polarity swap stub lengths:")
            print(f"        P: src={p_src_stub:.3f}mm + N_tgt={n_tgt_stub:.3f}mm = {p_stub_length:.3f}mm")
            print(f"        N: src={n_src_stub:.3f}mm + P_tgt={p_tgt_stub:.3f}mm = {n_stub_length:.3f}mm")
    else:
        p_stub_length = p_src_stub + p_tgt_stub
        n_stub_length = n_src_stub + n_tgt_stub

    p_length = p_routed_length + p_stub_length
    n_length = n_routed_length + n_stub_length

    # Debug: show length breakdown
    if config.verbose:
        from net_queries import calculate_via_barrel_length
        p_seg_only = math.fsum(segment_length(s) for s in p_segments)
        n_seg_only = math.fsum(segment_length(s) for s in n_segments)
        p_via_barrel = calculate_via_barrel_length(p_vias, pcb_data)
        n_via_barrel = calculate_via_barrel_length(n_vias, pcb_data)
        print(f"      P breakdown: {len(p_segments)} segs={p_seg_only:.3f}mm + {len(p_vias)} vias={p_via_barrel:.3f}mm + stub={p_stub_length:.3f}mm = {p_length:.3f}mm")
        print(f"      N breakdown: {len(n_segments)} segs={n_seg_only:.3f}mm + {len(n_vias)} vias={n_via_barrel:.3f}mm + stub={n_stub_length:.3f}mm = {n_length:.3f}mm")

        # Calculate pad escape distances (distance from pad center to nearest segment endpoint)
        # KiCad includes this in its track length measurement, we don't
        def calc_pad_escape(net_id, segments, stub_segs):
            all_segs = list(segments) + list(stub_segs)
            if not all_segs:
                return 0.0
            pads = pcb_data.pads_by_net.get(net_id, [])
            total_escape = 0.0
            for pad in pads:
                px, py = pad.global_x, pad.global_y
                # Find nearest segment endpoint to this pad
                min_dist = float('inf')
                for seg in all_segs:
                    d1 = math.sqrt((seg.start_x - px)**2 + (seg.start_y - py)**2)
                    d2 = math.sqrt((seg.end_x - px)**2 + (seg.end_y - py)**2)
                    min_dist = min(min_dist, d1, d2)
                if min_dist < float('inf'):
                    total_escape += min_dist
            return total_escape

        # Get stub segments from pcb_data
        p_stub_segs = [s for s in pcb_data.segments if s.net_id == p_net_id]
        n_stub_segs = [s for s in pcb_data.segments if s.net_id == n_net_id]
        p_escape = calc_pad_escape(p_net_id, p_segments, p_stub_segs)
        n_escape = calc_pad_escape(n_net_id, n_segments, n_stub_segs)
        print(f"      P pad escape: {p_escape:.3f}mm (KiCad adds this)")
        print(f"      N pad escape: {n_escape:.3f}mm (KiCad adds this)")
        print(f"      P total with escape: {p_length + p_escape:.3f}mm")
        print(f"      N total with escape: {n_length + n_escape:.3f}mm")

    delta = abs(p_length - n_length)
    if delta <= config.length_match_tolerance:
        print(f"    P/N intra-pair: already matched (delta={delta:.3f}mm <= {config.length_match_tolerance}mm)")
        _stat('within-tolerance', None, delta, delta)
        return result

    # Determine shorter track
    if p_length < n_length:
        shorter_segments = p_segments
        shorter_net_id = p_net_id
        shorter_vias = p_vias
        shorter_stub_length = p_stub_length  # Already post-swap if polarity_fixed
        longer_net_id = n_net_id
        shorter_label = "P"
    else:
        shorter_segments = n_segments
        shorter_net_id = n_net_id
        shorter_vias = n_vias
        shorter_stub_length = n_stub_length  # Already post-swap if polarity_fixed
        longer_net_id = p_net_id
        shorter_label = "N"

    # Target length for shorter track = length of longer track
    shorter_length = p_length if p_length < n_length else n_length
    target_length = p_length if p_length > n_length else n_length

    print(f"    P/N intra-pair: P={p_length:.3f}mm, N={n_length:.3f}mm, delta={delta:.3f}mm, adding meanders to {shorter_label}")

    # Lengthen the shorter track to match the longer one (shared meander
    # search — see _lengthen_net_with_meanders, also used by XNet matching).
    meandered_segments, new_shorter_length, bump_count = _lengthen_net_with_meanders(
        shorter_segments, shorter_net_id, longer_net_id, shorter_vias,
        shorter_stub_length, shorter_length, target_length, delta, config, pcb_data,
    )

    if bump_count == 0:
        print(f"    P/N intra-pair: could not fit meanders")
        _stat('unmatched', 'no-room-for-meanders', delta, delta)
        return result

    # Check if meanders actually improved delta (don't make things worse)
    new_delta = abs(new_shorter_length - target_length)
    if new_delta >= delta:
        # Meanders made delta worse or same, skip them
        print(f"    P/N intra-pair: meanders would increase delta ({new_delta:.3f}mm >= {delta:.3f}mm), skipping")
        _stat('unmatched', 'meanders-would-worsen', delta, delta)
        return result

    # Rebuild segment list with meandered shorter track
    other_segments = [s for s in all_segments if s.net_id != shorter_net_id]
    new_segments = other_segments + meandered_segments

    # Update result
    result['new_segments'] = new_segments
    if shorter_net_id == p_net_id:
        result['p_routed_length'] = new_shorter_length
    else:
        result['n_routed_length'] = new_shorter_length

    # Calculate new delta using fresh values (not stale result dict values from inter-pair)
    if shorter_net_id == p_net_id:
        new_delta = abs(new_shorter_length - n_length)
    else:
        new_delta = abs(p_length - new_shorter_length)
    print(f"    P/N intra-pair: {bump_count} bumps added, new {shorter_label}={new_shorter_length:.3f}mm, new delta={new_delta:.3f}mm")
    _stat('matched' if new_delta <= config.length_match_tolerance else 'improved',
          None if new_delta <= config.length_match_tolerance else 'meanders-ran-out',
          new_delta, delta)

    return result


def _board_pair_lengths(pcb_data, p_net_id, n_net_id):
    """P and N copper lengths measured off the BOARD, for a pair with no
    usable in-run pair result (#766). Returns (p_mm, n_mm) or None when either
    member has no copper at all (nothing to compare)."""
    if pcb_data is None:
        return None
    segs_p = [s for s in pcb_data.segments if s.net_id == p_net_id]
    segs_n = [s for s in pcb_data.segments if s.net_id == n_net_id]
    if not segs_p or not segs_n:
        return None
    vias_p = [v for v in pcb_data.vias if v.net_id == p_net_id]
    vias_n = [v for v in pcb_data.vias if v.net_id == n_net_id]
    return (calculate_route_length(segs_p, vias_p, pcb_data),
            calculate_route_length(segs_n, vias_n, pcb_data))


def run_intra_pair_matching(diff_pair_ids, routed_results, config, pcb_data,
                            skip_p_net_ids=()):
    """Intra-pair P/N length matching over EVERY pair in the run (#766).

    Driven by the PAIR LIST, not by the shape of the results dict. The old loop
    walked `routed_results` and required each result to carry `is_diff_pair` +
    `p_net_id` -- stamps that only SOME coupled-route constructors apply. A pair
    routed by the direct-hybrid escape (coupled middle + point-to-point terminal
    legs, `_route_direct_hybrid`) gets a bare 4-key result, so it was skipped
    without printing anything at all: the run reported it routed, and its P/N
    skew shipped unmeasured. On a 6-pair MIPI/CSI board the reporter saw exactly
    one pair matched at 0.18mm while four hybrid-routed pairs shipped at
    1.69-4.00mm against a 0.15mm tolerance.

    Every pair now produces a record, matched or not, and every unmatched pair
    names its reason. Returns a list of
    {'pair', 'p_net', 'n_net', 'status', 'reason', 'delta_mm',
     'delta_before_mm'} sorted by pair name; `status` is one of:

      matched          meanders brought the pair inside the tolerance
      improved         meanders reduced the skew but ran out short of tolerance
      within-tolerance already inside the tolerance, nothing to do
      unmatched        has pair copper, could not be meandered (see reason)
      not-matchable    no single in-run pair result to meander (see reason);
                       `delta_mm` is then MEASURED OFF THE BOARD when both
                       members have copper, so the caller still learns the skew
      skipped          deliberately handled elsewhere (see reason)
    """
    skip = set(skip_p_net_ids or ())
    reports = []
    for pair_name, pair in diff_pair_ids:
        rec = {'pair': pair_name, 'p_net': pair.p_net_name, 'n_net': pair.n_net_name,
               'status': None, 'reason': None,
               'delta_mm': None, 'delta_before_mm': None}
        if pair.p_net_id in skip:
            # AC-coupled (XNet) members are matched end-to-end by the #196 pass;
            # matching them per-side too would double-meander.
            rec.update(status='skipped', reason='ac-coupled-xnet')
            print(f"\n{pair_name}:")
            print("    P/N intra-pair: matched end-to-end by the AC-coupled pass (#196)")
            reports.append(rec)
            continue

        rr_p = routed_results.get(pair.p_net_id)
        rr_n = routed_results.get(pair.n_net_id)
        print(f"\n{pair_name}:")
        if rr_p is not None and rr_n is not None and rr_p is rr_n:
            status = {}
            apply_intra_pair_length_matching(
                rr_p, config, pcb_data,
                p_net_id=pair.p_net_id, n_net_id=pair.n_net_id, status=status)
            rec.update(status=status.get('status'), reason=status.get('reason'),
                       delta_mm=status.get('delta_mm'),
                       delta_before_mm=status.get('delta_before_mm'))
            reports.append(rec)
            continue

        # No single result holding BOTH legs' copper: the pair never routed
        # coupled (failed, deferred, or its members were routed one at a time by
        # the single-ended fallback). The meander machinery matches a pair's two
        # legs inside one result, so there is nothing to meander here -- but the
        # skew is still measurable, and shipping it unmeasured is the failure
        # this reports.
        if rr_p is None and rr_n is None:
            reason = 'pair-not-routed-this-run'
        else:
            reason = 'members-routed-separately'
        rec.update(status='not-matchable', reason=reason)
        board = _board_pair_lengths(pcb_data, pair.p_net_id, pair.n_net_id)
        if board is not None:
            delta = abs(board[0] - board[1])
            rec['delta_mm'] = round(delta, 4)
            rec['delta_before_mm'] = rec['delta_mm']
            verdict = ("within" if delta <= config.length_match_tolerance
                       else f"OVER the {config.length_match_tolerance}mm tolerance")
            print(f"    P/N intra-pair: NOT matched ({reason}) -- board copper "
                  f"P={board[0]:.3f}mm, N={board[1]:.3f}mm, delta={delta:.3f}mm, {verdict}")
        else:
            print(f"    P/N intra-pair: NOT matched ({reason}) -- no pair copper to measure")
        reports.append(rec)

    return sorted(reports, key=lambda r: r['pair'])


def _ac_coupled_member_lengths(result, p_net_id, n_net_id, pcb_data):
    """Per-member P and N conductor contributions for end-to-end matching.

    Mirrors apply_intra_pair_length_matching's measurement (routed length plus
    polarity-aware stub lengths) but returns BOTH sides plus the raw segment / via
    lists so the caller can lengthen one member net in place.
    """
    from net_queries import calculate_route_length
    segs = result.get('new_segments', [])
    vias = result.get('new_vias', [])
    p_segs = [s for s in segs if s.net_id == p_net_id]
    n_segs = [s for s in segs if s.net_id == n_net_id]
    p_vias = [v for v in vias if v.net_id == p_net_id]
    n_vias = [v for v in vias if v.net_id == n_net_id]
    p_routed = calculate_route_length(p_segs, p_vias, pcb_data)
    n_routed = calculate_route_length(n_segs, n_vias, pcb_data)

    # Polarity-aware stub totals (each member can be independently polarity-fixed,
    # matching apply_intra_pair_length_matching's post-swap accounting).
    p_src = result.get('p_src_stub_length', 0.0)
    p_tgt = result.get('p_tgt_stub_length', 0.0)
    n_src = result.get('n_src_stub_length', 0.0)
    n_tgt = result.get('n_tgt_stub_length', 0.0)
    if result.get('polarity_fixed', False):
        p_stub, n_stub = p_src + n_tgt, n_src + p_tgt
    else:
        p_stub, n_stub = p_src + p_tgt, n_src + n_tgt

    return {
        'result': result,
        'p_net_id': p_net_id, 'n_net_id': n_net_id,
        'p_segs': p_segs, 'n_segs': n_segs,
        'p_vias': p_vias, 'n_vias': n_vias,
        'p_stub': p_stub, 'n_stub': n_stub,
        'p_routed': p_routed, 'n_routed': n_routed,
        'p_len': p_routed + p_stub, 'n_len': n_routed + n_stub,
    }


def apply_ac_coupled_length_matching(xnet, routed_results, config, pcb_data):
    """End-to-end length-match one AC-coupled XNet (issue #196).

    Matches the concatenated P conductor (the sum of every member pair's P net)
    against the concatenated N conductor, then distributes the compensating
    meanders across the member segments: first onto members where the shorter
    conductor is *also* the locally-shorter net (so it reduces local intra-pair
    skew too), then spilling any remainder onto the member with the most routed
    copper (the most meander room). The meanders are added one MEMBER NET at a
    time (never a concatenated list -- each half's own copper would otherwise read
    as a foreign net at zero clearance). Length-only, mirroring intra-pair.

    Edits each member result's 'new_segments' in place. Reverts all member
    edits and returns the original delta when the end-to-end skew would not
    improve (mirroring intra-pair's guard, #460). Returns the final end-to-end
    skew in mm (or None if a member was not fully routed); degrades gracefully
    on congestion and never falsely claims a match it could not make.
    """
    tol = config.length_match_tolerance
    name = "+".join(xnet.base_names)

    members = []
    for m in xnet.members:
        res = routed_results.get(m.p_net_id)
        if res is None or routed_results.get(m.n_net_id) is None:
            print(f"    AC-couple {name}: member '{m.base_name}' not fully routed; "
                  f"skipping end-to-end match")
            return None
        md = _ac_coupled_member_lengths(res, m.p_net_id, m.n_net_id, pcb_data)
        md['base'] = m.base_name
        members.append(md)

    # math.fsum, not the builtin sum() (#493): Python 3.12 switched sum() to
    # compensated summation, so a float sum differs by an ULP or two between
    # KiCad's bundled python and the system one -- enough to flip a decision
    # made from it and make routing depend on the interpreter.
    total_p = math.fsum(md['p_len'] for md in members)
    total_n = math.fsum(md['n_len'] for md in members)
    delta = abs(total_p - total_n)
    bridges = ", ".join(xnet.bridge_refs)
    print(f"    AC-couple {name}: end-to-end P={total_p:.3f}mm, N={total_n:.3f}mm, "
          f"delta={delta:.3f}mm (coupled via {bridges})")
    if delta <= tol:
        print(f"    AC-couple {name}: already matched (delta={delta:.3f}mm <= {tol}mm)")
        return delta

    # Shorter conductor S receives the meanders; L is the longer (the target).
    S, L = ('p', 'n') if total_p < total_n else ('n', 'p')
    need = delta

    # Plan per-member additions:
    #  phase 1 - bring members where S is locally shorter up to LOCAL match (this
    #            also reduces their local intra-pair skew), capped at the total need;
    #  phase 2 - spill any remaining end-to-end deficit onto the member with the
    #            most routed S copper (proxy for the most meander room).
    plan = [0.0] * len(members)
    remaining = need
    by_headroom = sorted(
        range(len(members)),
        key=lambda i: members[i][f'{L}_len'] - members[i][f'{S}_len'],
        reverse=True,
    )
    for i in by_headroom:
        if remaining <= tol:
            break
        headroom = members[i][f'{L}_len'] - members[i][f'{S}_len']
        if headroom <= tol:
            continue
        a = min(remaining, headroom)
        plan[i] += a
        remaining -= a
    if remaining > tol:
        spill = max(range(len(members)), key=lambda i: members[i][f'{S}_routed'])
        plan[spill] += remaining
        remaining = 0.0

    # Apply: lengthen each planned member's S net once (never re-meander a net).
    added_total = 0.0
    snapshots = {}  # id(res) -> (res, pre-pass new_segments) for the #460 revert
    for i, want in enumerate(plan):
        if want <= tol:
            continue
        md = members[i]
        s_id, l_id = md[f'{S}_net_id'], md[f'{L}_net_id']
        cur = md[f'{S}_len']
        new_segs, new_len, bumps = _lengthen_net_with_meanders(
            md[f'{S}_segs'], s_id, l_id, md[f'{S}_vias'], md[f'{S}_stub'],
            cur, cur + want, want, config, pcb_data,
        )
        if bumps <= 0:
            print(f"      {md['base']}: could not fit meanders on {S.upper()} "
                  f"(wanted +{want:.3f}mm)")
            continue
        added = new_len - cur
        res = md['result']
        if id(res) not in snapshots:
            snapshots[id(res)] = (res, list(res['new_segments']))
        res['new_segments'] = [s for s in res['new_segments']
                               if s.net_id != s_id] + new_segs
        md[f'{S}_len'] = new_len
        added_total += added
        print(f"      {md['base']}: +{added:.3f}mm on {S.upper()} ({bumps} bumps)")

    new_total_s = (total_p if S == 'p' else total_n) + added_total
    new_total_l = total_n if S == 'p' else total_p
    new_delta = abs(new_total_l - new_total_s)
    if added_total > tol and new_delta >= delta:
        # Mirror intra-pair's guard: a matching pass never degrades the board.
        # End-to-end compare only -- the phase-2 spill member is deliberately
        # overshot locally, so a per-member guard would reject valid plans.
        for res_obj, orig_segs in snapshots.values():
            res_obj['new_segments'] = orig_segs
        print(f"    AC-couple {name}: meanders would increase end-to-end delta "
              f"({new_delta:.3f}mm >= {delta:.3f}mm), reverting")
        return delta
    if added_total <= tol:
        print(f"    AC-couple {name}: could not fit end-to-end meanders "
              f"(residual delta={delta:.3f}mm)")
    else:
        print(f"    AC-couple {name}: added {added_total:.3f}mm to {S.upper()}, "
              f"end-to-end delta {delta:.3f} -> {new_delta:.3f}mm")
    return new_delta

"""
Shared base utilities for PCB routing.

This module contains the core position and geometry utilities used by all routing modules.
The bulk of routing functionality has been split into:
- connectivity.py: Endpoint finding, stub analysis, MST algorithms
- net_queries.py: Pad/net queries, MPS ordering, route length calculations
- pcb_modification.py: Add/remove routes, segment cleanup
"""

import math
from typing import Dict, List, Tuple

import numpy as np

from kicad_parser import Segment, POSITION_DECIMALS


def build_layer_map(layers: List[str]) -> Dict[str, int]:
    """
    Build a mapping from layer names to indices.

    Args:
        layers: List of layer names (e.g., ['F.Cu', 'In1.Cu', 'B.Cu'])

    Returns:
        Dict mapping layer name to its index in the list
    """
    return {name: idx for idx, name in enumerate(layers)}


def pos_key(x: float, y: float) -> Tuple[float, float]:
    """
    Normalize coordinates for position-based lookups.

    Use this consistently when building position sets or checking position membership
    to avoid floating-point comparison issues.
    """
    return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS))


def segment_length(seg: Segment) -> float:
    """Calculate the length of a single segment."""
    return math.sqrt((seg.end_x - seg.start_x)**2 + (seg.end_y - seg.start_y)**2)


def dist_sq_to_rounded_rect(
    point_x: float, point_y: float,
    half_width: float, half_height: float,
    corner_radius: float = 0.0
) -> float:
    """
    Calculate squared distance from a point to a rounded rectangle centered at origin.

    The rectangle extends from (-half_width, -half_height) to (half_width, half_height).
    Corner radius creates rounded corners (0 for sharp corners).

    Args:
        point_x, point_y: Point coordinates (relative to rectangle center)
        half_width, half_height: Rectangle half-dimensions
        corner_radius: Radius of rounded corners (0 for sharp corners)

    Returns:
        Squared distance from point to rectangle edge (0 if inside)
    """
    abs_x, abs_y = abs(point_x), abs(point_y)

    if corner_radius > 0:
        # Inner rectangle bounds (where corners start)
        inner_half_w = half_width - corner_radius
        inner_half_h = half_height - corner_radius

        # Check if point is in a corner region
        if abs_x > inner_half_w and abs_y > inner_half_h:
            # Distance to corner arc center
            dx = abs_x - inner_half_w
            dy = abs_y - inner_half_h
            dist_to_corner_center = math.sqrt(dx * dx + dy * dy)
            # Distance to arc edge
            dist = dist_to_corner_center - corner_radius
            return dist * dist if dist > 0 else 0.0

    # Point is along a flat edge - rectangular distance
    closest_x = max(-half_width, min(point_x, half_width))
    closest_y = max(-half_height, min(point_y, half_height))
    dx = point_x - closest_x
    dy = point_y - closest_y
    return dx * dx + dy * dy


def iter_pad_blocked_cells(
    pad_gx: int, pad_gy: int,
    half_width: float, half_height: float,
    margin: float,
    grid_step: float,
    corner_radius: float = 0.0,
    corner_buffer: float = None,
    off_x: float = 0.0,
    off_y: float = 0.0,
    rotation_deg: float = 0.0
):
    """
    Generate grid cells that should be blocked for a pad.

    Yields (gx, gy) tuples for all cells within margin distance of the pad edge.
    Uses rectangular-with-rounded-corners shape matching the actual pad geometry.

    rotation_deg rotates the rectangle in the global frame (pad.rect_rotation)
    for diagonal pads; 0 (the common axis-aligned case) is unchanged.

    Args:
        pad_gx, pad_gy: Pad center in grid coordinates
        half_width, half_height: Pad half-dimensions in mm
        margin: Blocking margin in mm (e.g., track_width/2 + clearance)
        grid_step: Grid step size in mm
        corner_radius: Corner radius for roundrect pads (0 for rectangular)
        off_x, off_y: pad center's sub-cell offset in mm
            (pad.global_x - pad_gx*grid_step). Measures distance from the REAL
            pad center, not the quantized cell, so a track can't sit a sub-cell
            inside the clearance on the rounding side (issue #70). 0 = legacy.

    Yields:
        (gx, gy) tuples for each blocked cell
    """
    # Buffer for grid discretization in corner/diagonal regions: a track
    # through a cell could be up to ~grid_step/2 closer to the pad than the
    # cell center. This applies to ALL pad shapes since diagonal approaches
    # can occur with rectangular pads too. Callers whose geometry adds further
    # sub-grid deviation (diff pair P/N offsets) pass a larger buffer.
    if corner_buffer is None:
        corner_buffer = grid_step / 2
    rotated = abs(rotation_deg) > 1e-9 and abs(abs(rotation_deg) - 180.0) > 1e-9
    if rotated:
        _rrad = math.radians(rotation_deg)
        _rc, _rs = math.cos(_rrad), math.sin(_rrad)
        gext_x = abs(half_width * _rc) + abs(half_height * _rs)
        gext_y = abs(half_width * _rs) + abs(half_height * _rc)
    else:
        gext_x, gext_y = half_width, half_height
    # +1 cell so the bbox still covers the region once shifted by the sub-cell offset.
    expand_x = int(math.ceil((gext_x + margin + corner_buffer) / grid_step)) + 1
    expand_y = int(math.ceil((gext_y + margin + corner_buffer) / grid_step)) + 1
    margin_sq = margin * margin
    buffered_margin_sq = (margin + corner_buffer) * (margin + corner_buffer)

    # Inner rectangle bounds (where corners start)
    # For rectangular pads (corner_radius=0), define corner region with a threshold
    corner_threshold = grid_step / 2 if corner_radius == 0 else 0
    inner_half_w = half_width - corner_radius - corner_threshold
    inner_half_h = half_height - corner_radius - corner_threshold

    for ex in range(-expand_x, expand_x + 1):
        for ey in range(-expand_y, expand_y + 1):
            # Cell center relative to the REAL pad center (issue #70).
            cell_x = ex * grid_step - off_x
            cell_y = ey * grid_step - off_y
            if rotated:
                # Rotate global offset into the pad's local frame: R(-rotation).
                cell_x, cell_y = (cell_x * _rc + cell_y * _rs,
                                  -cell_x * _rs + cell_y * _rc)
            abs_x, abs_y = abs(cell_x), abs(cell_y)

            # Use buffered margin in corner/diagonal regions where grid discretization
            # can cause tracks to be closer than expected (applies to all pad shapes)
            in_corner = abs_x > inner_half_w and abs_y > inner_half_h
            effective_margin_sq = buffered_margin_sq if in_corner else margin_sq

            dist_sq = dist_sq_to_rounded_rect(cell_x, cell_y, half_width, half_height, corner_radius)
            if dist_sq < effective_margin_sq:
                yield (pad_gx + ex, pad_gy + ey)


def pad_blocked_cells_array(
    pad_gx: int, pad_gy: int,
    half_width: float, half_height: float,
    margin: float,
    grid_step: float,
    corner_radius: float = 0.0,
    corner_buffer: float = None,
    off_x: float = 0.0,
    off_y: float = 0.0,
    rotation_deg: float = 0.0,
):
    """Vectorized twin of iter_pad_blocked_cells: returns an (N, 2) int32
    array of blocked (gx, gy) cells.

    off_x/off_y are the pad center's sub-cell offset in mm
    (real_center - quantized_cell, i.e. pad.global_x - pad_gx*grid_step). When
    given, cell distances are measured from the REAL pad center rather than the
    quantized grid cell, so a track centerline cannot sit a sub-cell closer to
    the pad than the clearance on the side the pad rounds toward -- the residual
    sub-cell PAD-SEGMENT violations of issue #70. With off_x=off_y=0 the result
    is unchanged (bit-identical to iter_pad_blocked_cells).

    rotation_deg rotates the (half_width x half_height) rectangle in the global
    frame -- for diagonal pads (pad.rect_rotation). 0 (axis-aligned, the common
    case) takes the original bit-identical code path.
    """
    if corner_buffer is None:
        corner_buffer = grid_step / 2
    rotated = abs(rotation_deg) > 1e-9 and abs(abs(rotation_deg) - 180.0) > 1e-9
    if rotated:
        _rrad = math.radians(rotation_deg)
        _rc, _rs = math.cos(_rrad), math.sin(_rrad)
        # Global half-extent of the rotated rect, so the search bbox covers it.
        gext_x = abs(half_width * _rc) + abs(half_height * _rs)
        gext_y = abs(half_width * _rs) + abs(half_height * _rc)
    else:
        gext_x, gext_y = half_width, half_height
    # +1 cell so the bbox still covers the disk once shifted by the <=half-cell
    # sub-cell offset.
    expand_x = int(math.ceil((gext_x + margin + corner_buffer) / grid_step)) + 1
    expand_y = int(math.ceil((gext_y + margin + corner_buffer) / grid_step)) + 1
    margin_sq = margin * margin
    buffered_margin_sq = (margin + corner_buffer) * (margin + corner_buffer)

    corner_threshold = grid_step / 2 if corner_radius == 0 else 0
    inner_half_w = half_width - corner_radius - corner_threshold
    inner_half_h = half_height - corner_radius - corner_threshold

    ex = np.arange(-expand_x, expand_x + 1, dtype=np.int64)
    ey = np.arange(-expand_y, expand_y + 1, dtype=np.int64)
    # Match the generator's loop order: ex outer, ey inner
    exg, eyg = np.meshgrid(ex, ey, indexing="ij")
    # Cell center relative to the REAL pad center: a cell sits at (gx+ex)*step,
    # and the real pad center is pad_gx*step + off_x, so the offset along the
    # axis is ex*step - off_x.
    cell_x = exg.astype(np.float64) * grid_step - off_x
    cell_y = eyg.astype(np.float64) * grid_step - off_y
    if rotated:
        # Rotate global cell offsets into the pad's local frame: R(-rotation).
        lx = cell_x * _rc + cell_y * _rs
        ly = -cell_x * _rs + cell_y * _rc
        cell_x, cell_y = lx, ly
    abs_x = np.abs(cell_x)
    abs_y = np.abs(cell_y)

    in_corner = (abs_x > inner_half_w) & (abs_y > inner_half_h)
    effective_margin_sq = np.where(in_corner, buffered_margin_sq, margin_sq)

    # dist_sq_to_rounded_rect, vectorized with the same operations
    if corner_radius > 0:
        cr_inner_w = half_width - corner_radius
        cr_inner_h = half_height - corner_radius
        corner_region = (abs_x > cr_inner_w) & (abs_y > cr_inner_h)
        dxc = abs_x - cr_inner_w
        dyc = abs_y - cr_inner_h
        dist = np.sqrt(dxc * dxc + dyc * dyc) - corner_radius
        corner_dist_sq = np.where(dist > 0, dist * dist, 0.0)
    else:
        corner_region = np.zeros_like(abs_x, dtype=bool)
        corner_dist_sq = np.zeros_like(abs_x)

    closest_x = np.maximum(-half_width, np.minimum(cell_x, half_width))
    closest_y = np.maximum(-half_height, np.minimum(cell_y, half_height))
    dx = cell_x - closest_x
    dy = cell_y - closest_y
    rect_dist_sq = dx * dx + dy * dy

    dist_sq = np.where(corner_region, corner_dist_sq, rect_dist_sq)
    mask = dist_sq < effective_margin_sq

    cells = np.empty((int(mask.sum()), 2), dtype=np.int32)
    cells[:, 0] = (exg[mask] + pad_gx).astype(np.int32)
    cells[:, 1] = (eyg[mask] + pad_gy).astype(np.int32)
    return cells


# Offset-pattern caches for batched rasterization. The patterns are tiny
# (a few hundred cells) and reused for every segment/via on the board.
_SQUARE_OFFSETS_CACHE: Dict[int, "np.ndarray"] = {}
_CIRCLE_OFFSETS_CACHE: Dict[Tuple[int, float], "np.ndarray"] = {}


def square_offsets(expansion: int) -> "np.ndarray":
    """(K, 2) int32 offsets covering the full square [-e, e] x [-e, e],
    in the same (ex outer, ey inner) order as the legacy loops."""
    offs = _SQUARE_OFFSETS_CACHE.get(expansion)
    if offs is None:
        r = np.arange(-expansion, expansion + 1, dtype=np.int32)
        exg, eyg = np.meshgrid(r, r, indexing="ij")
        offs = np.column_stack([exg.ravel(), eyg.ravel()]).astype(np.int32)
        _SQUARE_OFFSETS_CACHE[expansion] = offs
    return offs


def circle_offsets(block_range: int, effective_sq: float) -> "np.ndarray":
    """(K, 2) int32 offsets with ex^2 + ey^2 <= effective_sq, matching the
    legacy loops' integer-vs-float comparison and iteration order."""
    key = (block_range, float(effective_sq))
    offs = _CIRCLE_OFFSETS_CACHE.get(key)
    if offs is None:
        r = np.arange(-block_range, block_range + 1, dtype=np.int32)
        exg, eyg = np.meshgrid(r, r, indexing="ij")
        mask = (exg.astype(np.int64) ** 2 + eyg.astype(np.int64) ** 2) <= effective_sq
        offs = np.column_stack([exg[mask], eyg[mask]]).astype(np.int32)
        _CIRCLE_OFFSETS_CACHE[key] = offs
    return offs

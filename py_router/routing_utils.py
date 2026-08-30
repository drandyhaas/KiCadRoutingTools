"""
Shared base utilities for PCB routing.

This module contains the core position and geometry utilities used by all routing modules.
The bulk of routing functionality has been split into:
- connectivity.py: Endpoint finding, stub analysis, MST algorithms
- net_queries.py: Pad/net queries, MPS ordering, route length calculations
- pcb_modification.py: Add/remove routes, segment cleanup
"""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

import numpy as np

import env_knobs
from collections import OrderedDict
from kicad_parser import Segment, POSITION_DECIMALS


# --- Rotation-aware pad-rectangle geometry -------------------------------------
# A pad's size_x/size_y are board-resolved (axis-aligned for orthogonal pads);
# any residual tilt is carried in pad.rect_rotation (0 for the common case, a
# value in (-90,90] for non-orthogonally-placed pads). These helpers test the
# pad's TRUE rectangle at any angle. All reduce to the plain axis-aligned test
# when rect_rotation == 0, so orthogonal pads are unaffected.

def _to_pad_frame(x: float, y: float, pad) -> Tuple[float, float]:
    """Offset of (x, y) from the pad centre, expressed in the pad's own frame."""
    dx = x - pad.global_x
    dy = y - pad.global_y
    rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
    if rr:
        rad = math.radians(rr)
        c, s = math.cos(rad), math.sin(rad)
        return dx * c + dy * s, -dx * s + dy * c
    return dx, dy


def point_in_pad_rect(x: float, y: float, pad, margin: float = 0.0) -> bool:
    """True if (x, y) lies within the pad rectangle expanded by `margin`."""
    lx, ly = _to_pad_frame(x, y, pad)
    return abs(lx) <= pad.size_x / 2 + margin and abs(ly) <= pad.size_y / 2 + margin


def point_to_pad_rect_dist(x: float, y: float, pad, margin: float = 0.0) -> float:
    """Distance from (x, y) to the pad rectangle (+margin); 0.0 if inside."""
    lx, ly = _to_pad_frame(x, y, pad)
    ox = max(abs(lx) - (pad.size_x / 2 + margin), 0.0)
    oy = max(abs(ly) - (pad.size_y / 2 + margin), 0.0)
    return math.hypot(ox, oy)


def into_pad_frame_point(x: float, y: float, pad) -> Tuple[float, float]:
    """Rotate a board point about the pad centre into the pad's axis-aligned
    frame, returned in absolute board coordinates. Lets a routine that tests an
    axis-aligned pad rectangle (e.g. segment_to_rect_distance) stay exact for a
    tilted pad: rotate the query geometry with this, keep the pad axis-aligned."""
    lx, ly = _to_pad_frame(x, y, pad)
    return pad.global_x + lx, pad.global_y + ly


def filter_cells_in_pad_rect(cells: "np.ndarray", grid_step: float, pad,
                             margin: float = 0.0) -> "np.ndarray":
    """Filter an (N, 2+) int grid-cell array to rows whose board position lies in
    the pad rectangle (+margin), honoring rect_rotation. No-op (returns the input)
    for axis-aligned pads, so orthogonal keepouts are unchanged."""
    rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
    if not rr or len(cells) == 0:
        return cells
    bx = cells[:, 0] * grid_step - pad.global_x
    by = cells[:, 1] * grid_step - pad.global_y
    rad = math.radians(rr)
    c, s = math.cos(rad), math.sin(rad)
    lx = bx * c + by * s
    ly = -bx * s + by * c
    mask = (np.abs(lx) <= pad.size_x / 2 + margin) & (np.abs(ly) <= pad.size_y / 2 + margin)
    return cells[mask]


def pad_rect_halfspan(pad, margin: float = 0.0) -> Tuple[float, float]:
    """Axis-aligned bounding-box half-extents (in x, y) of the pad rectangle
    rotated by rect_rotation, plus `margin`. Used to size a search/keepout box
    that fully contains the (possibly tilted) pad."""
    hx, hy = pad.size_x / 2, pad.size_y / 2
    rr = getattr(pad, 'rect_rotation', 0.0) or 0.0
    if rr:
        rad = math.radians(rr)
        c, s = abs(math.cos(rad)), abs(math.sin(rad))
        return hx * c + hy * s + margin, hx * s + hy * c + margin
    return hx + margin, hy + margin


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
    # DELIBERATELY NOT MEMOIZED -- do not paste the _PAD_OFFSETS_CACHE
    # fast-path from pad_blocked_cells_array in here. Two reasons:
    #   1. This is a GENERATOR. `return <array>` inside one is not a result,
    #      it is a bare StopIteration -- the fast path would yield NOTHING
    #      and the pad would silently get no keep-out at all. That shipped
    #      once (02f5375a, v0.20.4) and emptied this function on every warm
    #      cache; tests/test_pad_offset_keepout.py pins it.
    #   2. Even done correctly it would be wrong to share: this scalar
    #      rasterizer is the INDEPENDENT reference implementation that
    #      test_pad_offset_keepout.py grades the vectorized twin against.
    #      Serving it from the twin's cache makes that cross-check vacuous
    #      (and only when the cache happens to be warm, so it would pass or
    #      fail by call order). The perf case for the memo is entirely
    #      pad_blocked_cells_array's -- that is the twin every obstacle
    #      builder actually calls.
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


# Relative-offset memo for the pad rasterizer (see docstring below).
_PAD_OFFSETS_CACHE: Dict[Tuple, "np.ndarray"] = {}
_PAD_OFFSETS_ROWS = 0
_PAD_OFFSETS_ROW_CAP = 2_000_000


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

    Memoized (2026-08-14 orangecrab profiling: 2.04M calls / 84s, the pad
    twin of the capsule memo below): unlike the capsule, the ENTIRE
    computation here lives in the pad-relative frame -- pad_gx/pad_gy enter
    only as a final INTEGER shift -- so caching the relative offsets keyed
    by geometry alone and adding the position per call is bit-identical by
    construction (integer adds are exact; the #493 float-translation hazard
    does not apply). One cache entry serves every same-geometry pad on the
    board (a BGA field is one entry per sub-cell offset class).

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
    key = (half_width, half_height, margin, grid_step, corner_radius,
           corner_buffer, off_x, off_y, rotation_deg)
    offs = _PAD_OFFSETS_CACHE.get(key)
    if offs is not None:
        out = np.empty_like(offs)
        np.add(offs, np.array([[pad_gx, pad_gy]], dtype=np.int32), out=out)
        return out
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

    global _PAD_OFFSETS_ROWS
    offs = np.empty((int(mask.sum()), 2), dtype=np.int32)
    offs[:, 0] = exg[mask].astype(np.int32)
    offs[:, 1] = eyg[mask].astype(np.int32)
    offs.setflags(write=False)
    if _PAD_OFFSETS_ROWS + len(offs) > _PAD_OFFSETS_ROW_CAP:
        _PAD_OFFSETS_CACHE.clear()
        _PAD_OFFSETS_ROWS = 0
    _PAD_OFFSETS_CACHE[key] = offs
    _PAD_OFFSETS_ROWS += len(offs)
    cells = np.empty_like(offs)
    np.add(offs, np.array([[pad_gx, pad_gy]], dtype=np.int32), out=cells)
    return cells


# Exact-key memo for the capsule rasterizer. Profiling the orangecrab rescue
# tail (2026-08-14): the rescue/escalation ladders rebuild windowed obstacle
# maps per rung (826 full builds in one step), re-rasterizing the same ~13k
# segments ~800x each -- 10.8M calls / 332s of a 1228s profiled step, most
# of it per-call meshgrid allocation. Keys are the EXACT input floats:
# translation canonicalization is NOT bit-safe ((ax+k)*step float-rounds
# differently per cell -- the #493 one-ULP class), so a hit returns the
# byte-identical cell set by construction and cached-vs-computed behavior
# cannot diverge. Arrays are returned READ-ONLY and shared (consumers stamp
# or copy, never mutate -- audited: obstacle_map._add_segment_obstacle,
# obstacle_cache._collect_segment_obstacles, plane_obstacle_builder,
# blocking_analysis). Bounded by total cached rows; wholesale clear on
# overflow (the keyspace only churns via per-rung clearance margins).
_SEG_CAPSULE_CACHE: "OrderedDict[Tuple[float, float, float, float, float, float], np.ndarray]" = OrderedDict()
_SEG_CAPSULE_ROWS = 0

# Span twin of the capsule memo (#: 5.2x denser, so it holds a working set the
# cell form evicted). The two SHARE the raster budget: spans take the bulk
# because every stamping consumer uses them, and the cell form keeps a small
# slice for the few consumers that iterate cells.
_SEG_SPAN_CACHE: "OrderedDict[Tuple[float, float, float, float, float, float], np.ndarray]" = OrderedDict()
_SEG_SPAN_ROWS = 0


def _seg_span_row_budget() -> int:
    # 12 bytes per (gx, lo, hi) int32 row, 40% of the shared raster budget.
    return int(env_knobs.RASTER_CACHE_MB * 0.40 * 1e6 / 12)


def _seg_capsule_row_budget() -> int:
    # 8 bytes per (N,2) int32 row. Now 10% of the shared budget, not 50%:
    # the stamping consumers moved to segment_blocked_spans (40%), and what is
    # left here serves only the few consumers that iterate cells. LRU-evicted, never wholesale-cleared: profiling showed the old
    # 64MB clear-all cap cycling ~40x on orangecrab (69% hit rate where the
    # keyspace supports ~99%).
    return int(env_knobs.RASTER_CACHE_MB * 0.10 * 1e6 / 8)


_SQUARE_OFFSETS_CACHE: Dict[int, "np.ndarray"] = {}
_CIRCLE_OFFSETS_CACHE: Dict[Tuple[int, float], "np.ndarray"] = {}


def _capsule_mask(x1: float, y1: float, x2: float, y2: float,
                  margin: float, grid_step: float):
    """(xs, ys, mask) for the capsule around (x1,y1)-(x2,y2).

    THE predicate, in one place: both segment_blocked_cells_array and
    segment_blocked_spans derive from this, so the cell form and the span form
    can never disagree about membership.
    """
    inv = 1.0 / grid_step
    glo_x = int(math.floor((min(x1, x2) - margin) * inv))
    ghi_x = int(math.ceil((max(x1, x2) + margin) * inv))
    glo_y = int(math.floor((min(y1, y2) - margin) * inv))
    ghi_y = int(math.ceil((max(y1, y2) + margin) * inv))
    xs = np.arange(glo_x, ghi_x + 1, dtype=np.int32)
    ys = np.arange(glo_y, ghi_y + 1, dtype=np.int32)
    gxg, gyg = np.meshgrid(xs, ys, indexing="ij")
    cx = gxg.astype(np.float64) * grid_step
    cy = gyg.astype(np.float64) * grid_step
    dx, dy = x2 - x1, y2 - y1
    l2 = dx * dx + dy * dy
    if l2 <= 0.0:
        t = np.zeros_like(cx)
    else:
        t = ((cx - x1) * dx + (cy - y1) * dy) / l2
        np.clip(t, 0.0, 1.0, out=t)
    ddx = cx - (x1 + t * dx)
    ddy = cy - (y1 + t * dy)
    mask = (ddx * ddx + ddy * ddy) < margin * margin
    return xs, ys, gxg, gyg, mask


def segment_blocked_spans(x1: float, y1: float, x2: float, y2: float,
                          margin: float, grid_step: float) -> "np.ndarray":
    """(M, 3) int32 SPANS (gx, gy_lo, gy_hi -- both inclusive) covering exactly
    the cells segment_blocked_cells_array returns.

    Same membership, ~5.2x less memory. A capsule is CONVEX, so each column's
    cells are one contiguous run: storing (col, lo, hi) costs 12 bytes per
    column against 8 bytes x ~8.3 cells (measured over 400 representative
    capsules). That is what lets the memo hold a working set the cell form
    could not -- on glasgow the cell cache sat pinned at its 16M-row ceiling
    with 2,270,477 of its 2,317,497 misses being EVICTIONS, i.e. keys that
    would have hit in a cache that fit.

    Consumers stamp these through GridObstacleMap.add_blocked_cell_spans_batch
    / add_blocked_via_spans_batch, which expand in Rust. Expanding in Python
    instead measures 7.44 us against a 0.17 us cache hit (~65 s per route), so
    the span form only pays when the CONSUMER takes spans.

    Read-only and shared, exactly like the cell form.
    """
    global _SEG_SPAN_ROWS
    key = (x1, y1, x2, y2, margin, grid_step)
    cached = _SEG_SPAN_CACHE.get(key)
    if cached is not None:
        _SEG_SPAN_CACHE.move_to_end(key)
        return cached
    xs, ys, _gxg, _gyg, mask = _capsule_mask(x1, y1, x2, y2, margin, grid_step)
    any_col = mask.any(axis=1)
    sel = np.flatnonzero(any_col)
    if sel.size:
        lo = mask.argmax(axis=1)[sel]
        hi = mask.shape[1] - 1 - mask[:, ::-1].argmax(axis=1)[sel]
        out = np.empty((sel.size, 3), dtype=np.int32)
        out[:, 0] = xs[sel]
        out[:, 1] = ys[lo]
        out[:, 2] = ys[hi]
        # Convexity says each column is contiguous; verify rather than trust,
        # because a non-contiguous column would silently BLOCK EXTRA cells.
        if int((out[:, 2] - out[:, 1] + 1).sum()) != int(mask.sum()):
            raise AssertionError("capsule column not contiguous")
    else:
        out = np.empty((0, 3), dtype=np.int32)
    out.setflags(write=False)
    _SEG_SPAN_CACHE[key] = out
    _SEG_SPAN_ROWS += len(out)
    budget = _seg_span_row_budget()
    while _SEG_SPAN_ROWS > budget and _SEG_SPAN_CACHE:
        _, old_arr = _SEG_SPAN_CACHE.popitem(last=False)
        _SEG_SPAN_ROWS -= len(old_arr)
    return out


def segment_blocked_cells_array(x1: float, y1: float, x2: float, y2: float,
                                margin: float, grid_step: float) -> "np.ndarray":
    """(N, 2) int32 cells whose centre is within ``margin`` mm of the TRUE float
    segment (x1,y1)-(x2,y2) -- a capsule (fat line with rounded ends).

    Prefer ``segment_blocked_spans`` for anything that stamps into a
    GridObstacleMap; this cell form remains for consumers that iterate cells.

    Replaces the walk_line(rounded endpoints) + square_offsets box stamp, which
    rounded the endpoints to the grid and used a Chebyshev box: an off-grid track
    (terminal connection to an off-grid pad, ~31% of segments) had its keep-out
    shifted up to a half cell off its real centreline, and a diagonal track's box
    staircase under-covered the perpendicular direction between steps. A foreign
    track cleared the rounded stamp but grazed the real track (issue #70/B). Here
    distances are measured from the real segment, so off-grid + diagonal are exact.

    The result is memoized (exact float key) and READ-ONLY -- callers must not
    mutate it.
    """
    global _SEG_CAPSULE_ROWS
    key = (x1, y1, x2, y2, margin, grid_step)
    cached = _SEG_CAPSULE_CACHE.get(key)
    if cached is not None:
        _SEG_CAPSULE_CACHE.move_to_end(key)
        return cached
    _xs, _ys, gxg, gyg, mask = _capsule_mask(x1, y1, x2, y2, margin, grid_step)
    out = np.empty((int(mask.sum()), 2), dtype=np.int32)
    out[:, 0] = gxg[mask]
    out[:, 1] = gyg[mask]
    out.setflags(write=False)
    _SEG_CAPSULE_CACHE[key] = out
    _SEG_CAPSULE_ROWS += len(out)
    budget = _seg_capsule_row_budget()
    while _SEG_CAPSULE_ROWS > budget and _SEG_CAPSULE_CACHE:
        _, old_arr = _SEG_CAPSULE_CACHE.popitem(last=False)
        _SEG_CAPSULE_ROWS -= len(old_arr)
    return out

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

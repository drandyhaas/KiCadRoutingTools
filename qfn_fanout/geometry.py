"""
Geometry calculations for QFN/QFP fanout routing.

Functions for calculating fanout stub positions and angles.
"""
from __future__ import annotations

import math
from typing import Tuple

from qfn_fanout.types import QFNLayout, PadInfo


def calculate_fanout_stub(pad_info: PadInfo, layout: QFNLayout,
                          straight_length: float, max_diagonal_length: float,
                          grid_step: float = 0.0,
                          angle_ref_off: float = 0.0,
                          ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Calculate fanout stub with two segments: straight then an angled fan.

    Returns (corner_pos, end_pos):
    - corner_pos: where the straight segment ends and the angled fan begins
    - end_pos: final fanned-out endpoint

    Works entirely in GLOBAL board coordinates from the pad's escape vector
    (pad_info.escape_direction, the pad's outward long axis) and the along-edge
    tangent perpendicular to it, so it is correct for any board rotation or
    internal pad orientation. The straight segment escapes along the lead; the
    angled fan spreads tips toward the corners (length grows with distance from
    the edge midpoint).

    Fan angle increases with edge position (issue #200): a pad at the edge
    midpoint escapes nearly straight, and the angle ramps up to a full 45 deg
    at the outermost (corner) pad. A uniform 45 deg fan keeps every adjacent
    stub parallel, so neighbouring tips stay only pitch/sqrt(2) apart -- too
    tight on a fine-pitch edge to drop an escape via between them (the
    placement-side root cause behind #197 on watchy: USB_D+ boxed 0.225 mm from
    USB_DET). Ramping the angle makes adjacent stubs *diverge* instead of
    running parallel, which spreads the tips apart (USB_D+/USB_DET now ~0.40 mm)
    while the corner pad still reaches its full 45 deg spread.

    `angle_ref_off` is the largest |along-edge offset| among the pads on this
    side; the angle ramp is normalized by it so the OUTERMOST pad (not the
    notional bbox corner, which sits beyond the last pad) hits exactly 45 deg.
    When 0 (caller didn't supply it) the ramp falls back to edge_position. The
    fan *length* still scales with edge_position (bbox-relative) so corner stubs
    don't over-extend toward the adjacent edge.
    """
    pad = pad_info.pad
    pad_x, pad_y = pad.global_x, pad.global_y
    side = pad_info.side

    if side == 'center':
        return ((pad_x, pad_y), (pad_x, pad_y))

    # Escape vector (global, unit) and along-edge tangent perpendicular to it.
    esc_x, esc_y = pad_info.escape_direction
    tan_x, tan_y = -esc_y, esc_x

    # Corner point: end of straight segment (escape past the pad).
    corner_x = pad_x + esc_x * straight_length
    corner_y = pad_y + esc_y * straight_length

    # Signed offset along the edge from the package center (projection on tangent).
    off = (pad_x - layout.center_global_x) * tan_x + (pad_y - layout.center_global_y) * tan_y
    half_edge = (layout.width if side in ('top', 'bottom') else layout.height) / 2
    edge_position = min(abs(off) / half_edge, 1.0) if half_edge > 0 else 0.0

    # Diagonal length: 0 at center, max_diagonal_length at corners. A near-zero
    # fan collapses to the straight escape (end == corner).
    diagonal_length = edge_position * max_diagonal_length
    fan_dir = 1.0 if off >= 0 else -1.0
    if diagonal_length < 0.01:
        end_x, end_y = corner_x, corner_y
    else:
        # Escape (outward) component is the full diagonal; the along-edge
        # component is scaled by the angle ramp so the fan angle goes from ~0
        # at the edge midpoint to a full 45 deg at the outermost pad (issue
        # #200). Normalize by the side's outermost pad offset when available so
        # the corner pad reaches a true 45 deg, not the ~37 deg the bbox-half
        # normalization would give (the last pad is inset from the bbox corner).
        if angle_ref_off > 0:
            angle_pos = min(abs(off) / angle_ref_off, 1.0)
        else:
            angle_pos = edge_position
        diag_component = diagonal_length / math.sqrt(2)
        tan_component = diag_component * angle_pos
        end_x = corner_x + esc_x * diag_component + fan_dir * tan_x * tan_component
        end_y = corner_y + esc_y * diag_component + fan_dir * tan_y * tan_component

    # Land the stub end on the routing grid (issue #149) so the router gets an
    # on-grid terminal and a foreign track on the nearest cell can't graze this
    # end by a sub-cell amount. Grid anchored at the origin. (Center pads return
    # early above with no stub, so there is nothing off-grid to snap there.)
    if grid_step > 0:
        ideal_x, ideal_y = end_x, end_y
        end_x = round(ideal_x / grid_step) * grid_step
        end_y = round(ideal_y / grid_step) * grid_step

        # Direction-preserving snap (issue #200): near the edge midpoint the
        # along-edge displacement is much smaller than grid_step, so plain
        # rounding can pull the tip ACROSS the corner toward the neighbour --
        # reversing the bend (e.g. an inner stub leaning into its neighbour
        # instead of away). Keep the snapped tip on the fan_dir side of the
        # corner: if it crossed, pick the nearest of the four grid points
        # bracketing the ideal end whose along-edge offset from the corner is
        # still non-negative in fan_dir.
        if diagonal_length >= 0.01:
            ftx, fty = fan_dir * tan_x, fan_dir * tan_y
            def _toff(x, y):
                return (x - corner_x) * ftx + (y - corner_y) * fty
            if _toff(end_x, end_y) < -1e-9:
                gx0 = math.floor(ideal_x / grid_step)
                gy0 = math.floor(ideal_y / grid_step)
                best = None
                best_d = None
                for gx in (gx0, gx0 + 1):
                    for gy in (gy0, gy0 + 1):
                        cx, cy = gx * grid_step, gy * grid_step
                        if _toff(cx, cy) < -1e-9:
                            continue
                        d = (cx - ideal_x) ** 2 + (cy - ideal_y) ** 2
                        if best_d is None or d < best_d:
                            best_d, best = d, (cx, cy)
                if best is not None:
                    end_x, end_y = best

    return ((corner_x, corner_y), (end_x, end_y))

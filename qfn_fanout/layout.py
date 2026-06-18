"""
QFN/QFP layout analysis functions.

Analyzes footprint geometry to extract package parameters and pad information.
"""

import math
from typing import Optional, Tuple

from kicad_parser import Pad, Footprint, local_to_global
from qfn_fanout.types import QFNLayout, PadInfo


def analyze_qfn_layout(footprint: Footprint) -> Optional[QFNLayout]:
    """
    Analyze a footprint to extract QFN layout parameters.

    Works in the footprint's LOCAL frame (pad.local_x/local_y), where the pad
    rows are axis-aligned no matter how the part is rotated on the board. Using
    the global bbox instead misclassifies every pad on a non-orthogonally placed
    package (e.g. an LQFP144 at -135 deg forms a diamond, so almost no pad lands
    near a global bbox edge and the whole part fails to fan out).
    """
    pads = footprint.pads
    if len(pads) < 4:
        return None

    # Get bounding box from pad positions in the LOCAL (footprint) frame.
    x_positions = [p.local_x for p in pads]
    y_positions = [p.local_y for p in pads]
    min_x, max_x = min(x_positions), max(x_positions)
    min_y, max_y = min(y_positions), max(y_positions)

    width = max_x - min_x
    height = max_y - min_y

    # Find pads on each edge to determine pitch
    # Use a tolerance based on the package size
    edge_tolerance = min(width, height) * 0.1  # 10% of smaller dimension

    # Get pads on each edge (local frame)
    top_pads = [p for p in pads if abs(p.local_y - min_y) < edge_tolerance]
    right_pads = [p for p in pads if abs(p.local_x - max_x) < edge_tolerance]
    bottom_pads = [p for p in pads if abs(p.local_y - max_y) < edge_tolerance]
    left_pads = [p for p in pads if abs(p.local_x - min_x) < edge_tolerance]

    # Calculate pitch from the edge with most pads
    all_edge_pads = [(top_pads, 'x'), (bottom_pads, 'x'), (left_pads, 'y'), (right_pads, 'y')]
    pad_pitch = None

    for edge_pads, axis in all_edge_pads:
        if len(edge_pads) > 1:
            if axis == 'x':
                positions = sorted(set(p.local_x for p in edge_pads))
            else:
                positions = sorted(set(p.local_y for p in edge_pads))

            if len(positions) > 1:
                pitches = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
                # Use the minimum non-zero pitch
                min_pitch = min(p for p in pitches if p > 0.1)
                if pad_pitch is None or min_pitch < pad_pitch:
                    pad_pitch = min_pitch

    if pad_pitch is None:
        pad_pitch = 0.5  # Fallback

    center_local_x = (min_x + max_x) / 2
    center_local_y = (min_y + max_y) / 2
    center_gx, center_gy = local_to_global(footprint.x, footprint.y,
                                           footprint.rotation,
                                           center_local_x, center_local_y)

    return QFNLayout(
        center_x=center_local_x,
        center_y=center_local_y,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        pad_pitch=pad_pitch,
        edge_tolerance=edge_tolerance,
        fp_x=footprint.x,
        fp_y=footprint.y,
        fp_rotation=footprint.rotation,
        center_global_x=center_gx,
        center_global_y=center_gy,
    )


def _pad_long_axis_global(pad: Pad) -> Tuple[float, float]:
    """Unit vector along the pad's LONG axis in global board coordinates.

    size_x/size_y are board-space dims (the parser already resolved orthogonal
    rotation into them); rect_rotation carries any residual diagonal tilt.
    """
    base = (1.0, 0.0) if pad.size_x >= pad.size_y else (0.0, 1.0)
    if pad.rect_rotation:
        rr = math.radians(pad.rect_rotation)
        c, s = math.cos(rr), math.sin(rr)
        return (base[0] * c - base[1] * s, base[0] * s + base[1] * c)
    return base


def _pad_extent_along(pad: Pad, gx: float, gy: float) -> float:
    """Full extent (mm) of the pad rectangle along a GLOBAL unit direction."""
    if pad.rect_rotation:
        rr = math.radians(pad.rect_rotation)
        c, s = math.cos(rr), math.sin(rr)
        gx, gy = gx * c + gy * s, -gx * s + gy * c
    return pad.size_x * abs(gx) + pad.size_y * abs(gy)


def analyze_pad(pad: Pad, layout: QFNLayout) -> PadInfo:
    """
    Analyze a pad: classify its edge (local frame) and compute the GLOBAL escape
    direction along the pad's own long axis, pointed away from the package
    center. Using the pad's lead direction (rather than the bbox-edge normal)
    makes the escape correct for footprints whose pads are not perpendicular to
    their edge (e.g. parts authored at a 45 deg internal orientation); for the
    usual QFP it is identical to the edge normal.
    """
    x, y = pad.local_x, pad.local_y

    # Distance to each local edge (classification only)
    dist_top = abs(y - layout.min_y)
    dist_bottom = abs(y - layout.max_y)
    dist_left = abs(x - layout.min_x)
    dist_right = abs(x - layout.max_x)

    min_dist = min(dist_top, dist_bottom, dist_left, dist_right)
    tol = layout.edge_tolerance

    if min_dist == dist_top and dist_top < tol:
        side = 'top'
    elif min_dist == dist_bottom and dist_bottom < tol:
        side = 'bottom'
    elif min_dist == dist_left and dist_left < tol:
        side = 'left'
    elif min_dist == dist_right and dist_right < tol:
        side = 'right'
    else:
        return PadInfo(pad=pad, side='center', escape_direction=(0.0, 0.0),
                       pad_length=max(pad.size_x, pad.size_y),
                       pad_width=min(pad.size_x, pad.size_y))

    # Escape along the pad's long axis, signed to point away from package center.
    ax, ay = _pad_long_axis_global(pad)
    out_x = pad.global_x - layout.center_global_x
    out_y = pad.global_y - layout.center_global_y
    if ax * out_x + ay * out_y < 0:
        ax, ay = -ax, -ay

    pad_width = _pad_extent_along(pad, ax, ay)            # extent along escape
    pad_length = _pad_extent_along(pad, -ay, ax)          # extent along edge

    return PadInfo(
        pad=pad,
        side=side,
        escape_direction=(ax, ay),
        pad_length=pad_length,
        pad_width=pad_width
    )

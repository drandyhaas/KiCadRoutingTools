"""
QFN/QFP layout analysis functions.

Analyzes footprint geometry to extract package parameters and pad information.
"""

from typing import Optional

from kicad_parser import Pad, Footprint
from qfn_fanout.types import QFNLayout, PadInfo


def analyze_qfn_layout(footprint: Footprint) -> Optional[QFNLayout]:
    """
    Analyze a footprint to extract QFN layout parameters.
    Derives everything from actual pad positions and sizes.
    """
    pads = footprint.pads
    if len(pads) < 4:
        return None

    # Get bounding box from pad positions
    x_positions = [p.global_x for p in pads]
    y_positions = [p.global_y for p in pads]
    min_x, max_x = min(x_positions), max(x_positions)
    min_y, max_y = min(y_positions), max(y_positions)

    width = max_x - min_x
    height = max_y - min_y

    # Find pads on each edge to determine pitch
    # Use a tolerance based on the package size
    edge_tolerance = min(width, height) * 0.1  # 10% of smaller dimension

    # Get pads on top edge
    top_pads = [p for p in pads if abs(p.global_y - min_y) < edge_tolerance]
    right_pads = [p for p in pads if abs(p.global_x - max_x) < edge_tolerance]
    bottom_pads = [p for p in pads if abs(p.global_y - max_y) < edge_tolerance]
    left_pads = [p for p in pads if abs(p.global_x - min_x) < edge_tolerance]

    # Calculate pitch from the edge with most pads
    all_edge_pads = [(top_pads, 'x'), (bottom_pads, 'x'), (left_pads, 'y'), (right_pads, 'y')]
    pad_pitch = None

    for edge_pads, axis in all_edge_pads:
        if len(edge_pads) > 1:
            if axis == 'x':
                positions = sorted(set(p.global_x for p in edge_pads))
            else:
                positions = sorted(set(p.global_y for p in edge_pads))

            if len(positions) > 1:
                pitches = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
                # Use the minimum non-zero pitch
                min_pitch = min(p for p in pitches if p > 0.1)
                if pad_pitch is None or min_pitch < pad_pitch:
                    pad_pitch = min_pitch

    if pad_pitch is None:
        pad_pitch = 0.5  # Fallback

    return QFNLayout(
        center_x=(min_x + max_x) / 2,
        center_y=(min_y + max_y) / 2,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        pad_pitch=pad_pitch,
        edge_tolerance=edge_tolerance
    )


def analyze_pad(pad: Pad, layout: QFNLayout) -> PadInfo:
    """
    Analyze a pad to determine its side and escape direction.
    Uses pad position AND geometry to determine orientation.
    """
    x, y = pad.global_x, pad.global_y

    # Distance to each edge
    dist_top = abs(y - layout.min_y)
    dist_bottom = abs(y - layout.max_y)
    dist_left = abs(x - layout.min_x)
    dist_right = abs(x - layout.max_x)

    min_dist = min(dist_top, dist_bottom, dist_left, dist_right)
    tol = layout.edge_tolerance

    # Determine side based on position
    if min_dist == dist_top and dist_top < tol:
        side = 'top'
    elif min_dist == dist_bottom and dist_bottom < tol:
        side = 'bottom'
    elif min_dist == dist_left and dist_left < tol:
        side = 'left'
    elif min_dist == dist_right and dist_right < tol:
        side = 'right'
    else:
        side = 'center'

    # Determine escape direction and pad dimensions based on side
    # For QFN/QFP, pads are typically elongated perpendicular to the chip edge
    if side == 'top':
        escape_direction = (0.0, -1.0)  # Escape upward (negative Y)
        pad_length = pad.size_x  # Along the edge
        pad_width = pad.size_y   # Perpendicular to edge
    elif side == 'bottom':
        escape_direction = (0.0, 1.0)   # Escape downward
        pad_length = pad.size_x
        pad_width = pad.size_y
    elif side == 'left':
        escape_direction = (-1.0, 0.0)  # Escape leftward
        pad_length = pad.size_y  # Along the edge (vertical)
        pad_width = pad.size_x   # Perpendicular
    elif side == 'right':
        escape_direction = (1.0, 0.0)   # Escape rightward
        pad_length = pad.size_y
        pad_width = pad.size_x
    else:
        escape_direction = (0.0, 0.0)
        pad_length = max(pad.size_x, pad.size_y)
        pad_width = min(pad.size_x, pad.size_y)

    return PadInfo(
        pad=pad,
        side=side,
        escape_direction=escape_direction,
        pad_length=pad_length,
        pad_width=pad_width
    )

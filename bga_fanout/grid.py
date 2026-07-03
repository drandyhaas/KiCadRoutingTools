"""
BGA grid analysis functions.

Analyzes BGA footprint geometry to extract grid parameters and routing channels.
"""
from __future__ import annotations

from typing import List, Optional, Tuple
from collections import defaultdict

from kicad_parser import Footprint
from bga_fanout.types import BGAGrid, Channel
from bga_fanout.constants import EDGE_PAD_TOLERANCE


# Coordinate-noise tolerance for grid clustering. Rotated (±90°) footprints
# pick up ~1e-6mm FP noise in global_x/global_y from the rotation transform,
# splitting each real ball row/column into several near-duplicate positions;
# the dominant-pitch histogram then sees the noise gaps as the pitch and
# reports 0.00mm (issue #283, zynq_ad9364). Any real BGA pitch is >=0.2mm,
# so 1µm cleanly separates noise from geometry.
_COORD_TOL = 1e-3


def _cluster_positions(values, tol: float = _COORD_TOL) -> List[float]:
    """Collapse sorted coordinate values into cluster means, merging values
    closer than `tol` (FP noise on rotated footprints)."""
    ordered = sorted(values)
    clusters = []
    group = [ordered[0]]
    for v in ordered[1:]:
        if v - group[-1] <= tol:
            group.append(v)
        else:
            clusters.append(sum(group) / len(group))
            group = [v]
    clusters.append(sum(group) / len(group))
    return clusters


def analyze_bga_grid(footprint: Footprint) -> Optional[BGAGrid]:
    """Analyze a footprint to extract BGA grid parameters."""
    pads = footprint.pads
    if len(pads) < 4:
        return None

    x_positions = _cluster_positions(p.global_x for p in pads)
    y_positions = _cluster_positions(p.global_y for p in pads)

    if len(x_positions) < 2 or len(y_positions) < 2:
        return None

    x_diffs = [x_positions[i+1] - x_positions[i] for i in range(len(x_positions)-1)]
    y_diffs = [y_positions[i+1] - y_positions[i] for i in range(len(y_positions)-1)]

    def get_dominant_pitch(diffs):
        if not diffs:
            return None
        rounded = [round(d, 2) for d in diffs]
        counts = defaultdict(int)
        for d in rounded:
            counts[d] += 1
        if not counts:
            return None
        dominant = max(counts.keys(), key=lambda k: counts[k])
        if counts[dominant] < len(diffs) * 0.5:
            return None
        if dominant <= _COORD_TOL:
            # A zero pitch means the positions were noise-split, not a grid
            # (issue #283); it would divide-by-zero downstream.
            return None
        return dominant

    pitch_x = get_dominant_pitch(x_diffs)
    pitch_y = get_dominant_pitch(y_diffs)

    if pitch_x is None or pitch_y is None:
        return None

    return BGAGrid(
        pitch_x=pitch_x,
        pitch_y=pitch_y,
        rows=y_positions,
        cols=x_positions,
        center_x=(min(x_positions) + max(x_positions)) / 2,
        center_y=(min(y_positions) + max(y_positions)) / 2,
        min_x=min(x_positions) - pitch_x / 2,
        max_x=max(x_positions) + pitch_x / 2,
        min_y=min(y_positions) - pitch_y / 2,
        max_y=max(y_positions) + pitch_y / 2
    )


def calculate_channels(grid: BGAGrid) -> List[Channel]:
    """Calculate routing channels between ball rows and columns."""
    channels = []
    idx = 0

    for i in range(len(grid.rows) - 1):
        y_pos = (grid.rows[i] + grid.rows[i + 1]) / 2
        channels.append(Channel(orientation='horizontal', position=y_pos, index=idx))
        idx += 1

    for i in range(len(grid.cols) - 1):
        x_pos = (grid.cols[i] + grid.cols[i + 1]) / 2
        channels.append(Channel(orientation='vertical', position=x_pos, index=idx))
        idx += 1

    return channels


def is_edge_pad(pad_x: float, pad_y: float, grid: BGAGrid, tolerance: float = EDGE_PAD_TOLERANCE) -> Tuple[bool, str]:
    """
    Check if a pad is on the outer edge of the BGA.
    Returns (is_edge, escape_direction).

    Edge pads are on the outermost row or column and can escape directly
    without needing a 45° stub.
    """
    on_left = abs(pad_x - grid.cols[0]) < tolerance
    on_right = abs(pad_x - grid.cols[-1]) < tolerance
    on_top = abs(pad_y - grid.rows[0]) < tolerance
    on_bottom = abs(pad_y - grid.rows[-1]) < tolerance

    # Determine escape direction based on which edge
    if on_right:
        return True, 'right'
    elif on_left:
        return True, 'left'
    elif on_bottom:
        return True, 'down'
    elif on_top:
        return True, 'up'

    return False, ''

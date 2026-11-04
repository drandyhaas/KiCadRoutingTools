"""
BGA grid analysis functions.

Analyzes BGA footprint geometry to extract grid parameters and routing channels.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple
import math as _math
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


def _mean(group: List[float]) -> float:
    """Exact cluster mean -- math.fsum, never the builtin sum().

    Python 3.12 changed sum() to use Neumaier compensated summation, so the
    builtin returns a DIFFERENT (more accurate) float on 3.12+ than on 3.9:
    sum([0.65]*15) is 9.75 on 3.14 and 9.750000000000002 on 3.9. KiCad bundles
    its own python (3.9 here) while the CLI runs the system one, so the same
    board fanned out differently depending on which interpreter ran it -- the
    1-2 ULP difference lands in the cluster means, shifts the BGA grid bounds
    (min_x 116.125 vs 116.12500000000003), and flips the escape direction of any
    ball sitting on an exact boundary tie. On eth_tap's U13 that was one ball
    (up -> right), which cascaded into 109 vs 110 fanout tracks and a different
    board from there on. math.fsum is exactly rounded and identical on every
    version, so the geometry no longer depends on the interpreter (#493).
    """
    return math.fsum(group) / len(group)


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
            clusters.append(_mean(group))
            group = [v]
    clusters.append(_mean(group))
    return clusters


def analyze_bga_grid(footprint: Footprint) -> Optional[BGAGrid]:
    """Analyze a footprint to extract BGA grid parameters."""
    # Mechanical NPTH pads (mounting holes baked into the footprint) carry no
    # copper and are never balls, but their outlier coordinates split the
    # dominant-pitch vote -- granit_cm5's CM4 carrier has 4 corner mounting
    # holes that made a 200-pad 0.4mm-pitch connector read as "not a BGA"
    # (#513 item 3). Cluster only real copper pads; net-0 SMD balls stay (a
    # depopulated/unconnected ball is still a grid member).
    pads = [p for p in footprint.pads if p.pad_type != 'np_thru_hole']
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


def ball_row_gap(footprint: Footprint, grid: BGAGrid) -> float:
    """Copper-to-copper gap between two adjacent balls of the array, the
    narrower of the two axes: what a track laid down a channel on the
    BALLS' OWN layer must fit, with its clearance to the ball on either side
    (#1518). The balls' dominant size is used -- the one nearly every ball
    has -- so one odd pad does not speak for the whole array."""
    def dominant(values):
        counts = defaultdict(int)
        for v in values:
            counts[round(v, 4)] += 1
        return max(counts, key=lambda k: (counts[k], k)) if counts else 0.0
    balls = [p for p in footprint.pads if p.pad_type != 'np_thru_hole']
    size_x = dominant(p.size_x for p in balls)
    size_y = dominant(p.size_y for p in balls)
    return min(grid.pitch_x - size_x, grid.pitch_y - size_y)



def staggered_lattice_diagnosis(footprint, grid) -> Optional[str]:
    """Is this footprint a STAGGERED lattice rather than a ball grid? (#500)

    `analyze_bga_grid` derives the pitch from the gaps between distinct pad
    coordinates. On a staggered multi-row package (AQFN and friends) the two
    offset rows project onto each axis at HALF the real pad spacing, so the
    reported pitch is half the truth and every downstream budget is computed
    against it. osprey_kb's Nordic_AQFN-73 reports `pitch 0.25` while 70 of its
    78 pads have a 0.5mm nearest neighbour; the escape budget then evaluates to
    `via <= -0.20mm` -- impossible for any via -- and the run spends 2967s
    dropping balls and retrying under-pad before finishing. qfn_fanout does the
    same chip in 2.4s with `--escape-method underpad --allow-via-in-pad`.

    Returns a human-readable reason when the footprint is not a grid array, or
    None when it looks like a genuine BGA/PGA/LGA.
    """
    pads = footprint.pads
    if not pads or grid is None:
        return None
    lattice = min(grid.pitch_x, grid.pitch_y)
    if lattice <= 0:
        return None

    # True nearest-neighbour spacing, in 2D.
    near = None
    for i, a in enumerate(pads):
        for b in pads[i + 1:]:
            d = _math.hypot(a.global_x - b.global_x, a.global_y - b.global_y)
            if d > 1e-6 and (near is None or d < near):
                near = d
    if near is None:
        return None

    cells = max(1, len(grid.rows) * len(grid.cols))
    occupancy = len(pads) / cells
    # A ball grid fills its lattice; a staggered lattice is a checkerboard, so
    # its nearest neighbour is ~sqrt(2)x or 2x the lattice step and most cells
    # are empty. Require BOTH signals so a sparse-but-true BGA (depopulated
    # centre) is not misread -- those still sit ON the lattice (near ~ lattice).
    if near > lattice * 1.4 and occupancy < 0.35:
        return (f"pads sit on a {lattice:.3f}mm lattice but no two are closer "
                f"than {near:.3f}mm, and only {occupancy * 100:.0f}% of the "
                f"{len(grid.rows)}x{len(grid.cols)} lattice is populated")
    return None

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


def is_edge_pad(pad_x: float, pad_y: float, grid: BGAGrid,
                tolerance: float = EDGE_PAD_TOLERANCE,
                preferred_dir: str = None) -> Tuple[bool, str]:
    """
    Check if a pad is on the outer edge of the BGA.
    Returns (is_edge, escape_direction).

    Edge pads are on the outermost row or column and can escape directly
    without needing a 45° stub. A CORNER pad sits on two edges and has two
    legal escape directions; preferred_dir (#469, the net's target side)
    picks between them when it matches one -- otherwise the fixed
    right/left/down/up priority stands.
    """
    on_left = abs(pad_x - grid.cols[0]) < tolerance
    on_right = abs(pad_x - grid.cols[-1]) < tolerance
    on_top = abs(pad_y - grid.rows[0]) < tolerance
    on_bottom = abs(pad_y - grid.rows[-1]) < tolerance

    legal = []
    if on_right:
        legal.append('right')
    if on_left:
        legal.append('left')
    if on_bottom:
        legal.append('down')
    if on_top:
        legal.append('up')
    if not legal:
        return False, ''
    if preferred_dir and preferred_dir in legal:
        return True, preferred_dir
    return True, legal[0]

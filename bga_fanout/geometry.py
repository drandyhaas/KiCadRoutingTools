"""
Geometry calculations for BGA fanout routing.

Functions for calculating 45-degree stubs, exit points, and jog endpoints.
"""
from __future__ import annotations

import math
from typing import List, Tuple, Optional, Dict

from bga_fanout.types import BGAGrid, Channel

# Reference prefixes place_fanout_clearance treats as movable passives (keep in
# sync with its --cap-prefix default): an unlocked <=2-copper-pad part with one
# of these prefixes will be nudged off fanout vias by the next pipeline step,
# so the fanout may ignore its copper when placing vias. Everything else --
# locked parts, connectors, test points, ICs -- never moves (#253/#254).
MOVABLE_PASSIVE_PREFIXES = ('C', 'R', 'FB')


def immovable_foreign_pads(pcb_data, exclude_ref: str) -> List:
    """Copper pads of foreign footprints that NO later pipeline step will move:
    locked footprints, and any part place_fanout_clearance won't relocate
    (i.e. not an unlocked 2-pad C*/R*/FB* passive). A through via placed by the
    fanout must clear these on every copper layer; movable caps are deliberately
    NOT included (cap-opt moves them off the vias afterwards)."""
    out = []
    for fp in pcb_data.footprints.values():
        if fp.reference == exclude_ref:
            continue
        copper_pads = [p for p in fp.pads
                       if any(str(l).endswith('.Cu') for l in p.layers)]
        if not copper_pads:
            continue
        movable = (not getattr(fp, 'locked', False)
                   and fp.reference.startswith(MOVABLE_PASSIVE_PREFIXES)
                   and len(copper_pads) <= 2)
        if movable:
            continue
        out.extend(copper_pads)
    return out


def via_clears_pad_rects(x: float, y: float, v_half: float, clearance: float,
                         pads) -> bool:
    """True if a through via at (x, y) clears every pad in `pads` (rect model,
    rect_rotation-aware) by `clearance` edge-to-edge. Vias span all copper
    layers, so the pads' own layers are irrelevant. The previous axis-aligned
    bbox under-covered a ROTATED pad's protruding corners, passing via sites
    whose ring grazed the real copper."""
    from routing_utils import point_to_pad_rect_dist
    for p in pads:
        if point_to_pad_rect_dist(x, y, p) < v_half + clearance - 1e-9:
            return False
    return True


def clamp_via_to_pad(via_size: float, via_drill: float, pad,
                     floors) -> Tuple[float, float, str, int]:
    """Issue #202: shrink a via-in-pad so it never bulges past the pad edge.

    A via dropped at a pad centre whose diameter exceeds the pad's smallest
    dimension bulges past the pad copper. A neighbouring different-net trace that
    legitimately cleared the (smaller) pad then grazes the (larger) via -- a real
    VIA-SEGMENT DRC error baked into the board before any signal routing runs
    (the dominant via-seg source on keks: 163 grazes from Ø0.5 vias in 0.41 mm
    pads).

    Clamp the via to the pad's min dimension (a circular via inscribes a
    rect/oval pad at min(size_x, size_y); for a circle pad that is the diameter),
    but NEVER below the fab via floor -- a pad smaller than the smallest
    manufacturable via keeps that floor via (it still bulges) rather than ship an
    unmanufacturable one. The drill follows to hold the annular ring at the fab
    floor.

    The clamp belongs at via PLACEMENT (not a post-pass) so the escape's own
    keep-out modelling uses the real, smaller via and neighbouring fanout tracks
    can route past it.

    ``floors`` is the fab-tier floor LADDER (nominal first, then escalation rungs;
    see fab_tiers.fab_floor_ladder): each rung is a flat floor dict. The via is
    clamped to the first rung whose via fits the pad; if the pad is smaller than
    even the deepest rung's via, it's held at that deepest floor (still bulges).

    Returns (size, drill, status, rung): status 'fits' (unchanged), 'clamped'
    (shrunk to fit), or 'floor' (pad < deepest fab floor). ``rung`` is the floor
    index used (0 = nominal/standard; >0 = escalated to a more-costly tier).
    """
    pad_min = min(pad.size_x, pad.size_y)
    if via_size <= pad_min + 1e-9:
        return via_size, via_drill, 'fits', 0

    # First rung whose smallest manufacturable via fits this pad.
    for rung, fab in enumerate(floors):
        floor_dia = fab['via_diameter']
        floor_drill = fab['via_drill']
        if pad_min < floor_dia - 1e-9:
            continue
        # The fab's SMALLEST manufacturable annular ring for THIS rung -- the one its
        # via pair uses ((0.45-0.20)/2 standard, (0.25-0.15)/2 advanced), NOT the
        # larger fab['annular'] (which would force the drill below the floor and
        # needlessly over-shrink a via that fits the pad fine).
        min_annular = (floor_dia - floor_drill) / 2.0
        new_size = round(pad_min, 4)
        # Keep the configured drill, only thinning it as far as the ring needs, and
        # never below this rung's drill floor.
        new_drill = max(min(via_drill, new_size - 2 * min_annular), floor_drill)
        return new_size, round(new_drill, 4), 'clamped', rung

    # Even the deepest rung's via bulges past this pad: hold it at that floor.
    deepest = len(floors) - 1
    return floors[deepest]['via_diameter'], floors[deepest]['via_drill'], 'floor', deepest


def create_45_stub(pad_x: float, pad_y: float,
                   channel: Channel,
                   escape_dir: str,
                   channel_offset: float = 0.0) -> Tuple[float, float]:
    """
    Create 45-degree stub from pad to channel.

    For differential pairs, the offset is applied AFTER calculating the 45° stub
    based on the channel center. This ensures both P and N travel the same
    horizontal/vertical distance before applying the parallel offset.

    Args:
        pad_x, pad_y: Pad position
        channel: Target channel
        escape_dir: Direction of escape
        channel_offset: Offset from channel center (for diff pairs)

    Returns:
        End position of the 45° stub
    """
    if channel.orientation == 'horizontal':
        # Target Y includes the offset
        target_y = channel.position + channel_offset
        # For true 45°, dx = |dy|
        dy_to_target = target_y - pad_y
        if escape_dir == 'right':
            dx = abs(dy_to_target)
        else:
            dx = -abs(dy_to_target)

        return (pad_x + dx, target_y)
    else:
        # Target X includes the offset
        target_x = channel.position + channel_offset
        # For true 45°, dy = |dx|
        dx_to_target = target_x - pad_x
        if escape_dir == 'down':
            dy = abs(dx_to_target)
        else:
            dy = -abs(dx_to_target)

        return (target_x, pad_y + dy)


def calculate_exit_point(stub_end: Tuple[float, float],
                         channel: Channel,
                         escape_dir: str,
                         grid: BGAGrid,
                         margin: float = 0.5,
                         channel_offset: float = 0.0) -> Tuple[float, float]:
    """Calculate where the route exits the BGA boundary."""
    if channel.orientation == 'horizontal':
        exit_y = channel.position + channel_offset
        if escape_dir == 'right':
            return (grid.max_x + margin, exit_y)
        else:
            return (grid.min_x - margin, exit_y)
    else:
        exit_x = channel.position + channel_offset
        if escape_dir == 'down':
            return (exit_x, grid.max_y + margin)
        else:
            return (exit_x, grid.min_y - margin)


def calculate_jog_end(exit_pos: Tuple[float, float],
                      escape_dir: str,
                      layer: str,
                      layers: List[str],
                      jog_length: float,
                      is_diff_pair: bool = False,
                      is_outside_track: bool = False,
                      pair_spacing: float = 0.0,
                      grid_step: float = 0.0) -> Tuple[Tuple[float, float], Optional[Tuple[float, float]]]:
    """
    Calculate the end position of the 45° jog at the exit.

    For differential pairs, the outside track needs to extend further before
    bending to maintain constant spacing through the 45° turn.

    Jog direction depends on layer:
    - Top layer (F.Cu): 45° to the left (from perspective walking towards BGA edge)
    - Bottom layer (B.Cu): 45° to the right
    - Middle layers: linear interpolation

    Args:
        exit_pos: Starting point of jog
        escape_dir: Direction of escape ('left', 'right', 'up', 'down')
        layer: Current layer
        layers: List of all available layers
        jog_length: Length of the jog (distance from BGA edge to first pad row/col)
        is_diff_pair: Whether this is part of a differential pair
        is_outside_track: Whether this is the outside track of the pair (needs extension)
        pair_spacing: Spacing between P and N tracks

    Returns:
        (jog_end, extension_point) - extension_point is the intermediate point for outside tracks
    """
    # Calculate layer position: 0 = top (left jog), 1 = bottom (right jog)
    try:
        layer_idx = layers.index(layer)
    except ValueError:
        layer_idx = 0

    num_layers = len(layers)
    if num_layers <= 1:
        layer_factor = 0.0  # Default to left jog
    else:
        layer_factor = layer_idx / (num_layers - 1)  # 0 to 1

    # Jog angle: -1 = left, +1 = right (from perspective of walking towards edge)
    # layer_factor 0 (top) -> -1 (left)
    # layer_factor 1 (bottom) -> +1 (right)
    jog_direction = 2 * layer_factor - 1  # Maps 0->-1, 1->+1

    # Calculate jog components based on escape direction
    # At 45°, both components equal jog_length / sqrt(2)
    # Reduce by factor of 4 for a shorter angled segment at the end of each stub
    diag = jog_length / math.sqrt(2) / 4

    ex, ey = exit_pos
    extension_point = None

    # For differential pairs, outside track extends further before bending
    # To maintain constant perpendicular spacing through a 45° turn:
    # extension = pair_spacing * (sqrt(2) - 1) ≈ 0.414 * pair_spacing
    if is_diff_pair and is_outside_track:
        extension = pair_spacing * (math.sqrt(2) - 1)
        if escape_dir == 'right':
            extension_point = (ex + extension, ey)
            ex = ex + extension
        elif escape_dir == 'left':
            extension_point = (ex - extension, ey)
            ex = ex - extension
        elif escape_dir == 'down':
            extension_point = (ex, ey + extension)
            ey = ey + extension
        else:  # up
            extension_point = (ex, ey - extension)
            ey = ey - extension

    if escape_dir == 'right':
        # Walking right, left is up (-Y), right is down (+Y)
        jog_end = (ex + diag, ey + jog_direction * diag)
    elif escape_dir == 'left':
        # Walking left, left is down (+Y), right is up (-Y)
        jog_end = (ex - diag, ey - jog_direction * diag)
    elif escape_dir == 'down':
        # Walking down, left is right (+X), right is left (-X)
        jog_end = (ex - jog_direction * diag, ey + diag)
    else:  # up
        # Walking up, left is left (-X), right is right (+X)
        jog_end = (ex + jog_direction * diag, ey - diag)

    # Land the stub end on the routing grid (issue #149) so the router has an
    # on-grid terminal and a foreign track on the nearest cell can't graze this
    # end by a sub-cell amount. The grid is anchored at the origin (the router's
    # grid nodes are integer multiples of grid_step).
    if grid_step > 0:
        jog_end = (round(jog_end[0] / grid_step) * grid_step,
                   round(jog_end[1] / grid_step) * grid_step)

    return jog_end, extension_point

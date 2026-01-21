"""
Obstacle map building for copper plane via placement and routing.

Provides functions to build obstacle maps for:
- Via placement (considering all layers)
- Single-layer routing (for via-to-pad traces)
"""

import math
from typing import List, Dict, Tuple, Optional

from kicad_parser import PCBData, Pad, Segment
from routing_config import GridRouteConfig, GridCoord
from routing_utils import iter_pad_blocked_cells

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap


def block_circle(obstacles: GridObstacleMap, cx: int, cy: int, radius_sq: float,
                 layer_idx: Optional[int] = None, via_mode: bool = False):
    """Block cells in a circular region.

    Args:
        obstacles: The obstacle map to update
        cx, cy: Center of the circle in grid coordinates
        radius_sq: Squared radius in grid units (avoids precision loss)
        layer_idx: Layer index for routing obstacles (ignored if via_mode=True)
        via_mode: If True, use add_blocked_via; if False, use add_blocked_cell
    """
    radius_int = int(math.ceil(math.sqrt(radius_sq)))
    for ex in range(-radius_int, radius_int + 1):
        for ey in range(-radius_int, radius_int + 1):
            if ex*ex + ey*ey <= radius_sq:
                if via_mode:
                    obstacles.add_blocked_via(cx + ex, cy + ey)
                else:
                    obstacles.add_blocked_cell(cx + ex, cy + ey, layer_idx)


def identify_target_pads(
    pcb_data: PCBData,
    net_id: int,
    plane_layer: str
) -> List[Dict]:
    """
    Identify pads that need via connections to the plane layer.

    Returns list of dicts with pad info and connection type:
    - "through_hole": Through-hole pad - can connect on any layer, no via needed
    - "direct": SMD pad on plane layer - zone connects directly, no via needed
    - "via_needed": SMD pad on opposite layer - needs via + trace
    """
    target_pads = []
    pads = pcb_data.pads_by_net.get(net_id, [])

    for pad in pads:
        # Check if pad has drill (through-hole)
        if pad.drill > 0:
            # Through-hole pad - directly connects to all layers including plane
            target_pads.append({
                'pad': pad,
                'type': 'through_hole',
                'needs_via': False,
                'needs_trace': False
            })
        elif plane_layer in pad.layers or "*.Cu" in pad.layers:
            # SMD pad on plane layer - direct zone connection
            target_pads.append({
                'pad': pad,
                'type': 'direct',
                'needs_via': False,
                'needs_trace': False
            })
        else:
            # SMD pad NOT on plane layer - needs via
            # Get the pad's actual layer for trace routing
            pad_layer = None
            for layer in pad.layers:
                if layer.endswith('.Cu') and not layer.startswith('*'):
                    pad_layer = layer
                    break

            target_pads.append({
                'pad': pad,
                'type': 'via_needed',
                'needs_via': True,
                'needs_trace': True,  # May need trace if via can't be at pad center
                'pad_layer': pad_layer
            })

    return target_pads


def build_via_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int
) -> GridObstacleMap:
    """
    Build obstacle map for via placement.

    Blocks:
    - Existing vias (all nets - via-via clearance)
    - Pads on all layers (except target net pads)
    - Tracks on all layers (except target net)
    - Board edge clearance
    - Through-hole pad drills (hole-to-hole clearance)
    """
    coord = GridCoord(config.grid_step)
    num_layers = len(config.layers)

    obstacles = GridObstacleMap(num_layers)

    # Precompute expansion for via-via clearance (squared, in grid units)
    # Add half grid step cushion to account for grid discretization
    grid_cushion = config.grid_step / 2
    via_via_expansion_mm = config.via_size + config.clearance + grid_cushion
    via_via_radius_sq = (via_via_expansion_mm / config.grid_step) ** 2
    via_via_radius_int = int(math.ceil(math.sqrt(via_via_radius_sq)))

    # Add existing vias as obstacles (including same net - can't place via too close to another)
    for via in pcb_data.vias:
        gx, gy = coord.to_grid(via.x, via.y)
        # Block via placement within via-via clearance
        for ex in range(-via_via_radius_int, via_via_radius_int + 1):
            for ey in range(-via_via_radius_int, via_via_radius_int + 1):
                if ex*ex + ey*ey <= via_via_radius_sq:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Add existing segments as obstacles (via can't overlap with tracks on ANY layer)
    # Since vias span all layers, we must check segments on all copper layers, not just config.layers
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        # Include any copper layer (*.Cu)
        if not seg.layer.endswith('.Cu'):
            continue
        # Use actual segment width for clearance calculation (not config.track_width)
        # Include grid cushion for discretization
        seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance + grid_cushion
        _add_segment_via_obstacle(obstacles, seg, coord, seg_expansion_mm)

    # Add pads as obstacles (excluding target net pads)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            _add_pad_via_obstacle(obstacles, pad, coord, config)

    # Add board edge via blocking
    _add_board_edge_via_obstacles(obstacles, pcb_data, config)

    # Add hole-to-hole clearance blocking for existing drills
    _add_drill_hole_via_obstacles(obstacles, pcb_data, config, exclude_net_id)

    return obstacles


def _add_segment_via_obstacle(obstacles: GridObstacleMap, seg: Segment,
                               coord: GridCoord, expansion_mm: float):
    """Add a segment as via blocking obstacle."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Compute squared radius in grid units (avoids precision loss)
    radius_sq = (expansion_mm / coord.grid_step) ** 2

    # Use Bresenham-style line blocking
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    x, y = gx1, gy1
    if dx > dy:
        err = dx / 2
        while x != gx2:
            block_circle(obstacles, x, y, radius_sq, via_mode=True)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            block_circle(obstacles, x, y, radius_sq, via_mode=True)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    block_circle(obstacles, gx2, gy2, radius_sq, via_mode=True)


def _add_pad_via_obstacle(obstacles: GridObstacleMap, pad: Pad,
                           coord: GridCoord, config: GridRouteConfig):
    """Add a pad as via blocking obstacle using rectangular shape with rounded corners."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    half_width = pad.size_x / 2
    half_height = pad.size_y / 2
    margin = config.via_size / 2 + config.clearance
    # Corner radius based on pad shape (circle/oval use min dimension, roundrect uses rratio)
    if pad.shape in ('circle', 'oval'):
        corner_radius = min(half_width, half_height)
    elif pad.shape == 'roundrect':
        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
    else:
        corner_radius = 0

    for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius):
        obstacles.add_blocked_via(cell_gx, cell_gy)


def _add_board_edge_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig):
    """Block via placement near board edges."""
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        return

    coord = GridCoord(config.grid_step)
    min_x, min_y, max_x, max_y = board_bounds

    edge_clearance = config.board_edge_clearance if config.board_edge_clearance > 0 else config.clearance
    via_edge_clearance = edge_clearance + config.via_size / 2
    via_expand = coord.to_grid_dist(via_edge_clearance)

    gmin_x, gmin_y = coord.to_grid(min_x, min_y)
    gmax_x, gmax_y = coord.to_grid(max_x, max_y)
    grid_margin = via_expand + 5

    # Block edges
    for gx in range(gmin_x - grid_margin, gmax_x + grid_margin + 1):
        for gy in range(gmin_y - grid_margin, gmax_y + grid_margin + 1):
            # Outside board + margin
            if gx < gmin_x + via_expand or gx > gmax_x - via_expand or \
               gy < gmin_y + via_expand or gy > gmax_y - via_expand:
                obstacles.add_blocked_via(gx, gy)


def _add_drill_hole_via_obstacles(obstacles: GridObstacleMap, pcb_data: PCBData,
                                    config: GridRouteConfig, exclude_net_id: int):
    """Block via placement near existing drill holes."""
    if config.hole_to_hole_clearance <= 0:
        return

    coord = GridCoord(config.grid_step)

    # Collect drill holes
    drill_holes = []

    for via in pcb_data.vias:
        drill_holes.append((via.x, via.y, via.drill))

    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            if pad.drill > 0:
                drill_holes.append((pad.global_x, pad.global_y, pad.drill))

    for hx, hy, drill_dia in drill_holes:
        required_dist = drill_dia / 2 + config.via_drill / 2 + config.hole_to_hole_clearance
        radius_sq = (required_dist / config.grid_step) ** 2
        gx, gy = coord.to_grid(hx, hy)
        block_circle(obstacles, gx, gy, radius_sq, via_mode=True)


def block_via_position(obstacles: GridObstacleMap, via_x: float, via_y: float,
                        coord: GridCoord, hole_to_hole_clearance: float, via_drill: float):
    """Block the area around a newly placed via for hole-to-hole clearance.

    Args:
        obstacles: The obstacle map to update
        via_x, via_y: Position of the placed via
        coord: Grid coordinate converter
        hole_to_hole_clearance: Minimum clearance between drill holes
        via_drill: Drill diameter of the via
    """
    gx, gy = coord.to_grid(via_x, via_y)
    # Required distance: (this_drill/2) + (other_drill/2) + clearance
    # Since we're placing vias of the same size, it's: via_drill + clearance
    required_dist = via_drill + hole_to_hole_clearance
    radius_sq = (required_dist / coord.grid_step) ** 2
    block_circle(obstacles, gx, gy, radius_sq, via_mode=True)


def build_routing_obstacle_map(
    pcb_data: PCBData,
    config: GridRouteConfig,
    exclude_net_id: int,
    route_layer: str,
    skip_pad_blocking: bool = False
) -> GridObstacleMap:
    """
    Build obstacle map for A* routing on a specific layer.

    Blocks:
    - Pads on the route layer (except target net pads) - skipped if skip_pad_blocking
    - Segments on the route layer (except target net)
    - Vias (they occupy space on all layers)

    Args:
        skip_pad_blocking: If True, don't block based on pad clearances.
                          Use this for plane via connections where DRC is lenient.
    """
    coord = GridCoord(config.grid_step)
    # Single layer routing
    num_layers = 1
    layer_idx = 0

    obstacles = GridObstacleMap(num_layers)

    # Add pads on this layer as obstacles (excluding target net)
    # Skip this entirely for plane connections where we're more lenient
    if not skip_pad_blocking:
        for net_id, pads in pcb_data.pads_by_net.items():
            if net_id == exclude_net_id:
                continue
            for pad in pads:
                # Check if pad is on the route layer
                if route_layer in pad.layers or "*.Cu" in pad.layers:
                    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                    half_width = pad.size_x / 2
                    half_height = pad.size_y / 2
                    margin = config.track_width / 2 + config.clearance
                    # Corner radius based on pad shape
                    if pad.shape in ('circle', 'oval'):
                        corner_radius = min(half_width, half_height)
                    elif pad.shape == 'roundrect':
                        corner_radius = pad.roundrect_rratio * min(pad.size_x, pad.size_y)
                    else:
                        corner_radius = 0
                    for cell_gx, cell_gy in iter_pad_blocked_cells(gx, gy, half_width, half_height, margin, config.grid_step, corner_radius):
                        obstacles.add_blocked_cell(cell_gx, cell_gy, layer_idx)

    # Add segments on this layer as obstacles (excluding target net)
    # Use actual segment width for proper clearance calculation
    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        if seg.layer != route_layer:
            continue
        # Clearance needed: our track half-width + existing segment half-width + clearance
        seg_expansion_mm = config.track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist(seg_expansion_mm))
        _add_segment_routing_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)

    # Add vias as obstacles (they block all layers)
    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        via_expansion = coord.to_grid_dist(via.size / 2 + config.track_width / 2 + config.clearance)
        for ex in range(-via_expansion, via_expansion + 1):
            for ey in range(-via_expansion, via_expansion + 1):
                if ex*ex + ey*ey <= via_expansion * via_expansion:
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    return obstacles


def _add_segment_routing_obstacle(obstacles: GridObstacleMap, seg: Segment,
                                    coord: GridCoord, layer_idx: int, expansion_grid: int):
    """Add a segment as a routing obstacle on a specific layer."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham line with expansion
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1

    x, y = gx1, gy1
    radius_sq = expansion_grid * expansion_grid
    if dx > dy:
        err = dx / 2
        while x != gx2:
            block_circle(obstacles, x, y, radius_sq, layer_idx=layer_idx, via_mode=False)
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != gy2:
            block_circle(obstacles, x, y, radius_sq, layer_idx=layer_idx, via_mode=False)
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    block_circle(obstacles, gx2, gy2, radius_sq, layer_idx=layer_idx, via_mode=False)

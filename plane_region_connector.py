"""
Plane Region Connector - Detects and routes between disconnected plane regions.

After power planes are created, regions may be effectively split due to vias and
traces from other nets cutting through the plane. This module detects disconnected
regions and routes wide, short tracks between them to ensure electrical continuity.
"""
from __future__ import annotations

from typing import List, Optional, Tuple, Dict, Set
from collections import deque
import math

import numpy as np

from kicad_parser import PCBData, Via, Segment, Pad, POSITION_DECIMALS, pad_drill_circles
from routing_config import GridRouteConfig, GridCoord
from routing_utils import (point_in_pad_rect, pad_rect_halfspan, filter_cells_in_pad_rect,
                           segment_blocked_cells_array)
from geometry_utils import UnionFind
from bresenham_utils import walk_line
from obstacle_map import (add_board_edge_obstacles, add_user_keepout_obstacles,
                          add_rule_area_keepout_obstacles,
                          _batch_cells_one_layer, _batch_vias,
                          block_track_cells_near_drills, _pad_has_copper)
from plane_obstacle_builder import (
    _precompute_circle_offsets,
    _batch_block_circles_via, _batch_block_circles_cell
)

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter
from single_ended_routing import _track_margin_for_width
import routing_defaults as defaults


def _collect_anchor_points(
    net_id: int,
    zone_bounds: Tuple[float, float, float, float],
    pcb_data: PCBData,
    coord: GridCoord,
    zone_layers: Set[str],
    routing_layers: List[str]
) -> Tuple[List[Tuple[float, float]], List[Tuple[int, int]]]:
    """
    Collect anchor points (vias and pads) for flood fill connectivity analysis.

    Args:
        net_id: Net ID of the plane
        zone_bounds: (min_x, min_y, max_x, max_y) of the zone area
        pcb_data: PCB data with vias and pads
        coord: Grid coordinate converter
        zone_layers: Layers that have zones for this net
        routing_layers: All copper layers for routing

    Returns:
        Tuple of (anchor_points, anchor_grid_points)
    """
    min_x, min_y, max_x, max_y = zone_bounds
    anchor_points: List[Tuple[float, float]] = []
    anchor_grid_points: List[Tuple[int, int]] = []
    seen_anchors: Set[Tuple[float, float]] = set()

    def via_connects_layer(via_layers: List[str], layer: str) -> bool:
        """Check if a via connects to a given layer."""
        if layer in via_layers:
            return True
        if 'F.Cu' in via_layers and 'B.Cu' in via_layers:
            return layer in routing_layers
        return False

    # Add vias of our net that touch any zone layer
    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        touches_zone = any(via_connects_layer(via.layers, zl) for zl in zone_layers)
        if not touches_zone:
            continue
        if min_x <= via.x <= max_x and min_y <= via.y <= max_y:
            key = (round(via.x, POSITION_DECIMALS), round(via.y, POSITION_DECIMALS))
            if key not in seen_anchors:
                seen_anchors.add(key)
                anchor_points.append((via.x, via.y))
                anchor_grid_points.append(coord.to_grid(via.x, via.y))

    # Add pads of our net on any zone layer
    for pad in pcb_data.pads_by_net.get(net_id, []):
        touches_zone = '*.Cu' in pad.layers or any(zl in pad.layers for zl in zone_layers)
        if not touches_zone:
            continue
        if min_x <= pad.global_x <= max_x and min_y <= pad.global_y <= max_y:
            key = (round(pad.global_x, POSITION_DECIMALS), round(pad.global_y, POSITION_DECIMALS))
            if key not in seen_anchors:
                seen_anchors.add(key)
                anchor_points.append((pad.global_x, pad.global_y))
                anchor_grid_points.append(coord.to_grid(pad.global_x, pad.global_y))

    return anchor_points, anchor_grid_points


def _collect_cross_layer_points(
    net_id: int,
    pcb_data: PCBData,
    routing_layers: List[str]
) -> List[Tuple[float, float, Set[str]]]:
    """
    Collect cross-layer connection points (vias and through-hole pads) for connectivity analysis.

    Args:
        net_id: Net ID of the plane
        pcb_data: PCB data with vias and pads
        routing_layers: All copper layers for routing

    Returns:
        List of (x, y, connected_layers) tuples
    """
    def get_via_connected_layers(via_layers: List[str]) -> Set[str]:
        """Get all layers a via connects to."""
        if 'F.Cu' in via_layers and 'B.Cu' in via_layers:
            return set(routing_layers)
        return set(via_layers)

    cross_layer_points: List[Tuple[float, float, Set[str]]] = []

    for via in pcb_data.vias:
        if via.net_id == net_id:
            cross_layer_points.append((via.x, via.y, get_via_connected_layers(via.layers)))

    for pad in pcb_data.pads_by_net.get(net_id, []):
        if '*.Cu' in pad.layers:  # Through-hole pad
            cross_layer_points.append((pad.global_x, pad.global_y, set(routing_layers)))

    return cross_layer_points


def _zone_interior_cells(
    net_id: int,
    layer: str,
    pcb_data: PCBData,
    coord: GridCoord,
    bounds_grid: Tuple[int, int, int, int],
) -> Optional[Set[Tuple[int, int]]]:
    """Grid cells whose centers lie inside ANY of this net's zone OUTLINES on
    `layer` (scanline rasterization, O(rows x edges) per polygon).

    The connectivity flood used to traverse every unblocked cell in the zone
    BOUNDS -- including empty space with no copper at all -- so a plane split
    into multiple outline islands (castor_pollux +3.3V: 3 outlines sharing
    In2.Cu with +3.3VA) graded as ONE region and the repair never bridged it
    (#217/#189 false 'fully connected'). Zone-layer traversal must stay
    inside the net's own outlines; same-net tracks crossing the gaps remain
    traversable via net_segment_cells.

    Returns None when the net has no zone polygons on the layer (caller keeps
    the legacy unrestricted flood -- e.g. zones not present in pcb_data).
    """
    polys = [z.polygon for z in (getattr(pcb_data, 'zones', None) or [])
             if z.net_id == net_id and z.layer == layer
             and getattr(z, 'polygon', None) and len(z.polygon) >= 3]
    if not polys:
        return None
    min_gx, max_gx, min_gy, max_gy = bounds_grid
    inside: Set[Tuple[int, int]] = set()
    for poly in polys:
        for gy in range(min_gy, max_gy + 1):
            _, y = coord.to_float(0, gy)
            xs = []
            n = len(poly)
            for i in range(n):
                x1, y1 = poly[i]
                x2, y2 = poly[(i + 1) % n]
                if (y1 <= y < y2) or (y2 <= y < y1):
                    xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
            xs.sort()
            for k in range(0, len(xs) - 1, 2):
                gx_lo, _ = coord.to_grid(xs[k], y)
                gx_hi, _ = coord.to_grid(xs[k + 1], y)
                for gx in range(max(gx_lo, min_gx), min(gx_hi, max_gx) + 1):
                    inside.add((gx, gy))
    return inside


def _build_layer_blocked_set(
    layer: str,
    net_id: int,
    pcb_data: PCBData,
    coord: GridCoord,
    layer_clearance: float
) -> Tuple[Set[Tuple[int, int]], Set[Tuple[int, int]]]:
    """
    Build blocked set and net segment cells for a single layer.

    Args:
        layer: Layer name to build blocked set for
        net_id: Net ID of the plane (excluded from blocking)
        pcb_data: PCB data
        coord: Grid coordinate converter
        layer_clearance: Clearance for zone fill

    Returns:
        Tuple of (blocked_cells, net_segment_cells)
    """
    blocked: Set[Tuple[int, int]] = set()

    # Block cells for other nets' vias
    for via in pcb_data.vias:
        if via.net_id == net_id:
            continue
        via_block_radius = max(1, coord.to_grid_dist(via.size / 2 + layer_clearance))
        gx, gy = coord.to_grid(via.x, via.y)
        for dx in range(-via_block_radius, via_block_radius + 1):
            for dy in range(-via_block_radius, via_block_radius + 1):
                if dx * dx + dy * dy <= via_block_radius * via_block_radius:
                    blocked.add((gx + dx, gy + dy))

    # Block cells for other nets' segments on this layer
    for seg in pcb_data.segments:
        if seg.net_id == net_id:
            continue
        if seg.layer != layer:
            continue
        seg_block_radius = max(1, coord.to_grid_dist(seg.width / 2 + layer_clearance))
        _block_segment_cells(blocked, seg, coord, seg_block_radius)

    # Block cells for other nets' pads that touch this layer
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id == net_id:
            continue
        for pad in pads:
            if layer not in pad.layers and '*.Cu' not in pad.layers:
                continue
            _block_pad_cells(blocked, pad, coord, layer_clearance)

    # Mark cells along same-net segments as connected
    net_segment_cells: Set[Tuple[int, int]] = set()
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        if seg.layer != layer:
            continue
        _add_segment_cells(net_segment_cells, seg, coord)

    return blocked, net_segment_cells


def find_disconnected_zone_regions(
    net_id: int,
    plane_layer: str,
    zone_bounds: Tuple[float, float, float, float],  # min_x, min_y, max_x, max_y
    pcb_data: PCBData,
    config: GridRouteConfig,
    zone_clearance: float = 0.2,
    analysis_grid_step: float = 0.5,  # Coarser grid for faster analysis
    routing_layers: Optional[List[str]] = None,  # All copper layers to check for cross-layer connectivity
    zone_layers: Optional[Set[str]] = None,  # Layers that have zones for this net (allow flood fill)
    debug: bool = False,  # If True, return debug paths showing connectivity
    zone_clearances: Optional[Dict[str, float]] = None  # Per-layer zone clearances (layer -> clearance)
) -> Tuple[List[List[Tuple[float, float]]], List[Set[Tuple[int, int]]], List[Tuple[List[Tuple[float, float]], str]]]:
    """
    Find disconnected regions within a zone using flood fill on a grid.

    Checks connectivity across copper layers - regions are considered connected if
    they can reach each other through vias, through-hole pads, or segments.

    On layers WITH zones: flood fill through unblocked copper
    On layers WITHOUT zones: only traverse along same-net segments

    Args:
        net_id: Net ID of the plane
        plane_layer: Layer of the plane (e.g., 'B.Cu')
        zone_bounds: (min_x, min_y, max_x, max_y) of the zone area
        pcb_data: PCB data with vias, segments, pads
        config: Routing configuration
        zone_clearance: Default clearance for zone fill around obstacles
        analysis_grid_step: Grid step for connectivity analysis (coarser = faster)
        routing_layers: All copper layers to check for cross-layer connectivity
        zone_layers: Layers that have zones for this net (if None, only plane_layer)
        debug: If True, return debug paths showing how regions are connected
        zone_clearances: Per-layer zone clearances (layer -> clearance). Falls back to zone_clearance if not specified.

    Returns:
        Tuple of:
        - List of anchor point lists (vias/pads positions) per region
        - List of grid cell sets per region (for finding closest points)
        - List of (path, layer) tuples showing connectivity paths (if debug=True, else empty)
    """
    # Use a coarser grid for connectivity analysis (much faster)
    coord = GridCoord(analysis_grid_step)
    min_x, min_y, max_x, max_y = zone_bounds

    # Convert bounds to grid
    min_gx, min_gy = coord.to_grid(min_x, min_y)
    max_gx, max_gy = coord.to_grid(max_x, max_y)

    # If no routing layers specified, just use the plane layer (legacy behavior)
    if routing_layers is None:
        routing_layers = [plane_layer]

    # If no zone layers specified, only the plane_layer has a zone
    if zone_layers is None:
        zone_layers = {plane_layer}

    # Collect anchor points using helper function
    anchor_points, anchor_grid_points = _collect_anchor_points(
        net_id, zone_bounds, pcb_data, coord, zone_layers, routing_layers
    )

    if len(anchor_points) < 2:
        # Not enough anchors to have disconnected regions
        return [anchor_points], [set(anchor_grid_points)], []

    # Collect cross-layer connection points using helper function
    cross_layer_points = _collect_cross_layer_points(net_id, pcb_data, routing_layers)

    # Helper function for via layer connections (still needed for loop below)
    def get_via_connected_layers(via_layers: List[str]) -> Set[str]:
        """Get all layers a via connects to."""
        if 'F.Cu' in via_layers and 'B.Cu' in via_layers:
            return set(routing_layers)
        return set(via_layers)

    # Build map from grid position to cross-layer point index
    grid_to_crosslayer: Dict[Tuple[int, int], List[int]] = {}
    for i, (x, y, layers) in enumerate(cross_layer_points):
        gp = coord.to_grid(x, y)
        if gp not in grid_to_crosslayer:
            grid_to_crosslayer[gp] = []
        grid_to_crosslayer[gp].append(i)

    # Union-find for cross-layer points
    cl_uf = UnionFind()

    # Union-find for anchors (will merge based on cross-layer connectivity)
    anchor_uf = UnionFind()

    # Debug paths: list of (path_points, layer_name) showing connectivity
    debug_paths: List[Tuple[List[Tuple[float, float]], str]] = []

    # First pass: connect cross-layer points that are at the same location
    # (e.g., a via and a pad at the same spot)
    for gp, indices in grid_to_crosslayer.items():
        for i in range(1, len(indices)):
            cl_uf.union(indices[0], indices[i])

    # Connect cross-layer points via same-net segments (segments directly connect points)
    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        # Find cross-layer points at segment endpoints
        start_gp = coord.to_grid(seg.start_x, seg.start_y)
        end_gp = coord.to_grid(seg.end_x, seg.end_y)
        start_cls = grid_to_crosslayer.get(start_gp, [])
        end_cls = grid_to_crosslayer.get(end_gp, [])
        # Union any cross-layer points at start with any at end (if segment is on their layer)
        for si in start_cls:
            if seg.layer in cross_layer_points[si][2]:
                for ei in end_cls:
                    if seg.layer in cross_layer_points[ei][2]:
                        cl_uf.union(si, ei)

    # For each layer, do flood fill to find connectivity through the plane
    # Cache the blocked set and segment cells for plane_layer to reuse later
    blocked_plane: Optional[Set[Tuple[int, int]]] = None
    net_plane_segment_cells: Optional[Set[Tuple[int, int]]] = None
    inside_plane: Optional[Set[Tuple[int, int]]] = None

    bounds_grid = (min_gx, max_gx, min_gy, max_gy)
    for layer in routing_layers:
        # Get layer-specific clearance (fall back to default zone_clearance)
        layer_clearance = zone_clearance
        if zone_clearances and layer in zone_clearances:
            layer_clearance = zone_clearances[layer]

        # Build blocked set and net segment cells using helper function
        blocked, net_segment_cells = _build_layer_blocked_set(
            layer, net_id, pcb_data, coord, layer_clearance
        )
        # Zone copper only exists inside this net's own outlines (#217).
        inside_zone = (_zone_interior_cells(net_id, layer, pcb_data, coord,
                                            bounds_grid)
                       if layer in zone_layers else None)

        # Cache plane_layer data for reuse in anchor flood fill
        if layer == plane_layer:
            blocked_plane = blocked
            net_plane_segment_cells = net_segment_cells
            inside_plane = inside_zone

        # Find cross-layer points on this layer
        layer_cls: List[int] = []
        for i, (x, y, layers) in enumerate(cross_layer_points):
            if layer in layers:
                layer_cls.append(i)

        if not layer_cls:
            continue

        # Map grid position to cross-layer indices on this layer
        layer_grid_to_cl: Dict[Tuple[int, int], List[int]] = {}
        for i in layer_cls:
            x, y, _ = cross_layer_points[i]
            gp = coord.to_grid(x, y)
            if gp not in layer_grid_to_cl:
                layer_grid_to_cl[gp] = []
            layer_grid_to_cl[gp].append(i)

        # Flood fill from each cross-layer point to find which ones connect on this layer
        layer_visited: Set[Tuple[int, int]] = set()
        # Track parent pointers for path reconstruction (only if debug)
        layer_parent: Dict[Tuple[int, int], Tuple[int, int]] = {} if debug else {}

        for start_cl_idx in layer_cls:
            x, y, _ = cross_layer_points[start_cl_idx]
            start_gx, start_gy = coord.to_grid(x, y)

            if (start_gx, start_gy) in layer_visited:
                # Already visited by a previous flood fill - that flood fill
                # already unioned this point with any reachable cross-layer points.
                continue

            # Flood fill
            queue = deque([(start_gx, start_gy)])
            layer_visited.add((start_gx, start_gy))
            if debug:
                layer_parent[(start_gx, start_gy)] = (start_gx, start_gy)  # Self-parent for start

            while queue:
                gx, gy = queue.popleft()

                # Check if we reached other cross-layer points
                if (gx, gy) in layer_grid_to_cl:
                    for cl_idx in layer_grid_to_cl[(gx, gy)]:
                        # Before union, check if this connects previously disconnected components
                        if debug and cl_uf.find(start_cl_idx) != cl_uf.find(cl_idx):
                            # Reconstruct path from start to this point
                            path_points = []
                            curr = (gx, gy)
                            while curr != layer_parent.get(curr, curr) or curr == (start_gx, start_gy):
                                path_points.append(coord.to_float(curr[0], curr[1]))
                                parent = layer_parent.get(curr)
                                if parent is None or parent == curr:
                                    break
                                curr = parent
                            path_points.reverse()
                            if len(path_points) > 1:
                                debug_paths.append((path_points, layer))
                        cl_uf.union(start_cl_idx, cl_idx)

                # Expand to neighbors: 4-connected through zone copper, plus
                # diagonal steps ALONG the net's own segment cells -- segment
                # rasterization is a Bresenham line, so a 45-degree repair
                # route is a diagonal cell chain a 4-connected flood cannot
                # walk. The previous repair invocation's region-connection
                # routes were therefore invisible to the next invocation's
                # model, which re-routed the SAME connections onto identical
                # coordinates (castor_pollux: 293 stacked duplicate GND
                # segments). A diagonal into own-copper cells is physically
                # real copper continuity, never a corner-cut through a gap.
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0),
                               (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                    nx, ny = gx + dx, gy + dy
                    if dx != 0 and dy != 0 and (nx, ny) not in net_segment_cells:
                        continue
                    if (nx, ny) in layer_visited:
                        continue
                    if nx < min_gx or nx > max_gx or ny < min_gy or ny > max_gy:
                        continue
                    if layer in zone_layers:
                        # Layer has a zone: flood through unblocked cells INSIDE
                        # this net's own zone outlines (zone copper), or through
                        # same-net segments anywhere. Unrestricted bounds-wide
                        # flooding unioned separate outline islands through
                        # copper-free space (#217 castor_pollux +3.3V).
                        if ((nx, ny) in blocked
                                or (inside_zone is not None
                                    and (nx, ny) not in inside_zone)):
                            if (nx, ny) not in net_segment_cells:
                                continue
                    else:
                        # Layer has no zone: only traverse along same-net segments
                        if (nx, ny) not in net_segment_cells:
                            continue
                    layer_visited.add((nx, ny))
                    if debug:
                        layer_parent[(nx, ny)] = (gx, gy)
                    queue.append((nx, ny))

    # Now map anchor connectivity based on cross-layer point connectivity
    # Build map from anchor grid position to anchor indices
    grid_to_anchors: Dict[Tuple[int, int], List[int]] = {}
    for i, gp in enumerate(anchor_grid_points):
        if gp not in grid_to_anchors:
            grid_to_anchors[gp] = []
        grid_to_anchors[gp].append(i)

    # For each anchor, find which cross-layer point(s) it corresponds to
    anchor_to_cl: Dict[int, int] = {}
    for i, gp in enumerate(anchor_grid_points):
        if gp in grid_to_crosslayer:
            # Anchor is at a cross-layer point position
            anchor_to_cl[i] = grid_to_crosslayer[gp][0]

    # Union anchors that share the same cross-layer component
    for i in range(len(anchor_points)):
        for j in range(i + 1, len(anchor_points)):
            cl_i = anchor_to_cl.get(i)
            cl_j = anchor_to_cl.get(j)
            if cl_i is not None and cl_j is not None:
                if cl_uf.find(cl_i) == cl_uf.find(cl_j):
                    anchor_uf.union(i, j)

    # Flood fill on plane layer to connect anchors not at cross-layer points
    # (in case there are SMD pads or other anchors that aren't vias)
    # Use cached blocked_plane and net_plane_segment_cells from the layer loop above
    assert blocked_plane is not None, "plane_layer should have been processed in the loop"
    assert net_plane_segment_cells is not None, "plane_layer should have been processed in the loop"
    plane_visited: Set[Tuple[int, int]] = set()
    # Per-flood fill cells, keyed by the starting anchor index: these ARE the
    # region's modeled fill on the plane layer, and the join uses them as
    # pseudo-anchor material (castor +3.3VA: the region's only real anchor
    # sat 20mm from where its island nearly touches the neighbour).
    flood_cells_by_start: Dict[int, Set[Tuple[int, int]]] = {}
    for start_anchor_idx in range(len(anchor_points)):
        start_gx, start_gy = anchor_grid_points[start_anchor_idx]

        if (start_gx, start_gy) in plane_visited:
            # Already visited by a previous flood fill - skip this anchor.
            # The flood fill that visited this cell already handled union
            # with any reachable anchors.
            continue

        queue = deque([(start_gx, start_gy)])
        plane_visited.add((start_gx, start_gy))
        this_flood: Set[Tuple[int, int]] = {(start_gx, start_gy)}
        flood_cells_by_start[start_anchor_idx] = this_flood

        while queue:
            gx, gy = queue.popleft()
            this_flood.add((gx, gy))

            if (gx, gy) in grid_to_anchors:
                for anchor_idx in grid_to_anchors[(gx, gy)]:
                    anchor_uf.union(start_anchor_idx, anchor_idx)

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                nx, ny = gx + dx, gy + dy
                # Diagonals only along own segment copper (Bresenham chains),
                # same as the cross-layer flood above.
                if dx != 0 and dy != 0 and (nx, ny) not in net_plane_segment_cells:
                    continue
                if (nx, ny) in plane_visited:
                    continue
                if nx < min_gx or nx > max_gx or ny < min_gy or ny > max_gy:
                    continue
                # Same inside-own-outline gate as the cross-layer flood (#217).
                if ((nx, ny) in blocked_plane
                        or (inside_plane is not None
                            and (nx, ny) not in inside_plane)):
                    if (nx, ny) not in net_plane_segment_cells:
                        continue
                plane_visited.add((nx, ny))
                queue.append((nx, ny))

    # Group anchors by their root
    groups: Dict[int, List[int]] = {}
    for i in range(len(anchor_points)):
        root = anchor_uf.find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(i)

    # Build result lists
    region_anchors: List[List[Tuple[float, float]]] = []
    region_cells: List[Set[Tuple[int, int]]] = []

    for root, indices in groups.items():
        anchors = [anchor_points[i] for i in indices]
        # The region's modeled FILL cells: union of its member anchors' plane
        # floods (previously just the anchor grid points, which starved the
        # join of pseudo-anchor material).
        cells = set(anchor_grid_points[i] for i in indices)
        for i in indices:
            cells |= flood_cells_by_start.get(i, set())
        region_anchors.append(anchors)
        region_cells.append(cells)

    return region_anchors, region_cells, debug_paths


def _add_segment_cells(
    cells: Set[Tuple[int, int]],
    seg: Segment,
    coord: GridCoord
):
    """Add grid cells along a segment (no expansion, just the segment path)."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        cells.add((gx, gy))


def _block_segment_cells(
    blocked: Set[Tuple[int, int]],
    seg: Segment,
    coord: GridCoord,
    block_radius: int
):
    """Block grid cells along a segment with given radius."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    radius_sq = block_radius * block_radius

    for gx, gy in walk_line(gx1, gy1, gx2, gy2):
        for ex in range(-block_radius, block_radius + 1):
            for ey in range(-block_radius, block_radius + 1):
                if ex * ex + ey * ey <= radius_sq:
                    blocked.add((gx + ex, gy + ey))


def _block_pad_cells(
    blocked: Set[Tuple[int, int]],
    pad: Pad,
    coord: GridCoord,
    clearance: float
):
    """Block grid cells for a pad with clearance (honors pad rotation)."""
    half_w, half_h = pad_rect_halfspan(pad, clearance)

    min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
    max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
    _, min_gy = coord.to_grid(0, pad.global_y - half_h)
    _, max_gy = coord.to_grid(0, pad.global_y + half_h)

    rotated = bool(getattr(pad, 'rect_rotation', 0.0))
    for gx in range(min_gx, max_gx + 1):
        for gy in range(min_gy, max_gy + 1):
            if rotated:
                bx, by = coord.to_float(gx, gy)
                if not point_in_pad_rect(bx, by, pad, clearance):
                    continue
            blocked.add((gx, gy))


def _interior_cells(cells):
    """Cells eroded by one analysis cell: the flood model approximates the
    fill at coarse resolution, so interior cells are far likelier to carry
    real copper -- a pseudo-anchor on a boundary cell can land where the
    actual fill was carved away, emitting a join that connects to nothing.
    Erosion ladder: 8-neighbor interior, else 4-neighbor (thin strip
    islands at a coarse grid have no 8-interior at all), else nothing (the
    join falls back to plain anchors)."""
    full = []
    ortho = []
    for (gx, gy) in cells:
        ok4 = all((gx + dx, gy + dy) in cells
                  for dx, dy in ((0, 1), (0, -1), (1, 0), (-1, 0)))
        if not ok4:
            continue
        ortho.append((gx, gy))
        if all((gx + dx, gy + dy) in cells
               for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
            full.append((gx, gy))
    return full or ortho


def _subsample_cell_points(cells, coord, max_pts: int = 400):
    """INTERIOR region fill cells as float points, subsampled."""
    interior = _interior_cells(cells)
    pts = [coord.to_float(gx, gy) for gx, gy in interior]
    if len(pts) > max_pts:
        stride = len(pts) // max_pts + 1
        pts = pts[::stride]
    return pts


def _real_fill_point(pt, net_id, pcb_data, zone_polys, plane_layer,
                     margin: float) -> bool:
    from check_connected import point_in_polygon
    """True when a disc of radius `margin` around pt provably sits in REAL
    zone fill: fully inside one outline and at least `margin` clear of every
    foreign copper item on the plane layer (the coarse flood model blurs the
    fill's clearance carving; joins attached to modeled-but-absent fill ship
    floating vias -- kicad-cli 'Via | Zone unconnected')."""
    x, y = pt
    probes = ((x, y), (x + margin, y), (x - margin, y),
              (x, y + margin), (x, y - margin))
    if not any(all(point_in_polygon(px, py, poly) for px, py in probes)
               for poly in zone_polys):
        return False
    m2 = margin
    for v in pcb_data.vias:
        if v.net_id != net_id and \
                math.hypot(x - v.x, y - v.y) < v.size / 2 + m2:
            return False
    for s in pcb_data.segments:
        if s.net_id == net_id or s.layer != plane_layer:
            continue
        dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
        L2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - s.start_x) * dx + (y - s.start_y) * dy) / L2)) if L2 else 0.0
        if math.hypot(x - (s.start_x + t * dx), y - (s.start_y + t * dy)) < s.width / 2 + m2:
            return False
    for pads in pcb_data.pads_by_net.values():
        for p in pads:
            if p.net_id == net_id:
                continue
            if p.drill <= 0 and plane_layer not in p.layers and '*.Cu' not in p.layers:
                continue
            if math.hypot(x - p.global_x, y - p.global_y) < \
                    max(p.size_x, p.size_y) / 2 + m2:
                return False
    return True


def _nearest_cell_points(cells, coord, near_pt, k: int = 8,
                         validity=None):
    """The k INTERIOR region fill cells closest to near_pt, as float points.
    A deep fill cell is electrically the region (new copper starting there
    lands in the zone fill), so these serve as pseudo-anchors. `validity`
    (optional callable) filters candidates to provably-real fill points."""
    pts = _subsample_cell_points(cells, coord, max_pts=2000)
    pts.sort(key=lambda p: (p[0] - near_pt[0]) ** 2 + (p[1] - near_pt[1]) ** 2)
    if validity is None:
        return pts[:k]
    out = []
    for p in pts:
        if validity(p):
            out.append(p)
            if len(out) >= k:
                break
    return out


def find_region_connection_points(
    region_anchors: List[List[Tuple[float, float]]],
    region_cells: List[Set[Tuple[int, int]]],
    coord: GridCoord
) -> List[Tuple[int, int, Tuple[float, float], Tuple[float, float], float]]:
    """
    Find MST edges connecting disconnected regions using closest anchor points.

    Args:
        region_anchors: List of anchor point lists per region
        region_cells: List of grid cell sets per region (unused for now, anchors suffice)
        coord: Grid coordinate converter

    Returns:
        List of (region_i, region_j, point_i, point_j, distance) for MST edges
    """
    n_regions = len(region_anchors)
    if n_regions < 2:
        return []

    # Closest approach between regions is measured over the regions' FILL
    # CELLS (subsampled) plus their anchors -- not anchors alone. A region's
    # only anchor can sit 20mm from where its island nearly touches the
    # other region (castor +3.3VA: RV1's pad at (58.5,66.9) vs the 4mm hop
    # at (55,87) Andy bridged by hand); anchor-only selection then asks the
    # router for the long way through the dense middle and fails.
    region_pts: List[List[Tuple[float, float]]] = []
    for i in range(n_regions):
        pts = list(region_anchors[i])
        pts.extend(_subsample_cell_points(region_cells[i] if i < len(region_cells) else (), coord))
        region_pts.append(pts)

    edges: List[Tuple[float, int, int, Tuple[float, float], Tuple[float, float]]] = []

    for i in range(n_regions):
        for j in range(i + 1, n_regions):
            best_dist = float('inf')
            best_pi = None
            best_pj = None

            for pi in region_pts[i]:
                for pj in region_pts[j]:
                    dist = math.sqrt((pi[0] - pj[0])**2 + (pi[1] - pj[1])**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_pi = pi
                        best_pj = pj

            if best_pi and best_pj:
                edges.append((best_dist, i, j, best_pi, best_pj))

    # Sort by distance and build MST using Kruskal's algorithm
    edges.sort(key=lambda e: e[0])

    mst_uf = UnionFind()
    mst_edges: List[Tuple[int, int, Tuple[float, float], Tuple[float, float], float]] = []

    for dist, i, j, pi, pj in edges:
        if not mst_uf.connected(i, j):
            mst_uf.union(i, j)
            mst_edges.append((i, j, pi, pj, dist))
            if len(mst_edges) == n_regions - 1:
                break

    return mst_edges


def find_open_space_point(
    anchors: List[Tuple[float, float]],
    base_obstacles: GridObstacleMap,
    plane_layer_idx: int,
    coord: GridCoord,
    search_radius: float = 5.0,
    bounds: Optional[Tuple[float, float, float, float]] = None
) -> Optional[Tuple[float, float]]:
    """
    Find the most open space near a region's anchors - a point with maximum clearance from obstacles.

    Args:
        anchors: List of anchor points in the region
        base_obstacles: Obstacle map to check clearance against
        plane_layer_idx: Layer index to check for obstacles
        coord: Grid coordinate converter
        search_radius: How far from anchors to search (mm)

    Returns:
        (x, y) of the most open point, or None if no good point found
    """
    if not anchors:
        return None

    # Find centroid of anchors as search center
    cx = sum(a[0] for a in anchors) / len(anchors)
    cy = sum(a[1] for a in anchors) / len(anchors)

    search_radius_grid = coord.to_grid_dist(search_radius)
    center_gx, center_gy = coord.to_grid(cx, cy)

    best_clearance = 0
    best_point = None

    # Search in a grid around the centroid
    for dx in range(-search_radius_grid, search_radius_grid + 1):
        for dy in range(-search_radius_grid, search_radius_grid + 1):
            gx, gy = center_gx + dx, center_gy + dy

            # Skip cells outside the board (bounds = board area minus edge
            # clearance). Empty space beyond the outline scores maximum clearance,
            # so without this the "most open" point lands off-board, placing the
            # open-space via and its route outside the edge (issue #119).
            if bounds is not None:
                fx, fy = coord.to_float(gx, gy)
                if not (bounds[0] <= fx <= bounds[2] and bounds[1] <= fy <= bounds[3]):
                    continue

            # Skip if this cell is blocked
            if base_obstacles.is_blocked(gx, gy, plane_layer_idx):
                continue

            # Skip if via placement is blocked here
            if base_obstacles.is_via_blocked(gx, gy):
                continue

            # Calculate clearance - distance to nearest blocked cell
            clearance = _calculate_clearance(gx, gy, base_obstacles, plane_layer_idx, max_check=10)

            if clearance > best_clearance:
                best_clearance = clearance
                best_point = coord.to_float(gx, gy)

    # Only return if we found a point with meaningful clearance (at least 2 grid steps)
    if best_clearance >= 2:
        return best_point
    return None


def _calculate_clearance(
    gx: int, gy: int,
    obstacles: GridObstacleMap,
    layer_idx: int,
    max_check: int = 10
) -> int:
    """Calculate clearance from a grid cell to nearest obstacle."""
    for r in range(1, max_check + 1):
        # Check cells at distance r (manhattan approximation for speed)
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) == r or abs(dy) == r:  # Only check perimeter
                    if obstacles.is_blocked(gx + dx, gy + dy, layer_idx):
                        return r - 1
    return max_check


from terminal_colors import GREEN, RED, YELLOW, RESET


def _try_route_between_regions(
    anchors_i: List[Tuple[float, float]],
    anchors_j: List[Tuple[float, float]],
    base_obstacles: GridObstacleMap,
    plane_layer_idx: int,
    routing_layers: List[str],
    config: GridRouteConfig,
    net_vias: List[Tuple[float, float]],
    max_track_width: float,
    min_track_width: float,
    max_iterations: int,
    coord: GridCoord,
    verbose: bool = False,
    router: Optional[GridRouter] = None,
    bounds: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[Optional[Tuple[List[Tuple[float, float, str]], List[Tuple[float, float]]]], float, Optional[Tuple[float, float]]]:
    """
    Try to route between two regions, attempting multiple track widths.

    Tries track widths from max to min, then falls back to open-space routing.

    Args:
        anchors_i: Anchor points in source region
        anchors_j: Anchor points in target region
        base_obstacles: Obstacle map
        plane_layer_idx: Index of the plane layer
        routing_layers: All routing layer names
        config: Routing configuration
        net_vias: Existing via positions on this net
        max_track_width: Maximum track width to try
        min_track_width: Minimum track width to try
        max_iterations: Max routing iterations
        coord: Grid coordinate converter
        verbose: Print debug info

    Returns:
        Tuple of (route_result, track_width_used, open_space_via_if_used)
    """
    import time as _time
    _attempt_count = 0
    _total_route_time = 0.0
    _attempt_details = []

    def _try_route(src, tgt, margin, iters, label):
        """Helper to attempt one route and track stats."""
        nonlocal _attempt_count, _total_route_time
        _t0 = _time.time()
        result, used_iters = route_plane_connection_wide(
            src, tgt,
            plane_layer_idx=plane_layer_idx,
            routing_layers=routing_layers,
            base_obstacles=base_obstacles,
            config=config,
            net_vias=net_vias,
            track_margin=margin,
            max_iterations=iters,
            verbose=verbose,
            router=router
        )
        _dt = _time.time() - _t0
        _attempt_count += 1
        _total_route_time += _dt
        _attempt_details.append(f"{label} {'OK' if result else 'fail'} {_dt:.2f}s/{used_iters}it")
        return result, used_iters

    # Generate width steps: min, min*2, min*4, ..., up to max
    track_widths_narrow_first = [min_track_width]
    w = min_track_width * 2
    while w <= max_track_width:
        track_widths_narrow_first.append(w)
        w = w * 2
    if track_widths_narrow_first[-1] < max_track_width:
        track_widths_narrow_first.append(max_track_width)

    result = None
    track_width = min_track_width
    open_space_via = None
    base_iterations = 0  # iterations used by narrowest successful route

    # Route from the region with FEWER anchors as SOURCES. A multi-source A* with
    # many sources and few targets floods a broad, weakly-guided frontier and often
    # exhausts the whole budget WITHOUT finding a path, whereas few-sources ->
    # many-targets beelines: its heuristic (distance to the NEAREST of many targets)
    # is far more informative and the frontier stays focused. On daisho, routing the
    # 299-anchor GND region as ~128 source cells toward a 1-anchor region exhausted
    # 200k iterations and FAILED, while the reverse (few sources -> that big region
    # as target) found the same connection in ~2k -- and edges list the giant region
    # first, so the old fixed A->B order paid the 200k failure on ~every big-region
    # edge before the reverse succeeded. Ordering cheap-direction-first is purely a
    # search-efficiency choice: same obstacle map, same connectivity (#263).
    if len(anchors_i) <= len(anchors_j):
        lo_src, hi_tgt = anchors_i, anchors_j
    else:
        lo_src, hi_tgt = anchors_j, anchors_i

    # Step 1: Try minimum width first (both directions) with full budget
    result, base_iterations = _try_route(
        lo_src, hi_tgt, 0, max_iterations, f"w={min_track_width:.2f}mm few->many")

    if result is None:
        result, base_iterations = _try_route(
            hi_tgt, lo_src, 0, max_iterations, f"w={min_track_width:.2f}mm many->few")

    if result is None:
        # Can't route even at min width - try open-space fallback
        _t0 = _time.time()
        open_i = find_open_space_point(anchors_i, base_obstacles, plane_layer_idx, coord, bounds=bounds)
        open_j = find_open_space_point(anchors_j, base_obstacles, plane_layer_idx, coord, bounds=bounds)
        _dt_open = _time.time() - _t0
        _attempt_details.append(f"open-search {_dt_open:.2f}s")
        _total_route_time += _dt_open

        open_attempts = []
        if open_i:
            open_attempts.append((anchors_i + [open_i], anchors_j, open_i))
        if open_j:
            open_attempts.append((anchors_i, anchors_j + [open_j], open_j))
        if open_i and open_j:
            open_attempts.append((anchors_i + [open_i], anchors_j + [open_j], open_i))

        for src, tgt, via_candidate in open_attempts:
            result, base_iterations = _try_route(src, tgt, 0, max_iterations, "open")
            if result:
                route_pts = result[0]
                for pt in route_pts:
                    if abs(pt[0] - via_candidate[0]) < 0.01 and abs(pt[1] - via_candidate[1]) < 0.01:
                        open_space_via = via_candidate
                        break
                break

    # Step 2: If we found a route at min width, try wider widths with bounded budget
    if result is not None and len(track_widths_narrow_first) > 1:
        narrow_result = result
        # Wider width is an OPTIONAL upgrade over the already-successful narrow
        # route, so cap its budget at the standard max_iterations: a widening that
        # can't fit the corridor then fails fast instead of exhausting up to 3x a
        # large base. On daisho base_iterations reached ~200k, so the old 3x gave
        # ~600k-iteration FAILING searches per edge (2-8s each, x every edge that
        # routed narrow) -- a dominant slice of the multi-minute region-connect
        # phase (#263). Never below base_iterations, so a legitimate widening
        # through the same corridor (needs ~base) still succeeds; only the trace
        # WIDTH is affected on a miss, never whether the regions connect.
        iter_budget = min(max(base_iterations * 3, 1000), max_iterations)

        # Try wider widths (skip min_track_width which we already did)
        for try_width in track_widths_narrow_first[1:]:
            # Same +1 quantization guard as route.py's _track_margin_for_width
            # (issue #268): the obstacle stamp blocks cell CENTERS strictly inside
            # the keep-out radius, so the outermost blocked cell sits up to ~one
            # cell inside it; a bare ceil margin measured from that shell lets a
            # widened trunk land tens of um inside the NPTH/copper floor.
            track_margin = _track_margin_for_width(
                try_width, min_track_width, config.grid_step)

            wider, _ = _try_route(
                lo_src, hi_tgt, track_margin, iter_budget,
                f"w={try_width:.2f}mm few->many")

            if wider is None:
                wider, _ = _try_route(
                    hi_tgt, lo_src, track_margin, iter_budget,
                    f"w={try_width:.2f}mm many->few")

            if wider is not None:
                result = wider
                track_width = try_width
            else:
                break  # If this width fails, wider ones will too

    if verbose:
        print(f"({_attempt_count} attempts, {_total_route_time:.1f}s: {'; '.join(_attempt_details)}) ", end="", flush=True)

    return result, track_width, open_space_via


# Cap on how many anchors of a region are fed into the connection A* as
# source/target seeds. A giant plane region (e.g. daisho's GND Region 0 has
# 320 anchors) otherwise pushes thousands of source/target cells into EVERY A*
# attempt, across N track widths x N region pairs -- the dominant cost of
# route_disconnected_planes on dense boards. Regions are internally connected,
# so routing between the anchors nearest each region's connection point is
# optimal; the full anchor set is retried as a fallback if the reduced route
# fails, so no connection is lost.
ANCHOR_SEED_CAP = 16


def _nearest_anchors(anchors, point, k=ANCHOR_SEED_CAP):
    """The <=k anchors nearest `point`. Returns `anchors` unchanged when there
    are <= k of them (no behavior change for small regions) or `point` is None."""
    if point is None or len(anchors) <= k:
        return anchors
    px, py = point
    return sorted(anchors, key=lambda a: (a[0] - px) ** 2 + (a[1] - py) ** 2)[:k]


def route_disconnected_regions(
    net_id: int,
    net_name: str,
    plane_layer: str,
    zone_bounds: Tuple[float, float, float, float],
    pcb_data: PCBData,
    config: GridRouteConfig,
    base_obstacles: GridObstacleMap,
    layer_map: Dict[str, int],
    zone_clearance: float = 0.2,
    max_track_width: float = 2.0,
    min_track_width: float = 0.2,
    track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    hole_to_hole_clearance: float = 0.3,
    analysis_grid_step: float = 0.5,
    max_iterations: int = 200000,
    verbose: bool = False,
    zone_layers: Optional[Set[str]] = None,
    debug_connectivity: bool = False,
    zone_clearances: Optional[Dict[str, float]] = None
) -> Tuple[List[Dict], List[Dict], int, List[List[Tuple[float, float]]], List[Tuple[List[Tuple[float, float]], str]]]:
    """
    Detect and route between disconnected zone regions.

    Args:
        net_id: Net ID of the plane
        net_name: Net name for logging
        plane_layer: Layer of the plane
        zone_bounds: (min_x, min_y, max_x, max_y) of the zone area
        pcb_data: PCB data
        config: Routing configuration
        base_obstacles: Pre-built obstacle map (will be modified with new routes)
        layer_map: Dict mapping layer names to indices
        zone_clearance: Clearance for zone fill
        max_track_width: Maximum track width for connections (mm)
        min_track_width: Minimum track width for connections (mm)
        track_via_clearance: Clearance from track to other nets' vias
        max_iterations: Maximum A* iterations per route attempt
        verbose: Print debug info
        zone_layers: Layers that have zones for this net (for cross-layer connectivity)
        debug_connectivity: If True, return connectivity paths from flood fill analysis
        zone_clearances: Per-layer zone clearances (layer -> clearance)

    Returns:
        Tuple of (list of segment dicts, list of via dicts, number of routes added,
                  list of route paths for debug, list of connectivity paths (path, layer))
    """
    coord = GridCoord(config.grid_step)

    # Find disconnected regions (checking connectivity across all layers)
    routing_layers = list(layer_map.keys())
    region_anchors, region_cells, connectivity_paths = find_disconnected_zone_regions(
        net_id, plane_layer, zone_bounds, pcb_data, config, zone_clearance,
        analysis_grid_step, routing_layers, zone_layers, debug_connectivity,
        zone_clearances=zone_clearances
    )

    n_regions = len(region_anchors)
    if n_regions < 2:
        n_anchors = len(region_anchors[0]) if region_anchors else 0
        print(f"  Zone is fully connected ({n_anchors} anchors in 1 region)")
        return [], [], 0, [], connectivity_paths

    # Count total anchors per region for display
    total_anchors = sum(len(anchors) for anchors in region_anchors)
    print(f"  Found {n_regions} disconnected regions ({total_anchors} total anchors)")
    if verbose:
        for i, anchors in enumerate(region_anchors):
            print(f"    Region {i}: {len(anchors)} anchor(s)")

    # Find MST edges to connect regions
    mst_edges = find_region_connection_points(region_anchors, region_cells, coord)
    print(f"  Routing {len(mst_edges)} connection(s) to join regions...")

    # Get plane layer index and routing layers from layer_map
    plane_layer_idx = layer_map.get(plane_layer)
    if plane_layer_idx is None:
        print(f"  Error: plane_layer '{plane_layer}' not in layer_map")
        return [], [], 0, []
    routing_layers = list(layer_map.keys())

    # Build list of existing vias and through-hole pads from this net (can be reused as layer transitions)
    net_vias: List[Tuple[float, float]] = [(v.x, v.y) for v in pcb_data.vias if v.net_id == net_id]
    # Add through-hole pads from this net (they connect all layers like vias)
    if net_id in pcb_data.pads_by_net:
        for pad in pcb_data.pads_by_net[net_id]:
            if '*.Cu' in pad.layers:  # Through-hole pad
                net_vias.append((pad.global_x, pad.global_y))

    segments: List[Dict] = []
    vias: List[Dict] = []
    routes_added = 0
    routes_failed = 0
    previous_routes: List[List[Tuple[float, float]]] = []

    # Create a single reusable router for all MST edges
    narrow_obstacles = None       # lazily-built map stamped at config.track_width
    routes_added_replay = []      # connections added this loop, replayed onto it
    plane_router = GridRouter(
        via_cost=config.via_cost_units(),
        h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost,
        via_proximity_cost=0,
        layer_costs=config.get_layer_costs(),
        proximity_heuristic_cost=config.get_proximity_heuristic_cost()
    )

    for edge_idx, (region_i, region_j, point_i, point_j, dist) in enumerate(mst_edges):
        # Get all anchors from each region for multi-point routing
        anchors_i = region_anchors[region_i]
        anchors_j = region_anchors[region_j]

        # Seed the A* only from the anchors nearest each region's connection
        # point (fix: giant regions otherwise feed thousands of seeds per attempt).
        seed_i = _nearest_anchors(anchors_i, point_i)
        seed_j = _nearest_anchors(anchors_j, point_j)
        # Pseudo-anchors on the fill nearest the closest approach: a new via
        # anywhere on a region's fill IS the region (castor +3.3VA -- the
        # human's bridge started at a bare fill spot 20mm from the anchor).
        _zone_polys = [z.polygon for z in (getattr(pcb_data, 'zones', []) or [])
                       if z.net_id == net_id and getattr(z, 'polygon', None)]
        _plane_layer_name = [l for l, i in layer_map.items()
                             if i == plane_layer_idx][0]
        _margin = zone_clearance

        def _valid_fill(pt):
            return _real_fill_point(pt, net_id, pcb_data, _zone_polys,
                                    _plane_layer_name, _margin)

        cells_i = _nearest_cell_points(
            region_cells[region_i] if region_i < len(region_cells) else (),
            coord, point_i, validity=_valid_fill)
        cells_j = _nearest_cell_points(
            region_cells[region_j] if region_j < len(region_cells) else (),
            coord, point_j, validity=_valid_fill)
        seed_i = seed_i + [p for p in cells_i if p not in seed_i]
        seed_j = seed_j + [p for p in cells_j if p not in seed_j]
        full_i = anchors_i + [p for p in cells_i if p not in anchors_i]
        full_j = anchors_j + [p for p in cells_j if p not in anchors_j]
        reduced = (len(seed_i) < len(full_i)) or (len(seed_j) < len(full_j))

        # Progress indicator
        seed_note = f" (seed {len(seed_i)}x{len(seed_j)})" if reduced else ""
        print(f"    [{edge_idx+1}/{len(mst_edges)}] Region {region_i} ({len(anchors_i)} anchors) <-> Region {region_j} ({len(anchors_j)} anchors){seed_note}...", end=" ", flush=True)

        def _connect(a_i, a_j):
            return _try_route_between_regions(
                a_i, a_j,
                base_obstacles=base_obstacles,
                plane_layer_idx=plane_layer_idx,
                routing_layers=routing_layers,
                config=config,
                bounds=zone_bounds,
                net_vias=net_vias,
                max_track_width=max_track_width,
                min_track_width=min_track_width,
                max_iterations=max_iterations,
                coord=coord,
                verbose=verbose,
                router=plane_router
            )

        # Try routing with multiple track widths using helper function
        result, track_width, open_space_via = _connect(seed_i, seed_j)

        # Fallback: if the reduced-anchor route failed, retry with the full
        # region anchor sets (the original, slower behavior) so we never miss a
        # connection that the full set would have found.
        if result is None and reduced:
            print(f"{YELLOW}retry-full{RESET} ", end="", flush=True)
            result, track_width, open_space_via = _connect(full_i, full_j)

        # Last resort (#217 castor +3.3VA): the corridor between two regions
        # can be narrower than REPAIR_MIN_TRACK_WIDTH while still fitting the
        # run's own signal --track-width (Andy hand-bridged the failing edge
        # at exactly 0.127). The base map's keep-outs are stamped at
        # min_track_width, so a narrower attempt needs a map stamped at the
        # narrow width -- built lazily once per net and kept in sync with the
        # connections already added this loop. A narrow bridge beats a split
        # plane.
        if result is None and config.track_width < min_track_width - 1e-9:
            if narrow_obstacles is None:
                print(f"{YELLOW}narrow-map{RESET} ", end="", flush=True)
                narrow_obstacles, _ = build_base_obstacles(
                    exclude_net_ids={net_id},
                    routing_layers=routing_layers,
                    pcb_data=pcb_data,
                    config=config,
                    track_width=config.track_width,
                    track_via_clearance=track_via_clearance,
                    hole_to_hole_clearance=hole_to_hole_clearance)
                for (_rp, _vp, _tw) in routes_added_replay:
                    add_route_to_obstacles(
                        narrow_obstacles, _rp, _vp, layer_map, _tw,
                        config.clearance, track_via_clearance, config,
                        hole_to_hole_clearance)
            print(f"{YELLOW}narrow-width {config.track_width:.3f}{RESET} ",
                  end="", flush=True)
            result, track_width, open_space_via = _try_route_between_regions(
                full_i, full_j,
                base_obstacles=narrow_obstacles,
                plane_layer_idx=plane_layer_idx,
                routing_layers=routing_layers,
                config=config,
                bounds=zone_bounds,
                net_vias=net_vias,
                max_track_width=config.track_width,
                min_track_width=config.track_width,
                max_iterations=max_iterations,
                coord=coord,
                verbose=verbose,
                router=plane_router
            )

        if result is None:
            print(f"{RED}FAILED{RESET}")
            routes_failed += 1
            if verbose:
                print(f"      Tried {len(anchors_i)}x{len(anchors_j)} + {len(anchors_j)}x{len(anchors_i)} + open-space combinations, no path found")
            continue

        route_points, via_positions = result

        # Calculate actual route length
        route_length = 0.0
        for k in range(len(route_points) - 1):
            p1, p2 = route_points[k], route_points[k + 1]
            route_length += math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

        # Count layers used
        layers_used = set(p[2] for p in route_points)
        layer_info = f", {len(layers_used)} layer(s)" if len(layers_used) > 1 else ""
        total_vias = len(via_positions) + (1 if open_space_via else 0)
        via_info = f", {total_vias} via(s)" if total_vias > 0 else ""
        open_info = " (via open-space)" if open_space_via else ""

        print(f"{GREEN}OK{RESET} width={track_width:.2f}mm, length={route_length:.1f}mm, {len(route_points)-1} seg(s){layer_info}{via_info}{open_info}")

        # Incrementally add this route to base obstacles for subsequent routes
        add_route_to_obstacles(
            base_obstacles, route_points, via_positions, layer_map,
            track_width, config.clearance, track_via_clearance, config,
            hole_to_hole_clearance
        )
        routes_added_replay.append((route_points, via_positions, track_width))
        if narrow_obstacles is not None:
            add_route_to_obstacles(
                narrow_obstacles, route_points, via_positions, layer_map,
                track_width, config.clearance, track_via_clearance, config,
                hole_to_hole_clearance
            )

        # Keep track of route for debug output
        previous_routes.append([(p[0], p[1]) for p in route_points])

        # Generate segments from route points (with layer info)
        for k in range(len(route_points) - 1):
            p1, p2 = route_points[k], route_points[k + 1]
            # Only create segment if on same layer
            if p1[2] == p2[2]:
                segments.append({
                    'start': (p1[0], p1[1]),
                    'end': (p2[0], p2[1]),
                    'width': track_width,
                    'layer': p1[2],
                    'net_id': net_id
                })

        # Generate vias and add to net_vias for reuse by subsequent routes
        # Check against existing net_vias for both duplicates AND hole-to-hole clearance
        # Required minimum distance between via centers = via_drill + hole_to_hole_clearance
        min_via_distance = config.via_drill + hole_to_hole_clearance

        # First filter via_positions to remove vias too close to each other within this route
        filtered_via_positions = []
        for vx, vy in via_positions:
            too_close_to_filtered = any(
                math.sqrt((fx - vx)**2 + (fy - vy)**2) < min_via_distance
                for fx, fy in filtered_via_positions
            )
            if not too_close_to_filtered:
                filtered_via_positions.append((vx, vy))

        for vx, vy in filtered_via_positions:
            too_close = any(
                math.sqrt((ex - vx)**2 + (ey - vy)**2) < min_via_distance
                for ex, ey in net_vias
            )
            if not too_close:
                vias.append({
                    'x': vx,
                    'y': vy,
                    'size': config.via_size,
                    'drill': config.via_drill,
                    'net_id': net_id
                })
                net_vias.append((vx, vy))  # Available for reuse

        # Add open-space via if one was used and not too close to existing vias
        if open_space_via:
            too_close = any(
                math.sqrt((ex - open_space_via[0])**2 + (ey - open_space_via[1])**2) < min_via_distance
                for ex, ey in net_vias
            )
            if not too_close:
                vias.append({
                    'x': open_space_via[0],
                    'y': open_space_via[1],
                    'size': config.via_size,
                    'drill': config.via_drill,
                    'net_id': net_id
                })
                net_vias.append(open_space_via)

        routes_added += 1

    # Summary for this net
    if routes_failed > 0:
        print(f"  {YELLOW}Result: {routes_added}/{len(mst_edges)} routes succeeded, {routes_failed} failed{RESET}")
    elif routes_added > 0:
        print(f"  {GREEN}Result: All {routes_added} route(s) succeeded{RESET}")

    return segments, vias, routes_added, previous_routes, connectivity_paths


def build_base_obstacles(
    exclude_net_ids: Set[int],
    routing_layers: List[str],
    pcb_data: PCBData,
    config: GridRouteConfig,
    track_width: float,
    track_via_clearance: float,
    hole_to_hole_clearance: float = 0.3,
    proximity_radius: float = 3.0,
    proximity_cost: float = 2.0
) -> Tuple[GridObstacleMap, Dict[str, int]]:
    """
    Build base obstacle map with all static obstacles, excluding specified nets.

    Args:
        exclude_net_ids: Set of net IDs to exclude from obstacles (the plane nets being routed)
        routing_layers: List of layer names available for routing
        pcb_data: PCB data
        config: Routing configuration
        track_width: Track width for clearance calculations
        track_via_clearance: Clearance from tracks to vias
        hole_to_hole_clearance: Minimum clearance between drill holes (mm)
        proximity_radius: Radius for proximity costs
        proximity_cost: Cost multiplier for proximity

    Returns:
        Tuple of (obstacle_map, layer_map)
    """
    coord = GridCoord(config.grid_step)
    # Half-cell discretization cushion (matches build_via_obstacle_map /
    # build_routing_obstacle_map): the keep-out cells are grid-snapped, but the
    # router's own connection/merge geometry can land OFF grid between cells, so
    # without this margin an off-grid connector segment grazes foreign copper
    # sub-cell (issue #173). Far smaller than the old flat 0.8mm over-block.
    cushion = config.grid_step / 2

    num_layers = len(routing_layers)
    layer_map = {layer: idx for idx, layer in enumerate(routing_layers)}

    obstacles = GridObstacleMap(num_layers)

    # Block other nets' vias (issue #173 parity with route.py): a foreign via
    # blocks TRACK routing within foreign_via_radius + our_track_half + clearance
    # on every layer, and new-VIA placement within foreign_via_radius +
    # our_via_radius + clearance -- both scaling with the foreign via's ACTUAL
    # size. The old code used a single flat track_via_clearance constant (0.8mm)
    # for all vias, which over-blocked typical vias, ignored their real size, and
    # added no copper via-via placement clearance. our_track_half uses track_width
    # (== min connection width): the region router routes at the min width against
    # this base map and adds its own extra margin when it widens.
    foreign_centers_by_size: Dict[float, List[Tuple[int, int]]] = {}
    for via in pcb_data.vias:
        if via.net_id in exclude_net_ids:
            continue
        foreign_centers_by_size.setdefault(via.size, []).append(coord.to_grid(via.x, via.y))
    for vsize, centers in foreign_centers_by_size.items():
        track_keepout_mm = vsize / 2 + track_width / 2 + config.clearance + cushion
        track_offs = _precompute_circle_offsets((track_keepout_mm / coord.grid_step) ** 2)
        for layer_idx in range(num_layers):
            _batch_block_circles_cell(obstacles, centers, track_offs, layer_idx)
        via_keepout_mm = vsize / 2 + config.via_size / 2 + config.clearance + cushion
        via_offs = _precompute_circle_offsets((via_keepout_mm / coord.grid_step) ** 2)
        _batch_block_circles_via(obstacles, centers, via_offs)

    # Add proximity costs around other nets' vias (on all layers)
    proximity_radius_grid = coord.to_grid_dist(proximity_radius)
    proximity_cost_grid = config.cell_cost(proximity_cost)

    all_other_vias = []
    for via in pcb_data.vias:
        if via.net_id not in exclude_net_ids:
            gx, gy = coord.to_grid(via.x, via.y)
            all_other_vias.append((gx, gy))

    if all_other_vias:
        obstacles.add_stub_proximity_costs_batch(
            all_other_vias,
            proximity_radius_grid,
            proximity_cost_grid,
            False
        )

    # Block via placement near ALL vias (including same-net) for hole-to-hole
    # clearance, using each EXISTING via's actual drill: required centre distance
    # is existing_drill/2 + new_drill/2 + h2h, not the equal-drill shortcut
    # h2h + config.via_drill, which let a 0.15-drill repair via land 0.354mm from
    # a 0.3-drill stitching via (needs 0.425 -- issue #274).
    # Via reuse is handled separately by snapping routes to existing vias
    via_centers_by_radius: Dict[int, List[Tuple[int, int]]] = {}
    for via in pcb_data.vias:
        vdrill = via.drill if via.drill and via.drill > 0 else config.via_drill
        r = max(1, coord.to_grid_dist_safe(
            hole_to_hole_clearance + vdrill / 2 + config.via_drill / 2))
        via_centers_by_radius.setdefault(r, []).append(coord.to_grid(via.x, via.y))
    for radius, centers in via_centers_by_radius.items():
        hole_circle_offsets = _precompute_circle_offsets(radius * radius)
        _batch_block_circles_via(obstacles, centers, hole_circle_offsets)

    # Block via placement near ALL through-hole pad holes (including same-net)
    # Via reuse at pads is handled separately by snapping routes to pad positions
    # Group pads by clearance radius for batch processing
    pad_centers_by_radius: Dict[int, List[Tuple[int, int]]] = {}
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            # Any drilled pad: PTH barrels AND NPTH mounting holes (which often
            # list only *.Mask) -- the drill goes through every layer (#268).
            # A milled SLOT drill is a capsule, not a round hole: sample it as
            # circles along its axis (pad_drill_circles) so the whole slot is
            # kept clear, not just a circle at its centre. Round drills yield one
            # circle == the old behaviour.
            if pad.drill > 0:
                for hx, hy, hdia in pad_drill_circles(pad):
                    gx, gy = coord.to_grid(hx, hy)
                    r_grid = max(1, coord.to_grid_dist_safe(
                        hole_to_hole_clearance + hdia / 2 + config.via_drill / 2))
                    pad_centers_by_radius.setdefault(r_grid, []).append((gx, gy))

    for radius, centers in pad_centers_by_radius.items():
        pad_circle_offsets = _precompute_circle_offsets(radius * radius)
        _batch_block_circles_via(obstacles, centers, pad_circle_offsets)

    # Block existing segments from other nets on each layer
    # Also block vias along segments (vias span all layers, so can't place via where segment exists)
    for seg in pcb_data.segments:
        if seg.net_id in exclude_net_ids:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        seg_expansion_mm = track_width / 2 + seg.width / 2 + config.clearance + cushion
        _block_segment_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_mm)
        # Also block vias along this segment - must include segment width for proper clearance
        via_seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance + cushion
        _block_segment_via_obstacle(obstacles, seg, coord, via_seg_expansion_mm)

    # Block pads from non-plane nets on their respective layers
    # (plane net pads are excluded here - they're anchors; other plane nets' pads
    # will be blocked per-net in route_disconnected_regions)
    npth_holes = []  # (x, y, drill) of no-copper holes -> NPTH track keep-out
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id in exclude_net_ids:
            continue
        for pad in pads:
            # NPTH mounting holes have NO copper (even when the library lists
            # *.Cu): the pad-rect block below would only enforce the routing
            # clearance, but copper must stay the NPTH-to-track fab floor from
            # the DRILL (issue #268) -- handled by the drill keep-out after
            # this loop instead.
            if pad.drill > 0 and not _pad_has_copper(pad):
                npth_holes.append((pad.global_x, pad.global_y, pad.drill))
                continue
            # Determine which layers this pad is on
            pad_layers_on = []
            if '*.Cu' in pad.layers:
                pad_layers_on = list(range(num_layers))
            else:
                for pl in pad.layers:
                    if pl in layer_map:
                        pad_layers_on.append(layer_map[pl])

            if not pad_layers_on:
                continue

            # Block rectangular area around pad with clearance for track routing.
            # Honor a per-pad local clearance override (fiducial keep-clear etc.).
            pad_clr = max(config.clearance, getattr(pad, 'local_clearance', 0.0) or 0.0)
            pad_expansion_mm = track_width / 2 + pad_clr + cushion
            half_w, half_h = pad_rect_halfspan(pad, pad_expansion_mm)
            min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
            max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
            _, min_gy = coord.to_grid(0, pad.global_y - half_h)
            _, max_gy = coord.to_grid(0, pad.global_y + half_h)
            gx_range = np.arange(min_gx, max_gx + 1, dtype=np.int32)
            gy_range = np.arange(min_gy, max_gy + 1, dtype=np.int32)
            if gx_range.size > 0 and gy_range.size > 0:
                gx_grid, gy_grid = np.meshgrid(gx_range, gy_range)
                rect_cells = np.column_stack([gx_grid.ravel(), gy_grid.ravel()])
                rect_cells = filter_cells_in_pad_rect(rect_cells, coord.grid_step, pad, pad_expansion_mm)
                for layer_idx in pad_layers_on:
                    layer_col = np.full((rect_cells.shape[0], 1), layer_idx, dtype=np.int32)
                    obstacles.add_blocked_cells_batch(np.hstack([rect_cells, layer_col]))
            # Also block vias around pad - new via radius + clearance from pad edge
            # (issue #173 parity: was config.via_size/2 + the flat 0.8 constant).
            via_expansion_mm = config.via_size / 2 + config.clearance + cushion
            via_half_w, via_half_h = pad_rect_halfspan(pad, via_expansion_mm)
            via_min_gx, _ = coord.to_grid(pad.global_x - via_half_w, 0)
            via_max_gx, _ = coord.to_grid(pad.global_x + via_half_w, 0)
            _, via_min_gy = coord.to_grid(0, pad.global_y - via_half_h)
            _, via_max_gy = coord.to_grid(0, pad.global_y + via_half_h)
            via_gx_range = np.arange(via_min_gx, via_max_gx + 1, dtype=np.int32)
            via_gy_range = np.arange(via_min_gy, via_max_gy + 1, dtype=np.int32)
            if via_gx_range.size > 0 and via_gy_range.size > 0:
                via_gx_grid, via_gy_grid = np.meshgrid(via_gx_range, via_gy_range)
                via_rect_cells = np.column_stack([via_gx_grid.ravel(), via_gy_grid.ravel()])
                via_rect_cells = filter_cells_in_pad_rect(via_rect_cells, coord.grid_step, pad, via_expansion_mm)
                obstacles.add_blocked_vias_batch(via_rect_cells)

    # Track keep-out around NPTH (no-copper) drills on every layer at exactly
    # the NPTH-to-track fab floor (issues #268/#233). No cushion: the floor is
    # the hard fab requirement and cell centers at >= the radius stay free, so
    # this blocks the minimum area that avoids real copper-to-hole violations.
    if npth_holes:
        npth_clr = max(config.clearance, defaults.NPTH_TO_TRACK_CLEARANCE)
        block_track_cells_near_drills(obstacles, npth_holes, track_width,
                                      npth_clr, config.grid_step,
                                      list(range(num_layers)))

    # Block areas outside the board outline (supports non-rectangular boards)
    add_board_edge_obstacles(obstacles, pcb_data, config, 0.0, layers=routing_layers)

    # Keep plane-repair routes out of user-drawn keepouts (#27) and KiCad
    # keep-out rule areas (#25), on the plane routing layers.
    add_user_keepout_obstacles(obstacles, pcb_data, config, coord, num_layers)
    add_rule_area_keepout_obstacles(obstacles, pcb_data, config, layers=routing_layers)

    return obstacles, layer_map


def add_route_to_obstacles(
    obstacles: GridObstacleMap,
    route_points: List[Tuple[float, float, str]],
    via_positions: List[Tuple[float, float]],
    layer_map: Dict[str, int],
    track_width: float,
    clearance: float,
    via_clearance: float,
    config: GridRouteConfig,
    hole_to_hole_clearance: float = 0.3
):
    """Add a completed route (segments and vias) to the obstacle map for subsequent routes to avoid."""
    coord = GridCoord(config.grid_step)
    num_layers = len(layer_map)
    cushion = config.grid_step / 2  # half-cell discretization margin (issue #173)

    # Block segments on their layers and also block vias along segments.
    # Exact capsule keep-out from the TRUE float route points (issue #173),
    # shared with route.py's obstacle builder -- no grid snapping of the
    # newly-routed connection geometry, which is exactly the off-grid copper
    # that the old bresenham stamp under-covered sub-cell.
    route_expansion_mm = track_width + clearance + cushion
    via_seg_expansion_mm = config.via_size / 2 + track_width / 2 + clearance + cushion

    for i in range(len(route_points) - 1):
        p1, p2 = route_points[i], route_points[i + 1]
        # Only block if on same layer (segments)
        if p1[2] == p2[2]:
            layer_idx = layer_map.get(p1[2], 0)
            cells = segment_blocked_cells_array(p1[0], p1[1], p2[0], p2[1],
                                                route_expansion_mm, coord.grid_step)
            _batch_cells_one_layer(obstacles, cells, layer_idx)
            # Also block vias along this segment
            vias = segment_blocked_cells_array(p1[0], p1[1], p2[0], p2[1],
                                               via_seg_expansion_mm, coord.grid_step)
            _batch_vias(obstacles, vias)

    # Newly-placed connection vias (all config.via_size) block TRACK routing at
    # via_size/2 + our_track_half + clearance, new-VIA placement at copper
    # clearance (via_size + clearance), and drill placement at hole-to-hole.
    # (issue #173 parity: was a flat via_clearance constant for track + hole-to-
    # hole only, with no copper via-via placement clearance.)
    track_keepout_mm = config.via_size / 2 + track_width / 2 + clearance + cushion
    via_copper_keepout_mm = config.via_size + clearance + cushion
    hole_clearance_radius = coord.to_grid_dist_safe(hole_to_hole_clearance + config.via_drill)

    if via_positions:
        via_grid_centers = [coord.to_grid(vx, vy) for vx, vy in via_positions]
        # Block track routing around vias on all layers
        via_cell_offsets = _precompute_circle_offsets((track_keepout_mm / coord.grid_step) ** 2)
        for layer_idx in range(num_layers):
            _batch_block_circles_cell(obstacles, via_grid_centers, via_cell_offsets, layer_idx)
        # Block new-via placement: copper clearance AND drill hole-to-hole.
        via_copper_offsets = _precompute_circle_offsets((via_copper_keepout_mm / coord.grid_step) ** 2)
        _batch_block_circles_via(obstacles, via_grid_centers, via_copper_offsets)
        hole_offsets = _precompute_circle_offsets(hole_clearance_radius * hole_clearance_radius)
        _batch_block_circles_via(obstacles, via_grid_centers, hole_offsets)


def route_plane_connection_wide(
    source_points: List[Tuple[float, float]],
    target_points: List[Tuple[float, float]],
    plane_layer_idx: int,
    routing_layers: List[str],
    base_obstacles: GridObstacleMap,
    config: GridRouteConfig,
    net_vias: List[Tuple[float, float]],
    track_margin: int = 0,
    max_iterations: int = 200000,
    verbose: bool = False,
    router: Optional[GridRouter] = None
) -> Optional[Tuple[List[Tuple[float, float, str]], List[Tuple[float, float]]]]:
    """
    Route a wide trace between any source point and any target point.

    The router will find the best path from ANY source to ANY target,
    allowing flexible connection between disconnected regions.

    Args:
        source_points: List of (x, y) anchor points in source region
        target_points: List of (x, y) anchor points in target region
        plane_layer_idx: Index of the plane layer in routing_layers
        routing_layers: List of layer names for routing
        base_obstacles: Pre-built obstacle map (will be cloned)
        config: Routing configuration
        net_vias: List of existing via positions from this net (to avoid duplicates)
        track_margin: Extra margin in grid cells for wide tracks
        max_iterations: Max routing iterations
        verbose: Print debug info

    Returns:
        Tuple of:
        - List of (x, y, layer) tuples for route segments
        - List of (x, y) tuples for NEW via positions (excluding reused ones)
        Or None if routing failed.
    """
    coord = GridCoord(config.grid_step)

    # Clone the base obstacles for this route (clone_fresh clears source/target cells)
    obstacles = base_obstacles.clone_fresh()
    n_layers = len(routing_layers)

    # Build set of via positions for quick lookup
    via_positions = set((round(vx, POSITION_DECIMALS), round(vy, POSITION_DECIMALS)) for vx, vy in net_vias)

    def is_at_via(x: float, y: float) -> bool:
        """Check if a point is at a via location (connects all layers)."""
        return (round(x, POSITION_DECIMALS), round(y, POSITION_DECIMALS)) in via_positions

    # Set up sources - all anchor points from source region
    # For vias (which connect all layers), set source on ALL layers
    sources = []
    for sx, sy in source_points:
        gx, gy = coord.to_grid(sx, sy)
        if is_at_via(sx, sy):
            # Via connects all layers - set source on all layers
            for layer_idx in range(n_layers):
                obstacles.add_source_target_cell(gx, gy, layer_idx)
                sources.append((gx, gy, layer_idx))
        else:
            # SMD pad - only on plane layer
            obstacles.add_source_target_cell(gx, gy, plane_layer_idx)
            sources.append((gx, gy, plane_layer_idx))

    # Set up targets - all anchor points from target region
    targets = []
    for tx, ty in target_points:
        gx, gy = coord.to_grid(tx, ty)
        if is_at_via(tx, ty):
            # Via connects all layers - set target on all layers
            for layer_idx in range(n_layers):
                obstacles.add_source_target_cell(gx, gy, layer_idx)
                targets.append((gx, gy, layer_idx))
        else:
            # SMD pad - only on plane layer
            obstacles.add_source_target_cell(gx, gy, plane_layer_idx)
            targets.append((gx, gy, plane_layer_idx))

    # Create router if not provided
    if router is None:
        router = GridRouter(
            via_cost=config.via_cost_units(),
            h_weight=config.heuristic_weight,
            turn_cost=config.turn_cost,
            via_proximity_cost=0,
            layer_costs=config.get_layer_costs(),
            proximity_heuristic_cost=config.get_proximity_heuristic_cost()
        )

    path, iterations, _ = router.route_with_frontier(
        obstacles, sources, targets, max_iterations,
        False,  # collinear_vias
        0,      # via_exclusion_radius
        None,   # start_direction
        None,   # end_direction
        0,      # direction_steps
        track_margin  # extra margin for wide tracks
    )

    if path is None:
        if verbose:
            print(f"    Route failed after {iterations} iterations")
        return None, iterations

    if verbose:
        print(f"    Route found in {iterations} iterations, {len(path)} points")

    # Convert path to float coordinates with layer info
    route_points: List[Tuple[float, float, str]] = []
    for i, (gx, gy, layer_idx) in enumerate(path):
        x, y = coord.to_float(gx, gy)
        layer_name = routing_layers[layer_idx]
        route_points.append((x, y, layer_name))

    # Find layer transitions and add vias where needed
    # Routes can now start/end at existing vias on any layer, so we only need to add
    # new vias where the route transitions between layers at a new location
    new_via_positions: List[Tuple[float, float]] = []
    added_via_keys: Set[Tuple[float, float]] = set()

    for i in range(1, len(route_points)):
        if route_points[i][2] != route_points[i-1][2]:
            # Layer transition - check if we need a new via
            via_x, via_y = route_points[i - 1][0], route_points[i - 1][1]
            via_key = (round(via_x, POSITION_DECIMALS), round(via_y, POSITION_DECIMALS))

            # Only add a new via if there isn't already one at this position
            if via_key not in via_positions and via_key not in added_via_keys:
                new_via_positions.append((via_x, via_y))
                added_via_keys.add(via_key)

    # Remove duplicate consecutive points (same x,y) keeping layer transitions
    cleaned_points: List[Tuple[float, float, str]] = []
    for pt in route_points:
        if cleaned_points:
            last = cleaned_points[-1]
            if pt[0] == last[0] and pt[1] == last[1] and pt[2] == last[2]:
                continue  # Skip duplicate
        cleaned_points.append(pt)

    return (cleaned_points, new_via_positions), iterations


def _block_segment_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    layer_idx: int,
    expansion_mm: float
):
    """Block cells along a segment: exact point-to-segment (capsule) keep-out
    from the TRUE float segment, shared with route.py's obstacle builder (issue
    #173). The previous bresenham stamp snapped the endpoints to the grid and
    under-covered off-grid/diagonal copper sub-cell."""
    cells = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                        seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
    _batch_cells_one_layer(obstacles, cells, layer_idx)


def _block_segment_via_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    expansion_mm: float
):
    """Block via placement along a segment (vias span all layers): exact capsule
    keep-out from the TRUE float segment (issue #173)."""
    vias = segment_blocked_cells_array(seg.start_x, seg.start_y,
                                       seg.end_x, seg.end_y, expansion_mm, coord.grid_step)
    _batch_vias(obstacles, vias)

"""
Plane Region Connector - Detects and routes between disconnected plane regions.

After power planes are created, regions may be effectively split due to vias and
traces from other nets cutting through the plane. This module detects disconnected
regions and routes wide, short tracks between them to ensure electrical continuity.
"""

from typing import List, Optional, Tuple, Dict, Set
from collections import deque
import math

from kicad_parser import PCBData, Via, Segment, Pad, POSITION_DECIMALS
from routing_config import GridRouteConfig, GridCoord
from geometry_utils import UnionFind

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'rust_router'))
from grid_router import GridObstacleMap, GridRouter


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

    for layer in routing_layers:
        # Get layer-specific clearance (fall back to default zone_clearance)
        layer_clearance = zone_clearance
        if zone_clearances and layer in zone_clearances:
            layer_clearance = zone_clearances[layer]

        # Build blocked set and net segment cells using helper function
        blocked, net_segment_cells = _build_layer_blocked_set(
            layer, net_id, pcb_data, coord, layer_clearance
        )

        # Cache plane_layer data for reuse in anchor flood fill
        if layer == plane_layer:
            blocked_plane = blocked
            net_plane_segment_cells = net_segment_cells

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

                # Expand to neighbors (4-connected for flood fill)
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nx, ny = gx + dx, gy + dy
                    if (nx, ny) in layer_visited:
                        continue
                    if nx < min_gx or nx > max_gx or ny < min_gy or ny > max_gy:
                        continue
                    if layer in zone_layers:
                        # Layer has a zone: flood through unblocked cells (zone copper)
                        # or through blocked cells if they're same-net segments
                        if (nx, ny) in blocked:
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
    for start_anchor_idx in range(len(anchor_points)):
        start_gx, start_gy = anchor_grid_points[start_anchor_idx]

        if (start_gx, start_gy) in plane_visited:
            # Already visited by a previous flood fill - skip this anchor.
            # The flood fill that visited this cell already handled union
            # with any reachable anchors.
            continue

        queue = deque([(start_gx, start_gy)])
        plane_visited.add((start_gx, start_gy))

        while queue:
            gx, gy = queue.popleft()

            if (gx, gy) in grid_to_anchors:
                for anchor_idx in grid_to_anchors[(gx, gy)]:
                    anchor_uf.union(start_anchor_idx, anchor_idx)

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = gx + dx, gy + dy
                if (nx, ny) in plane_visited:
                    continue
                if nx < min_gx or nx > max_gx or ny < min_gy or ny > max_gy:
                    continue
                if (nx, ny) in blocked_plane:
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
        cells = set(anchor_grid_points[i] for i in indices)
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

    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            cells.add((gx, gy))
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            cells.add((gx, gy))
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    cells.add((gx2, gy2))


def _block_segment_cells(
    blocked: Set[Tuple[int, int]],
    seg: Segment,
    coord: GridCoord,
    block_radius: int
):
    """Block grid cells along a segment with given radius."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    # Bresenham-like walk along segment
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = block_radius * block_radius

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-block_radius, block_radius + 1):
                for ey in range(-block_radius, block_radius + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        blocked.add((gx + ex, gy + ey))
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-block_radius, block_radius + 1):
                for ey in range(-block_radius, block_radius + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        blocked.add((gx + ex, gy + ey))
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-block_radius, block_radius + 1):
        for ey in range(-block_radius, block_radius + 1):
            if ex * ex + ey * ey <= radius_sq:
                blocked.add((gx2 + ex, gy2 + ey))


def _block_pad_cells(
    blocked: Set[Tuple[int, int]],
    pad: Pad,
    coord: GridCoord,
    clearance: float
):
    """Block grid cells for a pad with clearance."""
    half_w = pad.size_x / 2 + clearance
    half_h = pad.size_y / 2 + clearance

    min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
    max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
    _, min_gy = coord.to_grid(0, pad.global_y - half_h)
    _, max_gy = coord.to_grid(0, pad.global_y + half_h)

    for gx in range(min_gx, max_gx + 1):
        for gy in range(min_gy, max_gy + 1):
            blocked.add((gx, gy))


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

    # Find closest points between each pair of regions
    edges: List[Tuple[float, int, int, Tuple[float, float], Tuple[float, float]]] = []

    for i in range(n_regions):
        for j in range(i + 1, n_regions):
            best_dist = float('inf')
            best_pi = None
            best_pj = None

            for pi in region_anchors[i]:
                for pj in region_anchors[j]:
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
    search_radius: float = 5.0
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
    verbose: bool = False
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
    # Generate width steps: max, max/2, max/4, ..., min
    track_widths_to_try = []
    w = max_track_width
    while w >= min_track_width:
        track_widths_to_try.append(w)
        w = w / 2
    if track_widths_to_try[-1] > min_track_width:
        track_widths_to_try.append(min_track_width)

    result = None
    track_width = min_track_width
    open_space_via = None

    # Try each track width from widest to narrowest
    for try_width in track_widths_to_try:
        extra_margin_mm = (try_width - min_track_width) / 2
        track_margin = int(math.ceil(extra_margin_mm / config.grid_step))

        # Try routing in both directions - A* can find different paths depending on direction
        result = route_plane_connection_wide(
            anchors_i, anchors_j,
            plane_layer_idx=plane_layer_idx,
            routing_layers=routing_layers,
            base_obstacles=base_obstacles,
            config=config,
            net_vias=net_vias,
            track_margin=track_margin,
            max_iterations=max_iterations,
            verbose=verbose
        )

        if result is None:
            result = route_plane_connection_wide(
                anchors_j, anchors_i,
                plane_layer_idx=plane_layer_idx,
                routing_layers=routing_layers,
                base_obstacles=base_obstacles,
                config=config,
                net_vias=net_vias,
                track_margin=track_margin,
                max_iterations=max_iterations,
                verbose=verbose
            )

        if result is not None:
            track_width = try_width
            if verbose and try_width < max_track_width:
                print(f"width={try_width:.2f}mm...", end=" ", flush=True)
            break

    # Fallback: try routing via open-space points (areas with maximum clearance)
    if result is None:
        if verbose:
            print(f"trying open-space...", end=" ", flush=True)

        open_i = find_open_space_point(anchors_i, base_obstacles, plane_layer_idx, coord)
        open_j = find_open_space_point(anchors_j, base_obstacles, plane_layer_idx, coord)

        open_attempts = []
        if open_i:
            open_attempts.append((anchors_i + [open_i], anchors_j, open_i))
        if open_j:
            open_attempts.append((anchors_i, anchors_j + [open_j], open_j))
        if open_i and open_j:
            open_attempts.append((anchors_i + [open_i], anchors_j + [open_j], open_i))

        for src, tgt, via_candidate in open_attempts:
            result = route_plane_connection_wide(
                src, tgt,
                plane_layer_idx=plane_layer_idx,
                routing_layers=routing_layers,
                base_obstacles=base_obstacles,
                config=config,
                net_vias=net_vias,
                track_margin=0,  # min width for open-space
                max_iterations=max_iterations,
                verbose=verbose
            )
            if result:
                track_width = min_track_width
                route_pts = result[0]
                for pt in route_pts:
                    if abs(pt[0] - via_candidate[0]) < 0.01 and abs(pt[1] - via_candidate[1]) < 0.01:
                        open_space_via = via_candidate
                        break
                break

    return result, track_width, open_space_via


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
    track_via_clearance: float = 0.8,
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

    for edge_idx, (region_i, region_j, point_i, point_j, dist) in enumerate(mst_edges):
        # Get all anchors from each region for multi-point routing
        anchors_i = region_anchors[region_i]
        anchors_j = region_anchors[region_j]

        # Progress indicator
        print(f"    [{edge_idx+1}/{len(mst_edges)}] Region {region_i} ({len(anchors_i)} anchors) <-> Region {region_j} ({len(anchors_j)} anchors)...", end=" ", flush=True)

        # Try routing with multiple track widths using helper function
        result, track_width, open_space_via = _try_route_between_regions(
            anchors_i, anchors_j,
            base_obstacles=base_obstacles,
            plane_layer_idx=plane_layer_idx,
            routing_layers=routing_layers,
            config=config,
            net_vias=net_vias,
            max_track_width=max_track_width,
            min_track_width=min_track_width,
            max_iterations=max_iterations,
            coord=coord,
            verbose=verbose
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

    num_layers = len(routing_layers)
    layer_map = {layer: idx for idx, layer in enumerate(routing_layers)}

    obstacles = GridObstacleMap(num_layers)

    # Block other nets' vias as hard obstacles on ALL layers (vias are through-hole)
    via_radius = coord.to_grid_dist(track_via_clearance)
    for via in pcb_data.vias:
        if via.net_id in exclude_net_ids:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for layer_idx in range(num_layers):
            for ex in range(-via_radius, via_radius + 1):
                for ey in range(-via_radius, via_radius + 1):
                    if ex * ex + ey * ey <= via_radius * via_radius:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)

    # Add proximity costs around other nets' vias (on all layers)
    proximity_radius_grid = coord.to_grid_dist(proximity_radius)
    proximity_cost_grid = int(proximity_cost * 1000 / config.grid_step)

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

    # Block via placement near holes for hole-to-hole clearance
    hole_clearance_grid = max(1, coord.to_grid_dist(hole_to_hole_clearance + config.via_drill))

    # Block via placement near ALL vias (including same-net) for hole-to-hole clearance
    # Via reuse is handled separately by snapping routes to existing vias
    for via in pcb_data.vias:
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-hole_clearance_grid, hole_clearance_grid + 1):
            for ey in range(-hole_clearance_grid, hole_clearance_grid + 1):
                if ex * ex + ey * ey <= hole_clearance_grid * hole_clearance_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)

    # Block via placement near ALL through-hole pad holes (including same-net)
    # Via reuse at pads is handled separately by snapping routes to pad positions
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        for pad in pads:
            if '*.Cu' in pad.layers and pad.drill > 0:  # Through-hole pad with drill
                gx, gy = coord.to_grid(pad.global_x, pad.global_y)
                # Use pad's drill size + via drill for clearance calculation
                # (need to account for both drill radii: pad.drill/2 + clearance + via_drill/2)
                pad_hole_clearance_grid = max(1, coord.to_grid_dist(hole_to_hole_clearance + pad.drill / 2 + config.via_drill / 2))
                for ex in range(-pad_hole_clearance_grid, pad_hole_clearance_grid + 1):
                    for ey in range(-pad_hole_clearance_grid, pad_hole_clearance_grid + 1):
                        if ex * ex + ey * ey <= pad_hole_clearance_grid * pad_hole_clearance_grid:
                            obstacles.add_blocked_via(gx + ex, gy + ey)

    # Block existing segments from other nets on each layer
    # Also block vias along segments (vias span all layers, so can't place via where segment exists)
    for seg in pcb_data.segments:
        if seg.net_id in exclude_net_ids:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        seg_expansion_mm = track_width / 2 + seg.width / 2 + config.clearance
        seg_expansion_grid = max(1, coord.to_grid_dist(seg_expansion_mm))
        _block_segment_obstacle(obstacles, seg, coord, layer_idx, seg_expansion_grid)
        # Also block vias along this segment - must include segment width for proper clearance
        via_seg_expansion_mm = config.via_size / 2 + seg.width / 2 + config.clearance
        via_seg_expansion_grid = max(1, coord.to_grid_dist(via_seg_expansion_mm))
        _block_segment_via_obstacle(obstacles, seg, coord, via_seg_expansion_grid)

    # Block pads from non-plane nets on their respective layers
    # (plane net pads are excluded here - they're anchors; other plane nets' pads
    # will be blocked per-net in route_disconnected_regions)
    for pad_net_id, pads in pcb_data.pads_by_net.items():
        if pad_net_id in exclude_net_ids:
            continue
        for pad in pads:
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

            # Block rectangular area around pad with clearance for track routing
            pad_expansion_mm = track_width / 2 + config.clearance
            half_w = pad.size_x / 2 + pad_expansion_mm
            half_h = pad.size_y / 2 + pad_expansion_mm
            min_gx, _ = coord.to_grid(pad.global_x - half_w, 0)
            max_gx, _ = coord.to_grid(pad.global_x + half_w, 0)
            _, min_gy = coord.to_grid(0, pad.global_y - half_h)
            _, max_gy = coord.to_grid(0, pad.global_y + half_h)
            for layer_idx in pad_layers_on:
                for gx in range(min_gx, max_gx + 1):
                    for gy in range(min_gy, max_gy + 1):
                        obstacles.add_blocked_cell(gx, gy, layer_idx)
            # Also block vias around pad - need more clearance for via size
            via_expansion_mm = config.via_size / 2 + track_via_clearance
            via_half_w = pad.size_x / 2 + via_expansion_mm
            via_half_h = pad.size_y / 2 + via_expansion_mm
            via_min_gx, _ = coord.to_grid(pad.global_x - via_half_w, 0)
            via_max_gx, _ = coord.to_grid(pad.global_x + via_half_w, 0)
            _, via_min_gy = coord.to_grid(0, pad.global_y - via_half_h)
            _, via_max_gy = coord.to_grid(0, pad.global_y + via_half_h)
            for gx in range(via_min_gx, via_max_gx + 1):
                for gy in range(via_min_gy, via_max_gy + 1):
                    obstacles.add_blocked_via(gx, gy)

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

    # Block segments on their layers and also block vias along segments
    route_expansion_mm = track_width + clearance
    route_expansion_grid = max(1, coord.to_grid_dist(route_expansion_mm))
    via_seg_expansion_mm = config.via_size / 2 + track_width / 2 + clearance
    via_seg_expansion_grid = max(1, coord.to_grid_dist(via_seg_expansion_mm))

    for i in range(len(route_points) - 1):
        p1, p2 = route_points[i], route_points[i + 1]
        # Only block if on same layer (segments)
        if p1[2] == p2[2]:
            layer_idx = layer_map.get(p1[2], 0)
            gx1, gy1 = coord.to_grid(p1[0], p1[1])
            gx2, gy2 = coord.to_grid(p2[0], p2[1])
            _block_line_cells(obstacles, gx1, gy1, gx2, gy2, layer_idx, route_expansion_grid)
            # Also block vias along this segment
            _block_line_vias(obstacles, gx1, gy1, gx2, gy2, via_seg_expansion_grid)

    # Block vias on all layers (through-hole) for track routing clearance
    via_radius = coord.to_grid_dist(via_clearance)
    # Block via placement with hole-to-hole clearance (prevents new vias too close to these)
    hole_clearance_radius = coord.to_grid_dist(hole_to_hole_clearance + config.via_drill)

    for vx, vy in via_positions:
        gx, gy = coord.to_grid(vx, vy)
        # Block track routing around vias
        for layer_idx in range(num_layers):
            for ex in range(-via_radius, via_radius + 1):
                for ey in range(-via_radius, via_radius + 1):
                    if ex * ex + ey * ey <= via_radius * via_radius:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
        # Block via placement with proper hole-to-hole clearance
        for ex in range(-hole_clearance_radius, hole_clearance_radius + 1):
            for ey in range(-hole_clearance_radius, hole_clearance_radius + 1):
                if ex * ex + ey * ey <= hole_clearance_radius * hole_clearance_radius:
                    obstacles.add_blocked_via(gx + ex, gy + ey)


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
    verbose: bool = False
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

    # Create router and find path
    router = GridRouter(
        via_cost=config.via_cost * 1000,
        h_weight=config.heuristic_weight,
        turn_cost=config.turn_cost,
        via_proximity_cost=0
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
        return None

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

    return cleaned_points, new_via_positions


def _block_segment_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    layer_idx: int,
    expansion_grid: int
):
    """Block cells along a segment in the obstacle map."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    _block_line_cells(obstacles, gx1, gy1, gx2, gy2, layer_idx, expansion_grid)


def _block_segment_via_obstacle(
    obstacles: GridObstacleMap,
    seg: Segment,
    coord: GridCoord,
    expansion_grid: int
):
    """Block via placement along a segment (vias span all layers)."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)
    _block_line_vias(obstacles, gx1, gy1, gx2, gy2, expansion_grid)


def _block_line_vias(
    obstacles: GridObstacleMap,
    gx1: int, gy1: int,
    gx2: int, gy2: int,
    expansion_grid: int
):
    """Block via placement along a line with given expansion radius."""
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = expansion_grid * expansion_grid

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-expansion_grid, expansion_grid + 1):
        for ey in range(-expansion_grid, expansion_grid + 1):
            if ex * ex + ey * ey <= radius_sq:
                obstacles.add_blocked_via(gx2 + ex, gy2 + ey)


def _block_line_cells(
    obstacles: GridObstacleMap,
    gx1: int, gy1: int,
    gx2: int, gy2: int,
    layer_idx: int,
    expansion_grid: int
):
    """Block cells along a line with given expansion radius."""
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    gx, gy = gx1, gy1
    radius_sq = expansion_grid * expansion_grid

    if dx > dy:
        err = dx / 2
        while gx != gx2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            err -= dy
            if err < 0:
                gy += sy
                err += dx
            gx += sx
    else:
        err = dy / 2
        while gy != gy2:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    if ex * ex + ey * ey <= radius_sq:
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
            err -= dx
            if err < 0:
                gx += sx
                err += dy
            gy += sy

    # Block endpoint
    for ex in range(-expansion_grid, expansion_grid + 1):
        for ey in range(-expansion_grid, expansion_grid + 1):
            if ex * ex + ey * ey <= radius_sq:
                obstacles.add_blocked_cell(gx2 + ex, gy2 + ey, layer_idx)

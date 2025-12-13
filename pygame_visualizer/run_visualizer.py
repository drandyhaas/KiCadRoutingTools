#!/usr/bin/env python3
"""
Run the PyGame routing visualizer on a KiCad PCB file.

This script loads a KiCad PCB, sets up the routing context, and runs
the Rust A* router with real-time visualization.

Usage:
    python -m pygame_visualizer.run_visualizer input.kicad_pcb "Net-Name"

Or from the autorouter directory:
    python pygame_visualizer/run_visualizer.py input.kicad_pcb "Net-Name"
"""

import sys
import os
import fnmatch

# Add parent directory to path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Add rust_router to path
rust_router_dir = os.path.join(parent_dir, 'rust_router')
if rust_router_dir not in sys.path:
    sys.path.insert(0, rust_router_dir)

from typing import List, Tuple, Set, Optional

from kicad_parser import (
    parse_kicad_pcb, PCBData, Segment,
    auto_detect_bga_exclusion_zones, find_components_by_type
)
from batch_grid_router import (
    find_connected_groups, get_net_endpoints, GridRouteConfig,
    compute_mps_net_ordering, get_net_routing_endpoints,
    add_same_net_via_clearance
)

# Import Rust router
try:
    import grid_router
    from grid_router import GridObstacleMap, VisualRouter, GridRouter
    print(f"Using Rust router v{grid_router.__version__}")
except ImportError as e:
    print("ERROR: Rust router module not found!")
    print("Build it with:")
    print("  cd rust_router && cargo build --release")
    print("  cp target/release/grid_router.dll grid_router.pyd  # Windows")
    sys.exit(1)

from pygame_visualizer.visualizer import RoutingVisualizer
from pygame_visualizer.config import VisualizerConfig


def find_connected_groups(segments: List[Segment], tolerance: float = 0.01) -> List[List[Segment]]:
    """Find groups of connected segments using union-find."""
    if not segments:
        return []

    n = len(segments)
    parent = list(range(n))

    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        if px != py:
            parent[px] = py

    for i in range(n):
        for j in range(i + 1, n):
            si, sj = segments[i], segments[j]
            pts_i = [(si.start_x, si.start_y), (si.end_x, si.end_y)]
            pts_j = [(sj.start_x, sj.start_y), (sj.end_x, sj.end_y)]
            for pi in pts_i:
                for pj in pts_j:
                    if abs(pi[0] - pj[0]) < tolerance and abs(pi[1] - pj[1]) < tolerance:
                        union(i, j)
                        break

    groups = {}
    for i in range(n):
        root = find(i)
        if root not in groups:
            groups[root] = []
        groups[root].append(segments[i])

    return list(groups.values())


class GridCoord:
    """Coordinate conversion utilities."""

    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        return round(dist_mm * self.inv_step)


def build_base_obstacle_map(
    pcb_data: PCBData,
    nets_to_route: List[int],
    layers: List[str],
    grid_step: float = 0.1,
    track_width: float = 0.1,
    clearance: float = 0.1,
    via_size: float = 0.3,
    bga_zones: List[Tuple[float, float, float, float]] = None,
) -> Tuple[GridObstacleMap, List[Set[Tuple[int, int]]], Set[Tuple[int, int]], List[Tuple[int, int, int, int]]]:
    """
    Build base obstacle map with static obstacles, excluding all nets to be routed.
    This is built once at startup and cloned for each net.

    Returns:
        (obstacles, blocked_cells_per_layer, blocked_vias, bga_zones_grid)
    """
    coord = GridCoord(grid_step)
    num_layers = len(layers)
    layer_map = {name: idx for idx, name in enumerate(layers)}
    nets_to_route_set = set(nets_to_route)

    obstacles = GridObstacleMap(num_layers)

    # Caches for visualization
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Add BGA exclusion zones - block ALL layers
    if bga_zones:
        for zone in bga_zones:
            min_x, min_y, max_x, max_y = zone
            gmin_x, gmin_y = coord.to_grid(min_x, min_y)
            gmax_x, gmax_y = coord.to_grid(max_x, max_y)
            obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
            bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
            for layer_idx in range(num_layers):
                for gx in range(gmin_x, gmax_x + 1):
                    for gy in range(gmin_y, gmax_y + 1):
                        obstacles.add_blocked_cell(gx, gy, layer_idx)
                        blocked_cells[layer_idx].add((gx, gy))

    # Precompute expansions
    expansion_mm = track_width / 2 + clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = via_size / 2 + track_width / 2 + clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))
    via_track_expansion = max(1, coord.to_grid_dist(via_size / 2 + track_width / 2 + clearance))
    via_via_expansion = max(1, coord.to_grid_dist(via_size + clearance))

    # Add segments as obstacles (excluding nets we'll route)
    for seg in pcb_data.segments:
        if seg.net_id in nets_to_route_set:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid, blocked_cells, blocked_vias)

    # Add vias as obstacles (excluding nets we'll route)
    for via in pcb_data.vias:
        if via.net_id in nets_to_route_set:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion, via_via_expansion, blocked_cells, blocked_vias)

    # Add pads as obstacles (excluding nets we'll route)
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id in nets_to_route_set:
            continue
        for pad in pads:
            _add_pad_obstacle(obstacles, pad, coord, layer_map, expansion_grid, via_block_grid,
                              blocked_cells, blocked_vias, grid_step=grid_step)

    return obstacles, blocked_cells, blocked_vias, bga_zones_grid


def _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                          blocked_cells, blocked_vias):
    """Add a segment as obstacle."""
    gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
    gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        for ex in range(-expansion_grid, expansion_grid + 1):
            for ey in range(-expansion_grid, expansion_grid + 1):
                obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_block_grid, via_block_grid + 1):
            for ey in range(-via_block_grid, via_block_grid + 1):
                if ex * ex + ey * ey <= via_block_grid * via_block_grid:
                    obstacles.add_blocked_via(gx + ex, gy + ey)
                    blocked_vias.add((gx + ex, gy + ey))

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion, via_via_expansion,
                      blocked_cells, blocked_vias):
    """Add a via as obstacle."""
    gx, gy = coord.to_grid(via.x, via.y)
    for ex in range(-via_track_expansion, via_track_expansion + 1):
        for ey in range(-via_track_expansion, via_track_expansion + 1):
            if ex * ex + ey * ey <= via_track_expansion * via_track_expansion:
                for layer_idx in range(num_layers):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))
    for ex in range(-via_via_expansion, via_via_expansion + 1):
        for ey in range(-via_via_expansion, via_via_expansion + 1):
            if ex * ex + ey * ey <= via_via_expansion * via_via_expansion:
                obstacles.add_blocked_via(gx + ex, gy + ey)
                blocked_vias.add((gx + ex, gy + ey))


def _add_pad_obstacle(obstacles, pad, coord, layer_map, expansion_grid, via_block_grid,
                      blocked_cells, blocked_vias, grid_step=0.1, clearance=0.1, via_size=0.3):
    """Add a pad as obstacle (matches build_obstacle_map_with_cache logic)."""
    gx, gy = coord.to_grid(pad.global_x, pad.global_y)
    half_x_mm = pad.size_x / 2 + clearance
    half_y_mm = pad.size_y / 2 + clearance
    expand_x = coord.to_grid_dist(half_x_mm)
    expand_y = coord.to_grid_dist(half_y_mm)

    for layer_name in pad.layers:
        layer_idx = layer_map.get(layer_name)
        if layer_idx is None:
            continue
        for dx in range(-expand_x, expand_x + 1):
            for dy in range(-expand_y, expand_y + 1):
                obstacles.add_blocked_cell(gx + dx, gy + dy, layer_idx)
                blocked_cells[layer_idx].add((gx + dx, gy + dy))

    # Block vias around pad only if pad is on F.Cu or B.Cu
    if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
        via_clear_mm = via_size / 2 + clearance
        via_expand_x = int((pad.size_x / 2 + via_clear_mm) / grid_step)
        via_expand_y = int((pad.size_y / 2 + via_clear_mm) / grid_step)
        for dx in range(-via_expand_x, via_expand_x + 1):
            for dy in range(-via_expand_y, via_expand_y + 1):
                obstacles.add_blocked_via(gx + dx, gy + dy)
                blocked_vias.add((gx + dx, gy + dy))


def add_routed_net_obstacles(obstacles, pcb_data, net_id, layers, grid_step,
                             track_width=0.1, clearance=0.1, via_size=0.3,
                             blocked_cells=None, blocked_vias=None):
    """Add a routed net's segments, vias, and pads as obstacles (for incremental building)."""
    coord = GridCoord(grid_step)
    num_layers = len(layers)
    layer_map = {name: idx for idx, name in enumerate(layers)}

    expansion_grid = max(1, coord.to_grid_dist(track_width / 2 + clearance))
    via_block_grid = max(1, coord.to_grid_dist(via_size / 2 + track_width / 2 + clearance))
    via_track_expansion = max(1, coord.to_grid_dist(via_size / 2 + track_width / 2 + clearance))
    via_via_expansion = max(1, coord.to_grid_dist(via_size + clearance))

    if blocked_cells is None:
        blocked_cells = [set() for _ in range(num_layers)]
    if blocked_vias is None:
        blocked_vias = set()

    for seg in pcb_data.segments:
        if seg.net_id != net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue
        _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid, blocked_cells, blocked_vias)

    for via in pcb_data.vias:
        if via.net_id != net_id:
            continue
        _add_via_obstacle(obstacles, via, coord, num_layers, via_track_expansion, via_via_expansion, blocked_cells, blocked_vias)

    # Add pads as obstacles (same as batch_grid_router)
    pads = pcb_data.pads_by_net.get(net_id, [])
    for pad in pads:
        _add_pad_obstacle(obstacles, pad, coord, layer_map, expansion_grid, via_block_grid, blocked_cells, blocked_vias)


def build_obstacle_map_with_cache(
    pcb_data: PCBData,
    exclude_net_id: int,
    layers: List[str],
    grid_step: float = 0.1,
    track_width: float = 0.1,
    clearance: float = 0.1,
    via_size: float = 0.3,
    bga_zones: List[Tuple[float, float, float, float]] = None,
    unrouted_stubs: List[Tuple[float, float]] = None,
    stub_proximity_radius: float = 1.5,
    stub_proximity_cost: float = 3.0
) -> Tuple[GridObstacleMap, List[Set[Tuple[int, int]]], Set[Tuple[int, int]], List[Tuple[int, int, int, int]]]:
    """
    Build Rust obstacle map and return caches for visualization.

    Returns:
        (obstacles, blocked_cells_per_layer, blocked_vias, bga_zones_grid)
    """
    coord = GridCoord(grid_step)
    num_layers = len(layers)
    layer_map = {name: idx for idx, name in enumerate(layers)}

    # Create Rust obstacle map
    obstacles = GridObstacleMap(num_layers)

    # Caches for visualization
    blocked_cells: List[Set[Tuple[int, int]]] = [set() for _ in range(num_layers)]
    blocked_vias: Set[Tuple[int, int]] = set()
    bga_zones_grid: List[Tuple[int, int, int, int]] = []

    # Add BGA exclusion zones - block ALL layers
    if bga_zones:
        for zone in bga_zones:
            min_x, min_y, max_x, max_y = zone
            gmin_x, gmin_y = coord.to_grid(min_x, min_y)
            gmax_x, gmax_y = coord.to_grid(max_x, max_y)
            obstacles.set_bga_zone(gmin_x, gmin_y, gmax_x, gmax_y)
            bga_zones_grid.append((gmin_x, gmin_y, gmax_x, gmax_y))
            # Block ALL layers in BGA zone
            for layer_idx in range(num_layers):
                for gx in range(gmin_x, gmax_x + 1):
                    for gy in range(gmin_y, gmax_y + 1):
                        obstacles.add_blocked_cell(gx, gy, layer_idx)
                        blocked_cells[layer_idx].add((gx, gy))

    # Add segments as obstacles
    expansion_mm = track_width / 2 + clearance
    expansion_grid = max(1, coord.to_grid_dist(expansion_mm))
    via_block_mm = via_size / 2 + track_width / 2 + clearance
    via_block_grid = max(1, coord.to_grid_dist(via_block_mm))

    for seg in pcb_data.segments:
        if seg.net_id == exclude_net_id:
            continue
        layer_idx = layer_map.get(seg.layer)
        if layer_idx is None:
            continue

        gx1, gy1 = coord.to_grid(seg.start_x, seg.start_y)
        gx2, gy2 = coord.to_grid(seg.end_x, seg.end_y)

        # Bresenham line with expansion
        dx = abs(gx2 - gx1)
        dy = abs(gy2 - gy1)
        sx = 1 if gx1 < gx2 else -1
        sy = 1 if gy1 < gy2 else -1
        err = dx - dy

        gx, gy = gx1, gy1
        while True:
            for ex in range(-expansion_grid, expansion_grid + 1):
                for ey in range(-expansion_grid, expansion_grid + 1):
                    obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                    blocked_cells[layer_idx].add((gx + ex, gy + ey))
            for ex in range(-via_block_grid, via_block_grid + 1):
                for ey in range(-via_block_grid, via_block_grid + 1):
                    if ex * ex + ey * ey <= via_block_grid * via_block_grid:
                        obstacles.add_blocked_via(gx + ex, gy + ey)
                        blocked_vias.add((gx + ex, gy + ey))

            if gx == gx2 and gy == gy2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                gx += sx
            if e2 < dx:
                err += dx
                gy += sy

    # Add vias as obstacles
    via_track_expansion = max(1, coord.to_grid_dist(via_size / 2 + track_width / 2 + clearance))
    via_via_expansion = max(1, coord.to_grid_dist(via_size + clearance))

    for via in pcb_data.vias:
        if via.net_id == exclude_net_id:
            continue
        gx, gy = coord.to_grid(via.x, via.y)
        for ex in range(-via_track_expansion, via_track_expansion + 1):
            for ey in range(-via_track_expansion, via_track_expansion + 1):
                if ex * ex + ey * ey <= via_track_expansion * via_track_expansion:
                    for layer_idx in range(num_layers):
                        obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                        blocked_cells[layer_idx].add((gx + ex, gy + ey))
        for ex in range(-via_via_expansion, via_via_expansion + 1):
            for ey in range(-via_via_expansion, via_via_expansion + 1):
                if ex * ex + ey * ey <= via_via_expansion * via_via_expansion:
                    obstacles.add_blocked_via(gx + ex, gy + ey)
                    blocked_vias.add((gx + ex, gy + ey))

    # Add pads as obstacles
    for net_id, pads in pcb_data.pads_by_net.items():
        if net_id == exclude_net_id:
            continue
        for pad in pads:
            gx, gy = coord.to_grid(pad.global_x, pad.global_y)
            half_x_mm = pad.size_x / 2 + clearance
            half_y_mm = pad.size_y / 2 + clearance
            expand_x = coord.to_grid_dist(half_x_mm)
            expand_y = coord.to_grid_dist(half_y_mm)

            for ex in range(-expand_x, expand_x + 1):
                for ey in range(-expand_y, expand_y + 1):
                    for layer in pad.layers:
                        layer_idx = layer_map.get(layer)
                        if layer_idx is not None:
                            obstacles.add_blocked_cell(gx + ex, gy + ey, layer_idx)
                            blocked_cells[layer_idx].add((gx + ex, gy + ey))

            if 'F.Cu' in pad.layers or 'B.Cu' in pad.layers:
                via_clear_mm = via_size / 2 + clearance
                via_expand_x = int((pad.size_x / 2 + via_clear_mm) / grid_step)
                via_expand_y = int((pad.size_y / 2 + via_clear_mm) / grid_step)
                for ex in range(-via_expand_x, via_expand_x + 1):
                    for ey in range(-via_expand_y, via_expand_y + 1):
                        obstacles.add_blocked_via(gx + ex, gy + ey)
                        blocked_vias.add((gx + ex, gy + ey))

    # Add stub proximity costs (same as batch_grid_router)
    if unrouted_stubs:
        stub_radius_grid = coord.to_grid_dist(stub_proximity_radius)
        stub_cost_grid = int(stub_proximity_cost * 1000 / grid_step)
        for stub_x, stub_y in unrouted_stubs:
            gcx, gcy = coord.to_grid(stub_x, stub_y)
            for dx in range(-stub_radius_grid, stub_radius_grid + 1):
                for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                    dist_sq = dx * dx + dy * dy
                    if dist_sq <= stub_radius_grid * stub_radius_grid:
                        dist = (dist_sq ** 0.5)
                        proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                        cost = int(proximity * stub_cost_grid)
                        obstacles.set_stub_proximity(gcx + dx, gcy + dy, cost)

    return obstacles, blocked_cells, blocked_vias, bga_zones_grid


def get_stub_endpoints(pcb_data: PCBData, net_ids: List[int]) -> List[Tuple[float, float]]:
    """Get centroid positions of unrouted net stubs for proximity avoidance.

    Same as batch_grid_router.get_stub_endpoints().
    """
    stubs = []
    for net_id in net_ids:
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        if len(net_segments) < 2:
            continue
        groups = find_connected_groups(net_segments)
        if len(groups) < 2:
            continue
        for group in groups:
            points = []
            for seg in group:
                points.append((seg.start_x, seg.start_y))
                points.append((seg.end_x, seg.end_y))
            if points:
                cx = sum(p[0] for p in points) / len(points)
                cy = sum(p[1] for p in points) / len(points)
                stubs.append((cx, cy))
    return stubs


def get_bounds(pcb_data: PCBData, net_ids: List[int], padding: float = 5.0) -> Tuple[float, float, float, float]:
    """Get bounding box around all the nets' components."""
    xs = []
    ys = []

    for net_id in net_ids:
        pads = pcb_data.pads_by_net.get(net_id, [])
        segments = [s for s in pcb_data.segments if s.net_id == net_id]

        for pad in pads:
            xs.append(pad.global_x)
            ys.append(pad.global_y)

        for seg in segments:
            xs.extend([seg.start_x, seg.end_x])
            ys.extend([seg.start_y, seg.end_y])

    if not xs or not ys:
        return (0, 0, 100, 100)

    return (min(xs) - padding, min(ys) - padding, max(xs) + padding, max(ys) + padding)


def run_visualization(
    pcb_file: str,
    net_names: List[str],
    layers: List[str] = None,
    grid_step: float = 0.1,
    max_iterations: int = 100000,
    auto_advance: bool = False,
    display_time: float = 0.0,
    disable_bga_zones: bool = False,
    ordering_strategy: str = "mps",
    track_width: float = 0.1,
    clearance: float = 0.1,
    via_size: float = 0.3,
    via_drill: float = 0.2,
    via_cost: int = 25,
    heuristic_weight: float = 1.5,
    stub_proximity_radius: float = 1.5,
    stub_proximity_cost: float = 2.0
):
    """Run the visualization for multiple nets using the Rust router.

    If auto_advance is True, automatically proceed to next net without waiting for N key.
    display_time: seconds to display completed route before auto-advancing (only used with auto_advance).
    If disable_bga_zones is True, BGA exclusion zones are not applied.
    ordering_strategy: "mps" (default), "inside_out", or "original"
    """
    print(f"Loading {pcb_file}...")
    pcb_data = parse_kicad_pcb(pcb_file)

    # Default layers
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']

    print(f"Using layers: {layers}")

    # Auto-detect BGA zones (unless disabled)
    if disable_bga_zones:
        bga_zones = []
        print("BGA exclusion zones disabled")
    else:
        bga_zones = auto_detect_bga_exclusion_zones(pcb_data, margin=0.5)
        if bga_zones:
            print(f"Detected {len(bga_zones)} BGA exclusion zone(s)")

    # Find all net IDs and validate - check both pcb.nets and pads_by_net
    net_queue = []  # List of (net_name, net_id)
    for net_name in net_names:
        net_id = None
        # First check pcb.nets
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        # If not found, check pads_by_net (for nets not in pcb.nets)
        if net_id is None:
            for nid, pads in pcb_data.pads_by_net.items():
                for pad in pads:
                    if pad.net_name == net_name:
                        net_id = nid
                        break
                if net_id is not None:
                    break
        if net_id is None:
            print(f"Warning: Net '{net_name}' not found, skipping")
        else:
            net_queue.append((net_name, net_id))

    if not net_queue:
        print("No valid nets to route!")
        return

    # Apply net ordering strategy (same as batch_grid_router.py)
    if ordering_strategy == "mps":
        print("\nUsing MPS ordering strategy...")
        net_ids_for_mps = [net_id for _, net_id in net_queue]
        ordered_net_ids = compute_mps_net_ordering(pcb_data, net_ids_for_mps)
        # Rebuild net_queue in MPS order
        net_id_to_name = {net_id: net_name for net_name, net_id in net_queue}
        net_queue = [(net_id_to_name[nid], nid) for nid in ordered_net_ids]
    elif ordering_strategy == "inside_out" and bga_zones:
        # Sort nets inside-out from BGA center(s) for better escape routing
        def pad_in_bga_zone(pad):
            """Check if a pad is inside any BGA zone."""
            for zone in bga_zones:
                if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                    return True
            return False

        def get_min_distance_to_bga_center(net_id):
            """Get minimum distance from any BGA pad of this net to its BGA center."""
            pads = pcb_data.pads_by_net.get(net_id, [])
            if not pads:
                return float('inf')

            min_dist = float('inf')
            for zone in bga_zones:
                center_x = (zone[0] + zone[2]) / 2
                center_y = (zone[1] + zone[3]) / 2
                for pad in pads:
                    # Only consider pads that are inside this BGA zone
                    if zone[0] <= pad.global_x <= zone[2] and zone[1] <= pad.global_y <= zone[3]:
                        dist = ((pad.global_x - center_x) ** 2 + (pad.global_y - center_y) ** 2) ** 0.5
                        min_dist = min(min_dist, dist)
            return min_dist

        # Separate BGA nets from non-BGA nets
        bga_nets = []
        non_bga_nets = []
        for net_name, net_id in net_queue:
            pads = pcb_data.pads_by_net.get(net_id, [])
            has_bga_pad = any(pad_in_bga_zone(pad) for pad in pads)
            if has_bga_pad:
                bga_nets.append((net_name, net_id))
            else:
                non_bga_nets.append((net_name, net_id))

        # Sort BGA nets inside-out, keep non-BGA nets in original order
        bga_nets.sort(key=lambda x: get_min_distance_to_bga_center(x[1]))
        net_queue = bga_nets + non_bga_nets

        if bga_nets:
            print(f"Sorted {len(bga_nets)} BGA nets inside-out ({len(non_bga_nets)} non-BGA nets unchanged)")
    elif ordering_strategy == "original":
        print("Using original net ordering")

    print(f"\nWill route {len(net_queue)} nets")

    coord = GridCoord(grid_step)
    layer_map = {name: idx for idx, name in enumerate(layers)}

    # Compute global bounds for all nets once (so camera shows full routing area)
    all_net_ids = [net_id for _, net_id in net_queue]
    global_bounds = get_bounds(pcb_data, all_net_ids, padding=5.0)

    # Build base obstacle map once (excludes all nets we'll route)
    # This contains: BGA zones, pads/segments/vias from OTHER nets (not in our queue)
    import time
    print("\nBuilding base obstacle map...")
    t0 = time.time()
    base_obstacles, base_blocked_cells, base_blocked_vias, base_bga_zones_grid = build_base_obstacle_map(
        pcb_data, all_net_ids, layers,
        grid_step=grid_step,
        bga_zones=bga_zones
    )
    print(f"  Base obstacle map built in {time.time() - t0:.2f}s")

    # Track added segments/vias per net for restart capability
    # Key: net_idx, Value: (num_segments_added, num_vias_added)
    added_geometry = {}

    # Initialize visualizer once
    config = VisualizerConfig(layers=layers)
    visualizer = RoutingVisualizer(config)

    # In auto-advance mode, use maximum speed
    if auto_advance:
        visualizer.iterations_per_frame = config.max_speed

    # Routing state
    current_net_idx = 0
    router = None
    sources = []
    targets = []
    obstacles = None
    successful = 0
    failed = 0
    waiting_for_next = False  # Wait for 'N' key before next net
    display_wait_start = None  # Time when display wait started (for --display-time)
    total_iterations = 0  # Track total iterations across all nets
    net_results = []  # Track results per net: (name, iterations, path_length, success)
    current_net_iterations = 0  # Track iterations for current net (across direction attempts)

    # Direction search state
    forward_done = False  # True after forward attempt completes (success or fail)
    reverse_done = False  # True after reverse attempt completes (success or fail)
    original_sources = []
    original_targets = []

    # Routing config (set by setup_net, used by main loop for backwards retry)
    routing_config = None

    # Track routed/remaining nets to match batch router behavior
    routed_net_ids = []  # Nets that have been successfully routed
    remaining_net_ids = [nid for _, nid in net_queue]  # Nets not yet routed (or failed)
    def setup_net(net_idx: int) -> bool:
        """Set up routing for the net at the given index. Returns False if net should be skipped."""
        nonlocal router, sources, targets, obstacles, current_net_iterations
        nonlocal forward_done, reverse_done, original_sources, original_targets, routing_config

        if net_idx >= len(net_queue):
            return False

        net_name, net_id = net_queue[net_idx]
        print(f"\n[{net_idx + 1}/{len(net_queue)}] Setting up {net_name}...")
        print(f"  PCB has {len(pcb_data.segments)} segments and {len(pcb_data.vias)} vias")

        # Create a config for get_net_endpoints (matches batch router)
        routing_config = GridRouteConfig(
            layers=layers,
            grid_step=grid_step,
            track_width=track_width,
            clearance=clearance,
            via_size=via_size,
            via_drill=via_drill,
            via_cost=via_cost,
            max_iterations=max_iterations,
            heuristic_weight=heuristic_weight,
            stub_proximity_radius=stub_proximity_radius,
            stub_proximity_cost=stub_proximity_cost
        )

        # Find endpoints (handles segments, pads, or both) - same as batch router
        sources_full, targets_full, error = get_net_endpoints(pcb_data, net_id, routing_config)
        if error:
            print(f"  {error}")
            return False

        if not sources_full or not targets_full:
            print("  No valid source/target points, skipping")
            return False

        # Count groups for display
        net_segments = [s for s in pcb_data.segments if s.net_id == net_id]
        net_pads = pcb_data.pads_by_net.get(net_id, [])
        if net_segments:
            groups = find_connected_groups(net_segments)
            print(f"  Found {len(groups)} disconnected stub groups")
        else:
            print(f"  Routing between {len(net_pads)} pads (no stubs)")

        # Convert to grid-only format for routing (batch router does same thing internally)
        sources = [(s[0], s[1], s[2]) for s in sources_full]
        targets = [(t[0], t[1], t[2]) for t in targets_full]

        # Compute other unrouted nets (all remaining except current) - matches batch router
        other_unrouted = [nid for nid in remaining_net_ids if nid != net_id]

        # Get unrouted stubs for proximity avoidance - use all unrouted nets (not just future ones)
        unrouted_stubs = get_stub_endpoints(pcb_data, other_unrouted)

        # Build obstacle map by cloning base and adding other nets' geometry
        import time
        t0 = time.time()

        # Clone base obstacle map (contains BGA zones and non-queue nets' obstacles)
        obstacles = base_obstacles.clone()
        blocked_cells = [set(bc) for bc in base_blocked_cells]
        blocked_vias = set(base_blocked_vias)
        bga_zones_grid = base_bga_zones_grid

        # Precompute grid expansions
        expansion_grid = max(1, coord.to_grid_dist(0.1 / 2 + 0.1))
        via_block_grid = max(1, coord.to_grid_dist(0.3 / 2 + 0.1 / 2 + 0.1))
        via_track_expansion = max(1, coord.to_grid_dist(0.3 / 2 + 0.1 / 2 + 0.1))
        via_via_expansion = max(1, coord.to_grid_dist(0.3 + 0.1))

        # Add previously routed nets' segments/vias/pads as obstacles (from pcb_data)
        # These nets have new geometry added via add_path_to_pcb
        for routed_id in routed_net_ids:
            # Add segments (stubs + routed path)
            for seg in pcb_data.segments:
                if seg.net_id != routed_id:
                    continue
                layer_idx = layer_map.get(seg.layer)
                if layer_idx is None:
                    continue
                _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                                      blocked_cells, blocked_vias)
            # Add vias (from routed path)
            for via in pcb_data.vias:
                if via.net_id != routed_id:
                    continue
                _add_via_obstacle(obstacles, via, coord, len(layers), via_track_expansion, via_via_expansion,
                                  blocked_cells, blocked_vias)
            # Add pads
            pads = pcb_data.pads_by_net.get(routed_id, [])
            for pad in pads:
                _add_pad_obstacle(obstacles, pad, coord, layer_map, expansion_grid, via_block_grid,
                                  blocked_cells, blocked_vias, grid_step=grid_step)

        # Add other unrouted nets' stubs, vias, and pads as obstacles (not the current net)
        for other_net_id in other_unrouted:
            # Add segments (stubs)
            for seg in pcb_data.segments:
                if seg.net_id != other_net_id:
                    continue
                layer_idx = layer_map.get(seg.layer)
                if layer_idx is None:
                    continue
                _add_segment_obstacle(obstacles, seg, coord, layer_idx, expansion_grid, via_block_grid,
                                      blocked_cells, blocked_vias)
            # Add vias (in case they exist in input data)
            for via in pcb_data.vias:
                if via.net_id != other_net_id:
                    continue
                _add_via_obstacle(obstacles, via, coord, len(layers), via_track_expansion, via_via_expansion,
                                  blocked_cells, blocked_vias)
            # Add pads
            pads = pcb_data.pads_by_net.get(other_net_id, [])
            for pad in pads:
                _add_pad_obstacle(obstacles, pad, coord, layer_map, expansion_grid, via_block_grid,
                                  blocked_cells, blocked_vias, grid_step=grid_step)

        # Add stub proximity costs for remaining unrouted nets
        # Must match batch router's add_stub_proximity_costs logic
        if unrouted_stubs:
            stub_radius_grid = coord.to_grid_dist(routing_config.stub_proximity_radius)
            stub_cost_grid = int(routing_config.stub_proximity_cost * 1000 / grid_step)
            for (sx, sy) in unrouted_stubs:
                gx, gy = coord.to_grid(sx, sy)
                for dx in range(-stub_radius_grid, stub_radius_grid + 1):
                    for dy in range(-stub_radius_grid, stub_radius_grid + 1):
                        dist_sq = dx * dx + dy * dy
                        if dist_sq <= stub_radius_grid * stub_radius_grid:
                            dist = dist_sq ** 0.5
                            proximity = 1.0 - (dist / stub_radius_grid) if stub_radius_grid > 0 else 1.0
                            cost = int(proximity * stub_cost_grid)
                            if cost > 0:
                                obstacles.set_stub_proximity(gx + dx, gy + dy, cost)

        # Add same-net via clearance blocking (must match batch router)
        add_same_net_via_clearance(obstacles, pcb_data, net_id, routing_config)

        build_time = time.time() - t0
        print(f"  Obstacle map cloned+built in {build_time:.3f}s")

        # Debug: show layer distribution and obstacle info
        source_layers = set(s[2] for s in sources)
        target_layers = set(t[2] for t in targets)
        print(f"  Source points: {len(sources)} on layers {[layers[l] for l in source_layers]}")
        print(f"  Target points: {len(targets)} on layers {[layers[l] for l in target_layers]}")

        # Debug: check how many vias are blocked in the region around source/target
        all_pts = sources + targets
        if all_pts:
            min_gx = min(p[0] for p in all_pts)
            max_gx = max(p[0] for p in all_pts)
            min_gy = min(p[1] for p in all_pts)
            max_gy = max(p[1] for p in all_pts)
            region_vias_blocked = sum(1 for (vx, vy) in blocked_vias
                                      if min_gx - 10 <= vx <= max_gx + 10
                                      and min_gy - 10 <= vy <= max_gy + 10)
            print(f"  Blocked vias in routing region: {region_vias_blocked}")
            print(f"  Routing region: ({min_gx}, {min_gy}) to ({max_gx}, {max_gy})")

        # Add allowed cells around sources/targets to override BGA zone blocking
        # This only affects BGA zone blocking, not regular obstacle blocking (tracks, stubs, pads)
        allow_radius = 10
        for gx, gy, _ in sources + targets:
            for dx in range(-allow_radius, allow_radius + 1):
                for dy in range(-allow_radius, allow_radius + 1):
                    obstacles.add_allowed_cell(gx + dx, gy + dy)

        # Mark exact source/target cells so routing can start/end there even if blocked by
        # adjacent track expansion (but NOT blocked by BGA zones - use allowed_cells for that)
        # NOTE: Must pass layer to only allow override on the specific layer of the endpoint
        for gx, gy, layer in sources + targets:
            obstacles.add_source_target_cell(gx, gy, layer)

        # Update visualizer context (use global_bounds to show full routing area)
        visualizer.set_routing_context(
            obstacles,
            sources,
            targets,
            grid_step=grid_step,
            bounds=global_bounds,
            bga_zones=bga_zones_grid,
            blocked_cells=blocked_cells,
            blocked_vias=blocked_vias,
        )

        # Reset flags for new net
        forward_done = False
        reverse_done = False
        current_net_iterations = 0
        original_sources = list(sources)
        original_targets = list(targets)

        # Initialize Rust VisualRouter
        router = VisualRouter(via_cost=routing_config.via_cost * 1000, h_weight=routing_config.heuristic_weight)
        router.init(sources, targets, routing_config.max_iterations)

        # Tell visualizer which net we're routing
        visualizer.set_current_net(net_name, net_idx + 1, len(net_queue))
        visualizer.status_message = ""  # Clear any previous status

        return True

    def add_path_to_pcb(path, net_id: int, net_idx: int):
        """Add a routed path to pcb_data so subsequent routes avoid it."""
        from kicad_parser import Segment, Via
        layer_names = layers

        segs_before = len(pcb_data.segments)
        vias_before = len(pcb_data.vias)
        print(f"  Adding path with {len(path)} points to pcb_data...")

        for i in range(len(path) - 1):
            gx1, gy1, layer1 = path[i]
            gx2, gy2, layer2 = path[i + 1]

            x1, y1 = coord.to_float(gx1, gy1)
            x2, y2 = coord.to_float(gx2, gy2)

            if layer1 != layer2:
                # Via
                via = Via(
                    x=x1, y=y1,
                    size=0.3, drill=0.2,
                    layers=[layer_names[layer1], layer_names[layer2]],
                    net_id=net_id
                )
                pcb_data.vias.append(via)
            else:
                # Segment
                if (x1, y1) != (x2, y2):
                    seg = Segment(
                        start_x=x1, start_y=y1,
                        end_x=x2, end_y=y2,
                        width=0.1,
                        layer=layer_names[layer1],
                        net_id=net_id
                    )
                    pcb_data.segments.append(seg)

        # Track how many were added so we can remove them on restart
        segs_added = len(pcb_data.segments) - segs_before
        vias_added = len(pcb_data.vias) - vias_before
        added_geometry[net_idx] = (segs_added, vias_added)
        print(f"  Added {segs_added} segments and {vias_added} vias to pcb_data")

    def remove_net_geometry(net_idx: int):
        """Remove previously routed geometry for a net."""
        if net_idx in added_geometry:
            segs_added, vias_added = added_geometry[net_idx]
            if segs_added > 0:
                pcb_data.segments = pcb_data.segments[:-segs_added]
            if vias_added > 0:
                pcb_data.vias = pcb_data.vias[:-vias_added]
            del added_geometry[net_idx]

    # Set up first net
    while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
        failed += 1
        current_net_idx += 1

    if current_net_idx >= len(net_queue):
        print("\nNo routable nets found!")
        return

    print("\nStarting visualization with Rust router...")
    print("Controls: Space=pause, N=next net, R=restart net, Ctrl+R=restart all, +/-=speed, Q=quit")

    # Keep original pcb_data for restart all
    original_segments = list(pcb_data.segments)
    original_vias = list(pcb_data.vias)

    try:
        while visualizer.running:
            # Handle events
            if not visualizer.handle_events():
                break

            # Check for restart all request (Ctrl+R)
            if visualizer.restart_all_requested:
                visualizer.restart_all_requested = False
                # Reset pcb_data to original state
                pcb_data.segments = list(original_segments)
                pcb_data.vias = list(original_vias)
                # Clear completed routes display
                visualizer.clear_completed_routes()
                added_geometry.clear()
                net_results.clear()
                total_iterations = 0
                # Reset counters and start from first net
                current_net_idx = 0
                successful = 0
                failed = 0
                while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
                    failed += 1
                    current_net_idx += 1
                print("\nRestarting all nets...")
                continue

            # Check for restart request (restart current net)
            if visualizer.restart_requested:
                visualizer.restart_requested = False
                waiting_for_next = False  # Cancel any wait state
                # Remove any geometry added by this net before restarting
                if current_net_idx in added_geometry:
                    remove_net_geometry(current_net_idx)
                    # Also remove from visualizer's completed routes
                    visualizer.remove_last_completed_route()
                # Re-setup the current net
                if current_net_idx < len(net_queue):
                    setup_net(current_net_idx)
                    print(f"\nRestarting {net_queue[current_net_idx][0]}...")

            # Check for B key to force backwards direction
            if visualizer.backwards_requested:
                visualizer.backwards_requested = False
                waiting_for_next = False  # Cancel any wait state
                # Remove any geometry added by this net
                if current_net_idx in added_geometry:
                    remove_net_geometry(current_net_idx)
                    # Also remove from visualizer's completed routes
                    visualizer.remove_last_completed_route()
                # Re-setup the current net but immediately switch to backwards
                if current_net_idx < len(net_queue):
                    setup_net(current_net_idx)
                    net_name, _ = net_queue[current_net_idx]
                    print(f"\nForcing backwards direction for {net_name}...")
                    visualizer.status_message = "Trying backwards direction..."
                    # Swap sources and targets
                    sources, targets = original_targets, original_sources
                    forward_done = True  # Mark forward as done so we know we're in backwards mode
                    reverse_done = True  # Mark reverse as done so N goes to next net after this
                    router = VisualRouter(via_cost=routing_config.via_cost * 1000, h_weight=routing_config.heuristic_weight)
                    router.init(sources, targets, routing_config.max_iterations)
                continue

            # Advance search if not paused and not done
            if visualizer.should_step() and router and not router.is_done():
                # Run multiple iterations using Rust router
                snapshot = router.step(obstacles, visualizer.iterations_per_frame)
                visualizer.update_snapshot(snapshot)

                if snapshot.found:
                    net_name, net_id = net_queue[current_net_idx]
                    current_net_iterations += snapshot.iteration
                    path_len = len(snapshot.path) if snapshot.path else 0
                    direction = "back" if forward_done else "fwd"
                    print(f"\nPath found for {net_name} in {current_net_iterations} iterations ({direction})!")
                    print(f"Path length: {path_len}")
                    print("Press N to continue to next net, or B to try backwards")
                    visualizer.status_message = f"Path found ({direction}) - N=next, B=try backwards"
                    successful += 1
                    total_iterations += current_net_iterations
                    net_results.append((net_name, current_net_iterations, path_len, True, direction))

                    # Add path to pcb_data for subsequent routes
                    if snapshot.path:
                        add_path_to_pcb(snapshot.path, net_id, current_net_idx)
                        # Also add to visualizer for persistent display
                        visualizer.add_completed_route(snapshot.path)

                    # Update tracking lists to match batch router behavior
                    if net_id in remaining_net_ids:
                        remaining_net_ids.remove(net_id)
                    if net_id not in routed_net_ids:
                        routed_net_ids.append(net_id)

                    # Route succeeded - mark both directions done so N goes to next net
                    forward_done = True
                    reverse_done = True

                    # Wait for user to press N before next net
                    waiting_for_next = True

            # Check for N key to advance to next net or try reverse
            # In auto_advance mode, wait for display_time before proceeding
            if waiting_for_next:
                # Start display timer when entering wait state
                if display_wait_start is None:
                    display_wait_start = time.time()

                # Check if we should advance
                should_auto_advance = False
                if auto_advance:
                    elapsed = time.time() - display_wait_start
                    if elapsed >= display_time:
                        should_auto_advance = True
                    else:
                        # Update status message with countdown
                        remaining = display_time - elapsed
                        visualizer.status_message = f"Displaying route... ({remaining:.1f}s)"

                if not (visualizer.next_net_requested or should_auto_advance):
                    # Keep rendering while waiting
                    visualizer.render()
                    visualizer.tick()
                    continue

                visualizer.next_net_requested = False
                waiting_for_next = False
                display_wait_start = None  # Reset timer for next net

                # Check if we need to try backwards direction (forward failed, haven't tried backwards yet)
                if forward_done and not reverse_done:
                    # Forward failed, now try backwards
                    print(f"\nTrying backwards direction...")
                    visualizer.status_message = "Trying backwards direction..."
                    sources, targets = original_targets, original_sources  # Swap
                    router = VisualRouter(via_cost=routing_config.via_cost * 1000, h_weight=routing_config.heuristic_weight)
                    router.init(sources, targets, routing_config.max_iterations)
                    continue

                # Move to next net
                current_net_idx += 1
                while current_net_idx < len(net_queue) and not setup_net(current_net_idx):
                    failed += 1
                    current_net_idx += 1

                if current_net_idx >= len(net_queue):
                    print(f"\n{'='*78}")
                    print(f"All nets processed: {successful} successful, {failed} failed")
                    print(f"Total iterations: {total_iterations}")
                    print(f"\nResults summary:")
                    print(f"{'Net Name':<30} {'Iterations':>12} {'Path Len':>10} {'Dir':>6} {'Status':>8}")
                    print("-" * 78)
                    for name, iters, path_len, success, direction in net_results:
                        status = "OK" if success else "FAIL"
                        # Shorten net name if too long
                        short_name = name if len(name) <= 28 else "..." + name[-25:]
                        print(f"{short_name:<30} {iters:>12} {path_len:>10} {direction:>6} {status:>8}")
                    print("-" * 78)
                    print(f"{'TOTAL':<30} {total_iterations:>12}")
                    if auto_advance:
                        # Auto quit when done in auto mode
                        visualizer.running = False
                    else:
                        print("\nPress Q to quit or close window")
                    router = None  # Stop routing
                # Continue to next iteration (whether we set up a new net or finished all)
                continue

            # Check if current route is done (found or exhausted)
            if router and router.is_done() and not waiting_for_next:
                snapshot = visualizer.snapshot
                if snapshot and not snapshot.found:
                    # Track iterations from this attempt
                    current_net_iterations += snapshot.iteration
                    net_name, _ = net_queue[current_net_idx]

                    # Forward failed -> wait for N key to try backwards
                    if not forward_done:
                        forward_done = True
                        print(f"\nForward failed after {snapshot.iteration} iterations")
                        print("Press N to try backwards direction...")
                        visualizer.status_message = "Forward failed - Press N to try backwards"
                        waiting_for_next = True  # Wait for N key
                    else:
                        # Backwards also failed
                        reverse_done = True
                        print(f"\nBackwards also failed after {snapshot.iteration} iterations")
                        print(f"No path found for {net_name} after {current_net_iterations} total iterations")
                        print("Press N to continue to next net")
                        visualizer.status_message = "Both directions failed - Press N for next net"
                        failed += 1
                        total_iterations += current_net_iterations
                        net_results.append((net_name, current_net_iterations, 0, False, "both"))
                        waiting_for_next = True

            # Render
            visualizer.render()
            visualizer.tick()

    finally:
        visualizer.quit()

    return successful, failed, total_iterations, net_results


def expand_net_pattern(pcb_data: PCBData, pattern: str) -> List[str]:
    """
    Expand a wildcard pattern to matching net names.

    Patterns can include * and ? wildcards (fnmatch style).
    Example: "Net-(U2A-DATA_*)" matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.

    Returns list of matching net names in sorted order.
    """
    # Collect net names from both pcb.nets and pads_by_net
    all_net_names = set(net.name for net in pcb_data.nets.values())
    # Also include net names from pads (for nets not in pcb.nets)
    for pads in pcb_data.pads_by_net.values():
        for pad in pads:
            if pad.net_name:
                all_net_names.add(pad.net_name)
                break  # Only need one pad's net_name per net
    all_net_names = list(all_net_names)

    if '*' in pattern or '?' in pattern:
        matches = sorted([name for name in all_net_names if fnmatch.fnmatch(name, pattern)])
        return matches
    else:
        # Literal net name
        return [pattern] if pattern in all_net_names else []


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Visualize PCB routing with A* algorithm")
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("net_patterns", nargs="+", help="Net names or wildcard patterns")

    # Visualizer-specific options
    parser.add_argument("--auto", action="store_true",
                        help="Automatically advance to next net (no waiting for N key)")
    parser.add_argument("--display-time", type=float, default=0.0,
                        help="Time in seconds to display completed route before advancing (default: 0.0)")

    # Ordering and strategy options (same as batch_grid_router)
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="mps",
                        help="Net ordering strategy: mps (default, crossing conflicts), inside_out, or original")
    parser.add_argument("--no-bga-zones", action="store_true",
                        help="Disable BGA exclusion zone detection")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                        help="Routing layers to use (default: F.Cu In1.Cu In2.Cu B.Cu)")

    # Track and via geometry (same as batch_grid_router)
    parser.add_argument("--track-width", type=float, default=0.1,
                        help="Track width in mm (default: 0.1)")
    parser.add_argument("--clearance", type=float, default=0.1,
                        help="Clearance between tracks in mm (default: 0.1)")
    parser.add_argument("--via-size", type=float, default=0.3,
                        help="Via outer diameter in mm (default: 0.3)")
    parser.add_argument("--via-drill", type=float, default=0.2,
                        help="Via drill size in mm (default: 0.2)")

    # Router algorithm parameters (same as batch_grid_router)
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=25,
                        help="Penalty for placing a via in grid steps (default: 25)")
    parser.add_argument("--max-iterations", type=int, default=100000,
                        help="Max A* iterations before giving up (default: 100000)")
    parser.add_argument("--heuristic-weight", type=float, default=1.5,
                        help="A* heuristic weight (default: 1.5)")

    # Stub proximity penalty (same as batch_grid_router)
    parser.add_argument("--stub-proximity-radius", type=float, default=1.5,
                        help="Radius around stubs to penalize routing in mm (default: 1.5)")
    parser.add_argument("--stub-proximity-cost", type=float, default=2.0,
                        help="Cost penalty near stubs in mm equivalent (default: 2.0)")

    args = parser.parse_args()

    if not os.path.exists(args.input_file):
        print(f"File not found: {args.input_file}")
        sys.exit(1)

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file}...")
    pcb_data = parse_kicad_pcb(args.input_file)

    # Expand all patterns
    all_nets = []
    seen = set()
    for pattern in args.net_patterns:
        matching_nets = expand_net_pattern(pcb_data, pattern)
        if not matching_nets:
            print(f"Warning: Pattern '{pattern}' matched no nets")
        else:
            if '*' in pattern or '?' in pattern:
                print(f"Pattern '{pattern}' matched {len(matching_nets)} nets")
            for net in matching_nets:
                if net not in seen:
                    all_nets.append(net)
                    seen.add(net)

    if not all_nets:
        print("No nets matched the given patterns!")
        print("\nAvailable nets:")
        for nid, net in list(pcb_data.nets.items())[:20]:
            print(f"  {net.name}")
        if len(pcb_data.nets) > 20:
            print(f"  ... and {len(pcb_data.nets) - 20} more")
        sys.exit(1)

    mode_info = []
    if args.auto:
        mode_info.append("auto-advance")
        if args.display_time > 0:
            mode_info.append(f"display={args.display_time}s")
    if args.no_bga_zones:
        mode_info.append("no BGA zones")
    if args.ordering != "mps":
        mode_info.append(f"ordering={args.ordering}")
    mode_str = f" ({', '.join(mode_info)})" if mode_info else ""
    print(f"\nWill visualize routing of {len(all_nets)} nets{mode_str}")

    run_visualization(
        args.input_file, all_nets,
        layers=args.layers,
        grid_step=args.grid_step,
        max_iterations=args.max_iterations,
        auto_advance=args.auto,
        display_time=args.display_time,
        disable_bga_zones=args.no_bga_zones,
        ordering_strategy=args.ordering,
        track_width=args.track_width,
        clearance=args.clearance,
        via_size=args.via_size,
        via_drill=args.via_drill,
        via_cost=args.via_cost,
        heuristic_weight=args.heuristic_weight,
        stub_proximity_radius=args.stub_proximity_radius,
        stub_proximity_cost=args.stub_proximity_cost
    )


if __name__ == "__main__":
    main()

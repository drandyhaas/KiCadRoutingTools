"""
Zone geometry utilities for copper plane generation.

The Voronoi cells a shared plane layer is split from (route_planes' spine split), polygon clipping, and route
sampling for the cells' seeds.
"""
from __future__ import annotations

import math
from typing import List, Dict, Tuple

import numpy as np

from scipy.spatial import Voronoi


def voronoi_cells(
    vias_by_net: Dict[int, List[Tuple[float, float]]],
    board_bounds: Tuple[float, float, float, float],
    board_edge_clearance: float = 0.0
) -> Dict[int, List[List[Tuple[float, float]]]]:
    """
    The Voronoi cell of every seed point, by its net.

    Algorithm:
    1. Create a Voronoi cell around EACH seed (not a net's centroid)
    2. Label each cell with its seed's net_id
    3. Clip cells to board bounds (inset by board_edge_clearance)

    Args:
        vias_by_net: Dict mapping net_id -> list of (x, y) seed positions
        board_bounds: Tuple of (min_x, min_y, max_x, max_y)
        board_edge_clearance: Clearance from board edge for the cells (mm)

    Returns:
        Dict mapping net_id -> list of cells, each a list of (x, y) vertices

    Raises:
        ValueError: fewer than two seeds in all
    """
    min_x, min_y, max_x, max_y = board_bounds

    # Apply board edge clearance (inset the clipping bounds)
    clip_min_x = min_x + board_edge_clearance
    clip_min_y = min_y + board_edge_clearance
    clip_max_x = max_x - board_edge_clearance
    clip_max_y = max_y - board_edge_clearance

    # Collect all vias into a single list with net labels
    all_vias = []
    via_net_ids = []
    for net_id, positions in vias_by_net.items():
        for pos in positions:
            all_vias.append(pos)
            via_net_ids.append(net_id)

    if len(all_vias) < 2:
        raise ValueError(f"a Voronoi split needs two seeds, got {len(all_vias)}")

    # Add mirror points outside board bounds to ensure all regions are finite
    # This is a standard technique for bounded Voronoi
    pad = max(max_x - min_x, max_y - min_y) * 2
    mirror_points = [
        (min_x - pad, min_y - pad),
        (max_x + pad, min_y - pad),
        (min_x - pad, max_y + pad),
        (max_x + pad, max_y + pad),
        ((min_x + max_x) / 2, min_y - pad),
        ((min_x + max_x) / 2, max_y + pad),
        (min_x - pad, (min_y + max_y) / 2),
        (max_x + pad, (min_y + max_y) / 2),
    ]

    points = np.array(all_vias + mirror_points)
    vor = Voronoi(points)

    # Build polygon for each real via (not mirror points)
    via_polygons: Dict[int, List[List[Tuple[float, float]]]] = {net_id: [] for net_id in vias_by_net}

    for via_idx in range(len(all_vias)):
        region_idx = vor.point_region[via_idx]
        region = vor.regions[region_idx]

        if -1 in region or len(region) == 0:
            # Infinite region (shouldn't happen with mirror points, but handle it)
            continue

        # Get polygon vertices
        polygon = [tuple(vor.vertices[i]) for i in region]

        # Clip polygon to board bounds (with edge clearance applied)
        clipped = clip_polygon_to_rect(polygon, clip_min_x, clip_min_y, clip_max_x, clip_max_y)

        if clipped and len(clipped) >= 3:
            via_polygons[via_net_ids[via_idx]].append(clipped)

    return via_polygons


def clip_polygon_to_rect(
    polygon: List[Tuple[float, float]],
    min_x: float, min_y: float, max_x: float, max_y: float
) -> List[Tuple[float, float]]:
    """
    Clip a polygon to a rectangle using Sutherland-Hodgman algorithm.
    """
    def inside_edge(p, edge):
        """Check if point p is inside the clipping edge."""
        x, y = p
        if edge == 'left':
            return x >= min_x
        elif edge == 'right':
            return x <= max_x
        elif edge == 'bottom':
            return y >= min_y
        elif edge == 'top':
            return y <= max_y
        return True

    def intersect_edge(p1, p2, edge):
        """Find intersection of line p1-p2 with clipping edge."""
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1

        if edge == 'left':
            if abs(dx) < 1e-10:
                return (min_x, y1)
            t = (min_x - x1) / dx
            return (min_x, y1 + t * dy)
        elif edge == 'right':
            if abs(dx) < 1e-10:
                return (max_x, y1)
            t = (max_x - x1) / dx
            return (max_x, y1 + t * dy)
        elif edge == 'bottom':
            if abs(dy) < 1e-10:
                return (x1, min_y)
            t = (min_y - y1) / dy
            return (x1 + t * dx, min_y)
        elif edge == 'top':
            if abs(dy) < 1e-10:
                return (x1, max_y)
            t = (max_y - y1) / dy
            return (x1 + t * dx, max_y)
        return p1

    output = polygon
    for edge in ['left', 'right', 'bottom', 'top']:
        if not output:
            return []
        input_poly = output
        output = []

        for i in range(len(input_poly)):
            p1 = input_poly[i]
            p2 = input_poly[(i + 1) % len(input_poly)]

            p1_inside = inside_edge(p1, edge)
            p2_inside = inside_edge(p2, edge)

            if p1_inside and p2_inside:
                output.append(p2)
            elif p1_inside and not p2_inside:
                output.append(intersect_edge(p1, p2, edge))
            elif not p1_inside and p2_inside:
                output.append(intersect_edge(p1, p2, edge))
                output.append(p2)
            # else: both outside, add nothing

    return output


def sample_route_for_voronoi(
    route_path: List[Tuple[float, float]],
    sample_interval: float = 2.0
) -> List[Tuple[float, float]]:
    """
    Sample points along a route path for Voronoi seeding.

    Args:
        route_path: List of (x, y) points along the route
        sample_interval: Distance between samples in mm

    Returns:
        List of (x, y) sample points along the route
    """
    if not route_path or len(route_path) < 2:
        return []

    samples = []
    accumulated_dist = 0.0

    for i in range(len(route_path) - 1):
        x1, y1 = route_path[i]
        x2, y2 = route_path[i + 1]

        seg_len = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        if seg_len < 0.001:
            continue

        # Direction vector
        dx = (x2 - x1) / seg_len
        dy = (y2 - y1) / seg_len

        # Sample along this segment
        dist_in_seg = 0.0
        while dist_in_seg < seg_len:
            # Check if we need to place a sample
            if accumulated_dist >= sample_interval:
                # Place sample
                x = x1 + dx * dist_in_seg
                y = y1 + dy * dist_in_seg
                samples.append((x, y))
                accumulated_dist = 0.0

            # Step forward
            step = min(sample_interval - accumulated_dist, seg_len - dist_in_seg)
            dist_in_seg += step
            accumulated_dist += step

    return samples

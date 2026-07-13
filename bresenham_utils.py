"""Bresenham line-walking utilities for grid-based operations.

This module consolidates all Bresenham line algorithm implementations to avoid
code duplication across obstacle_map.py, obstacle_cache.py, blocking_analysis.py,
obstacle_costs.py, plane_blocker_detection.py, and plane_region_connector.py.
"""
from __future__ import annotations

from typing import Generator, Tuple


def walk_line(gx1: int, gy1: int, gx2: int, gy2: int) -> Generator[Tuple[int, int], None, None]:
    """Walk along a line from (gx1, gy1) to (gx2, gy2), yielding each grid point.

    Uses the optimized Bresenham line algorithm with the e2 = 2*err variant.
    Yields all grid points along the line, including both endpoints.

    Args:
        gx1, gy1: Starting grid coordinates
        gx2, gy2: Ending grid coordinates

    Yields:
        (gx, gy) tuples for each point along the line
    """
    dx = abs(gx2 - gx1)
    dy = abs(gy2 - gy1)
    sx = 1 if gx1 < gx2 else -1
    sy = 1 if gy1 < gy2 else -1
    err = dx - dy

    gx, gy = gx1, gy1
    while True:
        yield gx, gy

        if gx == gx2 and gy == gy2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            gx += sx
        if e2 < dx:
            err += dx
            gy += sy


def is_diagonal_segment(gx1: int, gy1: int, gx2: int, gy2: int) -> bool:
    """Check if a segment is diagonal (not axis-aligned)."""
    return abs(gx2 - gx1) > 0 and abs(gy2 - gy1) > 0



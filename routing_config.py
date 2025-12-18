"""
Configuration classes and coordinate utilities for PCB routing.
"""

import math
from typing import List, Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class DiffPair:
    """Represents a differential pair with P and N nets."""
    base_name: str  # Common name without _P/_N suffix
    p_net_id: Optional[int] = None
    n_net_id: Optional[int] = None
    p_net_name: Optional[str] = None
    n_net_name: Optional[str] = None

    @property
    def is_complete(self) -> bool:
        return self.p_net_id is not None and self.n_net_id is not None


@dataclass
class GridRouteConfig:
    """Configuration for grid-based routing."""
    track_width: float = 0.1  # mm
    clearance: float = 0.1  # mm between tracks
    via_size: float = 0.3  # mm via outer diameter
    via_drill: float = 0.2  # mm via drill
    grid_step: float = 0.1  # mm grid resolution
    via_cost: int = 25  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 100000
    heuristic_weight: float = 1.5
    # BGA exclusion zones (auto-detected from PCB) - vias blocked inside these areas
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stub_proximity_radius: float = 1.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 3.0  # mm equivalent cost at stub center
    # Direction search order: "forward", "backwards", or "random"
    direction_order: str = "forward"
    # Differential pair routing parameters
    diff_pair_gap: float = 0.1  # mm - gap between P and N traces (center-to-center = track_width + gap)
    min_diff_pair_centerline_setback: float = 0.6  # mm - minimum distance in front of stubs to start centerline route
    max_diff_pair_centerline_setback: float = 5.0  # mm - max setback to try if min is blocked
    diff_pair_turn_length: float = 0.3  # mm - length of turn segments at start/end of diff pair routes
    fix_polarity: bool = False  # Swap target pad nets if polarity swap needed
    debug_lines: bool = False  # Output debug geometry on User.2/3/8/9 layers


class GridCoord:
    """Utilities for converting between float (mm) and integer grid coordinates."""
    def __init__(self, grid_step: float = 0.1):
        self.grid_step = grid_step
        self.inv_step = 1.0 / grid_step

    def to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert float mm coordinates to integer grid coordinates."""
        return (round(x * self.inv_step), round(y * self.inv_step))

    def to_float(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convert integer grid coordinates to float mm coordinates."""
        return (gx * self.grid_step, gy * self.grid_step)

    def to_grid_dist(self, dist_mm: float) -> int:
        """Convert a distance in mm to grid units."""
        return round(dist_mm * self.inv_step)

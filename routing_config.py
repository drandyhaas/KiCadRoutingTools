"""
Configuration classes and coordinate utilities for PCB routing.
"""

import math
from typing import List, Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class DiffPairNet:
    """Represents a differential pair with P and N nets (tracks net IDs)."""
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
    via_cost: int = 50  # grid steps equivalent penalty for via
    layers: List[str] = field(default_factory=lambda: ['F.Cu', 'B.Cu'])
    max_iterations: int = 200000
    max_probe_iterations: int = 5000  # Quick probe per direction to detect stuck routes
    heuristic_weight: float = 1.9
    turn_cost: int = 1000  # Penalty for direction changes (encourages straighter paths)
    # BGA exclusion zones (auto-detected from PCB) - vias blocked inside these areas
    bga_exclusion_zones: List[Tuple[float, float, float, float]] = field(default_factory=list)
    stub_proximity_radius: float = 2.0  # mm - radius around stubs to penalize
    stub_proximity_cost: float = 0.2  # mm equivalent cost at stub center
    via_proximity_cost: float = 10.0  # via cost multiplier in stub/BGA proximity zones (0 = block vias)
    bga_proximity_radius: float = 10.0  # mm - distance from BGA edges to penalize
    bga_proximity_cost: float = 0.2  # mm equivalent cost at BGA edge
    # Direction search order: "forward", "backward", or "random"
    direction_order: str = "forward"
    # Differential pair routing parameters
    diff_pair_gap: float = 0.101  # mm - gap between P and N traces (center-to-center = track_width + gap)
    diff_pair_centerline_setback: float = None  # mm - distance in front of stubs to start centerline route (None = 2 * spacing)
    min_turning_radius: float = 0.2  # mm - minimum turning radius for pose-based routing
    fix_polarity: bool = True  # Swap target pad nets if polarity swap needed
    debug_lines: bool = False  # Output debug geometry on User.2/3/8/9 layers
    verbose: bool = False  # Print detailed diagnostic output
    max_rip_up_count: int = 3  # Maximum blockers to rip up at once during rip-up and retry (1 to N)
    max_setback_angle: float = 45.0  # Maximum angle (degrees) for setback position search
    track_proximity_distance: float = 2.0  # mm - radius around routed tracks to penalize (same layer)
    stub_layer_swap: bool = True  # Enable stub layer switching optimization
    track_proximity_cost: float = 0.2  # mm equivalent cost at track center
    target_swap_crossing_penalty: float = 1000.0  # Penalty for crossing assignments in target swap
    crossing_layer_check: bool = True  # Only count crossings when routes share a layer
    routing_clearance_margin: float = 1.15  # Multiplier on track-via clearance (1.0 = minimum DRC, 1.15 = safe)
    max_turn_angle: float = 180.0  # Max cumulative turn angle (degrees) before reset, to prevent U-turns
    gnd_via_enabled: bool = True  # Enable GND via placement near diff pair signal vias
    # Vertical alignment attraction - encourages tracks on different layers to stack
    vertical_attraction_radius: float = 0.2  # mm - radius for attraction lookup (0 = disabled)
    vertical_attraction_cost: float = 0.1  # mm equivalent bonus for aligned positions


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

    def to_grid_dist_safe(self, dist_mm: float) -> int:
        """Convert a distance in mm to grid units with a small safety margin.

        Adds half a grid step to the distance before rounding to account for
        discretization effects that can cause minor DRC violations.
        """
        return round((dist_mm + self.grid_step * 0.5) * self.inv_step)

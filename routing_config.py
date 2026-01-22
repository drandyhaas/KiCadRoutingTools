"""
Configuration classes and coordinate utilities for PCB routing.
"""

import math
from typing import List, Optional, Tuple, Dict
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
    bga_proximity_radius: float = 7.0  # mm - distance from BGA edges to penalize
    bga_proximity_cost: float = 0.2  # mm equivalent cost at BGA edge
    # Direction search order: "forward" or "backward"
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
    routing_clearance_margin: float = 1.0  # Multiplier on track-via clearance (1.0 = minimum DRC)
    hole_to_hole_clearance: float = 0.2  # mm - minimum clearance between drill holes (edge to edge)
    board_edge_clearance: float = 0.0  # mm - clearance from board edge (0 = use track clearance)
    max_turn_angle: float = 180.0  # Max cumulative turn angle (degrees) before reset, to prevent U-turns
    gnd_via_enabled: bool = True  # Enable GND via placement near diff pair signal vias
    # Vertical alignment attraction - encourages tracks on different layers to stack
    vertical_attraction_radius: float = 0.2  # mm - radius for attraction lookup (0 = disabled)
    vertical_attraction_cost: float = 0.1  # mm equivalent bonus for aligned positions
    # Length matching for DDR4 signals
    length_match_groups: List[List[str]] = field(default_factory=list)  # Groups of net patterns to match
    length_match_tolerance: float = 0.1  # mm - acceptable length variance within group
    meander_amplitude: float = 1.0  # mm - height of meander perpendicular to trace
    diff_chamfer_extra: float = 1.5  # Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings)
    diff_pair_intra_match: bool = False  # Enable intra-pair P/N length matching (meander shorter track)
    debug_memory: bool = False  # Print memory usage statistics at key points
    # Impedance-controlled routing
    impedance_target: Optional[float] = None  # Target impedance in ohms (None = use fixed track_width)
    layer_widths: Dict[str, float] = field(default_factory=dict)  # Per-layer widths for impedance control
    # Power net routing - per-net width overrides
    power_net_widths: Dict[int, float] = field(default_factory=dict)  # net_id -> width in mm
    # Layer cost weights - prefer certain layers over others (1.0 = normal, 1.5 = 50% more expensive)
    layer_costs: List[float] = field(default_factory=list)  # Per-layer cost multipliers

    def get_track_width(self, layer: str) -> float:
        """Get track width for a specific layer (impedance-aware).

        If impedance targeting is enabled and layer_widths is populated,
        returns the layer-specific width. Otherwise returns the default track_width.
        """
        if self.layer_widths and layer in self.layer_widths:
            return self.layer_widths[layer]
        return self.track_width

    def get_max_track_width(self) -> float:
        """Get the maximum track width across all layers.

        Used for via clearance calculations where we need to ensure clearance
        for the widest possible track (e.g., when a via connects two layers
        with different impedance-controlled widths).
        """
        if self.layer_widths:
            return max(self.layer_widths.values(), default=self.track_width)
        return self.track_width

    def get_net_track_width(self, net_id: int, layer: str) -> float:
        """Get track width for a specific net on a specific layer.

        Priority order:
        1. Per-net power width override (power_net_widths)
        2. Layer-specific width (layer_widths, for impedance control)
        3. Default track_width

        The returned width is always at least track_width (power net widths
        cannot be smaller than the base track width).

        Args:
            net_id: The net ID to get width for
            layer: The layer name

        Returns:
            Track width in mm (never less than track_width)
        """
        if net_id in self.power_net_widths:
            # Ensure power net width is at least the base track width
            return max(self.power_net_widths[net_id], self.track_width)
        return self.get_track_width(layer)

    def get_layer_costs(self) -> List[int]:
        """Get layer cost multipliers for the Rust router.

        Returns costs scaled by 1000 (1000 = 1.0x, 1500 = 1.5x penalty).
        If layer_costs is empty or shorter than layers list, uses 1000 (1.0x) for missing layers.
        """
        costs = []
        for i in range(len(self.layers)):
            if i < len(self.layer_costs):
                costs.append(int(self.layer_costs[i] * 1000))
            else:
                costs.append(1000)  # Default 1.0x
        return costs


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
        """Convert a distance in mm to grid units (rounds down)."""
        return int(dist_mm * self.inv_step)

    def to_grid_dist_safe(self, dist_mm: float) -> int:
        """Convert a distance in mm to grid units, rounding up for safety."""
        return math.ceil(dist_mm * self.inv_step)

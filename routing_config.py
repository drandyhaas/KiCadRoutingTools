"""
Configuration classes and coordinate utilities for PCB routing.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field

from routing_constants import FORBIDDEN_LAYER_COST

# Cost knobs (proximity costs, via_cost, attraction bonuses) are calibrated at
# this grid step: GridRouteConfig.cell_cost / via_cost_units scale them so the
# cost per mm of path is the same at any --grid-step, and identical to
# historical behavior at 0.1mm. This is a FIXED calibration baseline, NOT the
# grid-step default (routing_defaults.GRID_STEP) -- it must stay 0.1 even if the
# default grid changes, so the two are intentionally separate constants.
REFERENCE_GRID_STEP = 0.1  # mm


@dataclass
class DiffPairNet:
    """Represents a differential pair with P and N nets (tracks net IDs)."""
    base_name: str  # Common name without _P/_N suffix
    p_net_id: Optional[int] = None
    n_net_id: Optional[int] = None
    p_net_name: Optional[str] = None
    n_net_name: Optional[str] = None
    # Whether a P/N polarity pad swap may be applied to THIS pair (#279).
    # Set by batch_route_diff_pairs from --polarity-swap-nets; a swap is only
    # harmless when an endpoint can compensate (FPGA generic I/O, polarity-
    # tolerant protocol), so the default is deny.
    polarity_swap_allowed: bool = False

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
    # NOTE (soft-knobs C6): in the Rust via branch this also MULTIPLIES the
    # summed stub+layer proximity at the via site (track-proximity and
    # ripped-corridor soft costs included), not just stub/BGA-zone costs.
    via_proximity_cost: float = 10.0  # via cost multiplier in stub/BGA proximity zones (0 = block vias)
    bga_proximity_radius: float = 7.0  # mm - distance from BGA edges to penalize
    bga_proximity_cost: float = 0.2  # mm equivalent cost at BGA edge
    # Direction search order: "forward" or "backward"
    direction_order: str = "forward"
    # Differential pair routing parameters
    diff_pair_gap: float = 0.101  # mm - gap between P and N traces (center-to-center = track_width + gap)
    diff_pair_centerline_setback: float = None  # mm - distance in front of stubs to start centerline route (None = 2 * spacing)
    diff_pair_setback_no_ladder: bool = False  # when True, _setback_ladder yields ONLY
    # the configured setback (no 0.75/0.5/floor/1.5/2x expansion) -- used by the pinch
    # retry in _maybe_swap_to_hybrid so each attempt routes at the EXACT setback asked.
    # In a multi-point pair, a "terminal" whose P and N pads are farther apart
    # than diff_pair_uncouple_factor * (track_width + diff_pair_gap) is not a
    # coupled differential connection (e.g. spread-out test points). If the full
    # coupled chain can't be routed, such terminals are peeled off and their pads
    # routed single-ended (P->P, N->N) instead (issue #121).
    diff_pair_uncouple_factor: float = 6.0  # multiples of pair spacing (track+gap)
    min_turning_radius: float = 0.2  # mm - minimum turning radius for pose-based routing
    debug_lines: bool = False  # Output debug geometry on User.2/3/8/9 layers
    verbose: bool = False  # Print detailed diagnostic output
    max_rip_up_count: int = 3  # Maximum blockers to rip up at once during rip-up and retry (1 to N)
    # How the #85 arbitration decides keep-retry vs abandon after a Phase 3
    # tap rip-up cascade (docs/rip-up-reroute.md "Abandon metrics"). One of
    # phase3_routing.ABANDON_METRICS: stranded | total-pads | complete-nets |
    # congestion | history | weighted | probe | weighted-probe
    ripup_abandon_metric: str = 'stranded'
    max_setback_angle: float = 45.0  # Maximum angle (degrees) for setback position search
    track_proximity_distance: float = 2.0  # mm - radius around routed tracks to penalize (same layer)
    stub_layer_swap: bool = True  # Enable stub layer switching optimization
    track_proximity_cost: float = 0.0  # mm equivalent cost (0 = disabled)
    target_swap_crossing_penalty: float = 1000.0  # Penalty for crossing assignments in target swap
    crossing_layer_check: bool = True  # Only count crossings when routes share a layer
    routing_clearance_margin: float = 1.0  # Multiplier on track-via clearance (1.0 = minimum DRC)
    hole_to_hole_clearance: float = 0.20  # mm - edge-to-edge via drill spacing; JLC "Via
                                           # Hole-to-Hole Spacing" (keep in sync with
                                           # routing_defaults.HOLE_TO_HOLE_CLEARANCE)
    board_edge_clearance: float = 0.0  # mm - clearance from board edge (0 = use track clearance)
    max_turn_angle: float = 180.0  # Max cumulative turn angle (degrees) before reset, to prevent U-turns
    # Power-tap neck-down (issue #72): when a wide power-net tap edge fails,
    # retry it at the layer's default track width. The narrow neck extends
    # neckdown_length mm from the target pad; beyond that the track returns
    # to the power width wherever the wide clearance fits.
    power_tap_neckdown: bool = True
    neckdown_length: float = 2.5  # mm of narrow track from the target pad
    neckdown_taper_length: float = 0.5  # mm narrow->wide taper (0 = abrupt width step)
    gnd_via_enabled: bool = True  # Enable GND via placement near diff pair signal vias
    # Vertical alignment attraction - encourages tracks on different layers to stack
    vertical_attraction_radius: float = 1.0  # mm - radius for attraction lookup (0 = disabled); matches routing_defaults.VERTICAL_ATTRACTION_RADIUS (N1)
    vertical_attraction_cost: float = 0.0  # mm equivalent bonus for aligned positions
    # Ripped route avoidance - soft penalty for routing through a ripped net's former corridor
    ripped_route_avoidance_radius: float = 1.0  # mm - radius around ripped route segments/vias
    ripped_route_avoidance_cost: float = 0.1  # mm equivalent cost (0 = disabled)
    # Length matching for DDR4 signals
    length_match_groups: List[List[str]] = field(default_factory=list)  # Groups of net patterns to match
    length_match_tolerance: float = 0.1  # mm - acceptable length variance within group
    meander_amplitude: float = 1.0  # mm - height of meander perpendicular to trace
    diff_chamfer_extra: float = 1.5  # Chamfer multiplier for diff pair meanders (>1 avoids P/N crossings)
    diff_pair_intra_match: bool = False  # Enable intra-pair P/N length matching (meander shorter track)
    ac_couple_match: bool = False  # End-to-end length-match AC-coupled pairs split by series caps (#196)
    # Hybrid escape: when a coupled pair's terminal connector can't clear foreign
    # copper (#165 graze), keep the coupled middle and defer each terminal leg to
    # a point-to-point single-ended join instead of failing the whole pair.
    diff_pair_hybrid_escape: bool = True
    # Time matching (alternative to length matching) - matches propagation delay instead of length
    time_matching: bool = False  # If True, match by propagation time instead of length
    time_match_tolerance: float = 1.0  # ps - acceptable time variance within group
    debug_memory: bool = False  # Print memory usage statistics at key points
    # Output options
    add_teardrops: bool = False  # Add teardrop settings to all pads in output file
    # Impedance-controlled routing
    impedance_target: Optional[float] = None  # Target impedance in ohms (None = use fixed track_width)
    layer_widths: Dict[str, float] = field(default_factory=dict)  # Per-layer widths for impedance control
    # Power net routing - per-net width overrides
    power_net_widths: Dict[int, float] = field(default_factory=dict)  # net_id -> width in mm
    # Per-net netclass track width (auto-read from the .kicad_pro when --track-width
    # is omitted). Unlike power_net_widths this is the net's OWN class width and may
    # be SMALLER than the global track_width (a narrower class), floored at the fab
    # minimum by the caller. Lower priority than a manual power_net_widths override.
    net_track_widths: Dict[int, float] = field(default_factory=dict)  # net_id -> width in mm
    # Layer cost weights - prefer certain layers over others (1.0 = normal, 1.5 = 50% more expensive)
    layer_costs: List[float] = field(default_factory=list)  # Per-layer cost multipliers
    # Debug options
    collect_stats: bool = False  # Collect A* search statistics for debugging
    # Heuristic tuning
    proximity_heuristic_factor: float = 0.02  # Factor for proximity heuristic (higher = tighter heuristic, faster but may overestimate)
    # Layer direction preference - alternates H/V starting with horizontal on top
    direction_preference_cost: int = 50  # Cost penalty for non-preferred direction (0 = disabled)
    # Bus routing - auto-detection and parallel routing of grouped nets
    bus_enabled: bool = False  # Enable bus detection and routing
    bus_detection_radius: float = 5.0  # mm - max endpoint distance to form bus
    bus_min_nets: int = 2  # Minimum nets to form a bus
    bus_attraction_radius: float = 5.0  # mm - attraction radius from neighbor track
    bus_attraction_bonus: int = 5000  # Cost bonus for staying near neighbor
    # Guide corridor - route selected nets through a user-drawn polyline (issue #7)
    guide_corridor_enabled: bool = False  # Steer routed nets along a drawn guide path
    guide_corridor_layer: str = "User.1"  # User layer the guide polyline is drawn on
    guide_corridor_spacing: float = 0.0  # mm; 0 = endpoints only, else subdivide long segments
    corridor_waypoints: List[Tuple[int, int]] = field(default_factory=list)  # prebuilt grid waypoints
    # Keepout zone - keep routed tracks out of a user-drawn polygon (issue #27)
    keepout_enabled: bool = False  # Block routed tracks from a drawn keepout polygon
    keepout_layer: str = "User.2"  # User layer the keepout polygon is drawn on
    # Cross-class clearance (KiCad semantics, issue: PR392). Each entry maps a
    # net_id to that net's own net-class clearance (mm). KiCad's required spacing
    # between two nets of different classes is max(classA, classB); the obstacle
    # maps price every foreign/in-run obstacle at obstacle_clearance() below.
    # Auto-read from the .kicad_pro netclasses by route.py/route_diff.py (or
    # supplied via --net-clearances); the GUI derives it from the live board.
    # net_clearance_floor is the routing-side floor (max clearance among the nets
    # being routed in THIS call, >= config.clearance); set at run start. An empty
    # map + None floor reproduces plain config.clearance behaviour exactly.
    # This also subsumes #326 B5: a net's OWN copper is stamped at
    # obstacle_clearance() = max(floor, its class), so every same-run sibling keeps
    # at least the class spacing to it (get_net_clearance() is the #326-only view).
    net_clearances: Dict[int, float] = field(default_factory=dict)
    net_clearance_floor: Optional[float] = None

    def obstacle_clearance(self, net_id: int) -> float:
        """KiCad cross-class clearance for an obstacle belonging to `net_id`.

        Returns max(routing-side floor, that obstacle net's own class clearance).
        The floor (net_clearance_floor) defaults to config.clearance, and an
        absent net falls back to config.clearance, so an empty net_clearances map
        yields exactly config.clearance -- byte-identical to pre-PR392 behaviour.
        Consumers (base map builder + every incremental obstacle stamper) MUST
        route their foreign-copper clearance through this one method so the ADD
        and REMOVE paths derive an identical per-obstacle value (ref-count
        symmetry, issue #208/#309)."""
        floor = self.net_clearance_floor if self.net_clearance_floor is not None else self.clearance
        return max(floor, self.net_clearances.get(net_id, self.clearance))

    def set_net_clearances(self, net_clearances, routed_net_ids) -> None:
        """Install the cross-class clearance map and compute the routing-side
        floor over the nets being routed in this call. Inert (floor == clearance)
        when the map is empty. Restricting the floor to the ROUTED nets keeps a
        foreign class from inflating it (which would over-block every routed
        net)."""
        self.net_clearances = dict(net_clearances) if net_clearances else {}
        if self.net_clearances and routed_net_ids:
            routed = [self.net_clearances[nid] for nid in routed_net_ids
                      if nid in self.net_clearances]
            self.net_clearance_floor = max([self.clearance] + routed)
        else:
            self.net_clearance_floor = self.clearance

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
        1. Per-net power width override (power_net_widths) -- floored UP to track_width
        2. Per-net netclass width (net_track_widths) -- the net's OWN class width,
           EXACTLY (may be narrower than the global track_width); floored at the fab
           minimum by the caller. Only populated when --track-width was omitted.
        3. Layer-specific width (layer_widths, for impedance control)
        4. Default track_width

        Args:
            net_id: The net ID to get width for
            layer: The layer name

        Returns:
            Track width in mm
        """
        if net_id in self.power_net_widths:
            # Ensure power net width is at least the base track width
            return max(self.power_net_widths[net_id], self.track_width)
        if self.net_track_widths and net_id in self.net_track_widths:
            # #435 companion: route this net at its OWN class width (either direction).
            return self.net_track_widths[net_id]
        return self.get_track_width(layer)

    def get_net_clearance(self, net_id: int) -> float:
        """Clearance for stamping THIS net's copper as an obstacle (#326 B5):
        its netclass clearance when above the global value, else the global
        clearance. Never smaller than `clearance` (netclasses only widen)."""
        nc = self.net_clearances.get(net_id, 0.0) if self.net_clearances else 0.0
        return nc if nc > self.clearance else self.clearance

    def get_layer_costs(self) -> List[int]:
        """Get layer cost multipliers for the Rust router.

        Returns costs scaled by 1000 (1000 = 1.0x, 1500 = 1.5x penalty).
        If layer_costs is empty or shorter than layers list, uses 1000 (1.0x) for missing layers.
        A cost of -1 (FORBIDDEN_LAYER_COST) is emitted VERBATIM (not scaled): the Rust
        router skips track placement on any layer whose cost is negative, while the layer
        stays an obstacle and through-vias may span it.
        """
        layer_costs = self.layer_costs or []  # may be None when set explicitly
        costs = []
        for i in range(len(self.layers)):
            if i < len(layer_costs):
                cost = layer_costs[i]
                if cost < 0:
                    # Forbidden: emit the canonical sentinel UNSCALED. The Rust
                    # router treats ANY negative entry as forbidden, so every
                    # negative input folds to one value here -- this also avoids
                    # int(cost * 1000) truncating a tiny negative in (-0.001, 0)
                    # up to 0 (a zero-cost layer, NOT forbidden).
                    costs.append(FORBIDDEN_LAYER_COST)
                else:
                    costs.append(int(cost * 1000))
            else:
                costs.append(1000)  # Default 1.0x
        return costs

    def get_layer_direction_preferences(self) -> List[int]:
        """Get layer direction preferences for the Rust router.

        Returns list of preferences: 0=horizontal, 1=vertical, 255=none.
        Pattern alternates H/V starting with horizontal on top layer.
        Returns empty list if direction_preference_cost is 0 (disabled).
        """
        if self.direction_preference_cost == 0:
            return []  # Disabled
        prefs = []
        for i in range(len(self.layers)):
            # Alternate: even layers (0, 2, 4) = horizontal (0), odd layers (1, 3, 5) = vertical (1)
            prefs.append(i % 2)
        return prefs

    def cell_cost(self, cost_mm: float) -> int:
        """Per-cell cost units for costs that accumulate per visited cell
        (proximity penalties, attraction bonuses).

        Cost knobs are calibrated at REFERENCE_GRID_STEP: the per-cell value
        scales with grid_step so the accumulated cost per mm of path is
        independent of --grid-step (a finer grid visits proportionally more
        cells). At 0.1mm this reproduces the historical values exactly.
        """
        # soft-knobs B5: per-cell units are CONSTANT per cell (no grid_step
        # factor). The old extra (grid_step/REFERENCE) factor made every
        # per-cell knob relatively 2x/4x weaker at fine grids (0.05/0.025 --
        # exactly the fine-pitch ladder and net_rescue grids) and 2x stronger
        # at 0.2, because the base move cost per mm is 1000/grid_step, not
        # constant. Identical to the old value at the 0.1 reference grid.
        return int(cost_mm * 1000 / REFERENCE_GRID_STEP)

    def scaled_cell_units(self, units: float) -> int:
        """Per-cell cost knobs in raw units calibrated at REFERENCE_GRID_STEP
        (e.g. bus_attraction_bonus). soft-knobs B5: constant per cell -- the
        old (grid_step/REFERENCE) factor broke relative strength vs the move
        cost at non-reference grids. Identical at 0.1."""
        return int(units)

    def via_cost_units(self) -> int:
        """Per-via penalty in cost units.

        The via_cost knob is in grid steps at REFERENCE_GRID_STEP (default 50
        = 5mm of path); the value scales with 1/grid_step so a via costs the
        same mm-equivalent detour at any --grid-step.
        """
        return int(self.via_cost * 1000 * (REFERENCE_GRID_STEP / self.grid_step))

    def via_proximity_cost_int(self) -> int:
        """Rust-facing integer via-proximity multiplier.

        0 stays 0 (its special meaning: BLOCK vias near obstacles instead of
        penalizing); any positive fraction rounds to at least 1 (soft-knobs
        review B3: a GUI value of 0.5 passed through bare int() became 0 --
        neither blocked nor penalized, weaker than both settings around it).
        """
        c = self.via_proximity_cost
        return 0 if c == 0 else max(1, int(round(c)))

    def get_proximity_heuristic_cost(self) -> int:
        """Get the maximum proximity heuristic cost for the Rust router.

        Auto-computes expected proximity cost per grid step based on stub/track/BGA
        proximity settings. This tightens the A* heuristic for boards with high
        proximity costs, dramatically reducing search space (up to 6x speedup).

        The formula weights each proximity cost by its radius (larger radius = more
        of the path affected), sums them, and applies a coverage factor.

        Returns cost scaled for grid units (cost per grid step).
        """
        # Weight each proximity cost by its radius (larger radius = more path affected)
        stub_weight = self.stub_proximity_cost * self.stub_proximity_radius
        track_weight = self.track_proximity_cost * self.track_proximity_distance
        bga_weight = self.bga_proximity_cost * self.bga_proximity_radius
        total_weight = stub_weight + track_weight + bga_weight

        if total_weight > 0:
            # Simple formula: total_weight * factor
            # Default factor 0.02 is conservative to avoid overestimating for paths
            # that don't go through proximity zones. Tuned for ~5mm typical radius.
            estimated_cost = total_weight * self.proximity_heuristic_factor
            return self.cell_cost(estimated_cost)
        return 0

    def get_proximity_heuristic_for_zones(self, src_in_stub: bool, src_in_bga: bool,
                                          tgt_in_stub: bool, tgt_in_bga: bool) -> int:
        """Get proximity heuristic cost based on which zones the endpoints are in.

        More precise than get_proximity_heuristic_cost() - only adds costs for
        zones that the source or target is actually inside.

        Args:
            src_in_stub: True if source is in a stub proximity zone
            src_in_bga: True if source is in a BGA proximity zone
            tgt_in_stub: True if target is in a stub proximity zone
            tgt_in_bga: True if target is in a BGA proximity zone

        Returns cost scaled for grid units (cost per grid step).
        """
        total_weight = 0.0

        # Source endpoint zones
        if src_in_stub:
            total_weight += self.stub_proximity_cost * self.stub_proximity_radius
        if src_in_bga:
            total_weight += self.bga_proximity_cost * self.bga_proximity_radius

        # Target endpoint zones
        if tgt_in_stub:
            total_weight += self.stub_proximity_cost * self.stub_proximity_radius
        if tgt_in_bga:
            total_weight += self.bga_proximity_cost * self.bga_proximity_radius

        if total_weight > 0:
            estimated_cost = total_weight * self.proximity_heuristic_factor
            return self.cell_cost(estimated_cost)
        return 0


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

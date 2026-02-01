"""
Routing state management for PCB routing.

This module provides the RoutingState class that encapsulates all shared state
used during the routing process, enabling cleaner extraction of routing loops
into separate functions.
"""

from typing import List, Dict, Set, Tuple, Optional, Any
from dataclasses import dataclass, field

from kicad_parser import PCBData
from routing_config import GridRouteConfig
from routing_utils import build_layer_map


@dataclass
class RoutingState:
    """
    Encapsulates all shared state used during routing.

    This class holds the mutable state that gets updated as nets are routed,
    ripped up, and re-routed. Using a single state object makes it easier to
    pass context between routing functions.
    """

    # Core routing state
    pcb_data: PCBData
    config: GridRouteConfig

    # Lists and dicts tracking routed nets
    routed_net_ids: List[int] = field(default_factory=list)
    routed_net_paths: Dict[int, List[Tuple[int, int, int]]] = field(default_factory=dict)
    routed_results: Dict[int, Dict] = field(default_factory=dict)
    diff_pair_by_net_id: Dict[int, Tuple[str, Any]] = field(default_factory=dict)

    # Remaining nets to route
    remaining_net_ids: List[int] = field(default_factory=list)

    # Caches
    track_proximity_cache: Dict[int, Dict] = field(default_factory=dict)
    layer_map: Dict[str, int] = field(default_factory=dict)
    net_obstacles_cache: Dict[int, Any] = field(default_factory=dict)  # Pre-computed net obstacles

    # Ripped route avoidance costs - layer-specific (for segments)
    ripped_route_layer_costs: Dict[int, Any] = field(default_factory=dict)  # net_id -> numpy array [layer, gx, gy, cost]
    # Ripped route via positions (grid coords) - all-layer costs computed at merge time
    ripped_route_via_positions: Dict[int, List[Tuple[int, int]]] = field(default_factory=dict)

    # Reroute queue for ripped-up nets
    reroute_queue: List[Tuple] = field(default_factory=list)
    # Track which nets are already queued (to prevent duplicate entries)
    queued_net_ids: Set[int] = field(default_factory=set)

    # Tracking sets
    polarity_swapped_pairs: Set[str] = field(default_factory=set)
    rip_and_retry_history: Set[Tuple] = field(default_factory=set)
    ripup_success_pairs: Set[str] = field(default_factory=set)
    rerouted_pairs: Set[str] = field(default_factory=set)

    # Environment
    all_unrouted_net_ids: Set[int] = field(default_factory=set)
    gnd_net_id: Optional[int] = None

    # Counters
    successful: int = 0
    failed: int = 0
    total_time: float = 0.0
    total_iterations: int = 0
    route_index: int = 0

    # Results collection
    results: List[Dict] = field(default_factory=list)

    # Multi-point routing: track nets needing Phase 3 completion
    # Maps net_id -> main_result dict with 'multipoint_pad_info' and 'routed_pad_indices'
    pending_multipoint_nets: Dict[int, Dict] = field(default_factory=dict)

    # Layer swap tracking
    all_segment_modifications: List = field(default_factory=list)
    all_swap_vias: List = field(default_factory=list)
    pad_swaps: List[Dict] = field(default_factory=list)

    # Obstacle maps (set by caller)
    base_obstacles: Any = None
    diff_pair_base_obstacles: Any = None
    diff_pair_extra_clearance: float = 0.0
    working_obstacles: Any = None  # Incremental working map with all net obstacles

    # Configuration flags
    enable_layer_switch: bool = False
    debug_lines: bool = False

    # Cancellation callback
    cancel_check: Any = None  # Optional callable returning True if routing should be cancelled

    # Progress callback
    progress_callback: Any = None  # Optional callable(current, total, net_name) for progress updates

    # Target swap info (for output writing)
    target_swaps: Dict[str, str] = field(default_factory=dict)
    target_swap_info: List[Dict] = field(default_factory=list)
    single_ended_target_swaps: Dict[str, str] = field(default_factory=dict)
    single_ended_target_swap_info: List[Dict] = field(default_factory=list)

    # Total counts
    total_routes: int = 0
    total_layer_swaps: int = 0

    # Net history tracking for debugging failed routes
    # Maps net_id -> list of event dicts with keys: event, details, sequence
    net_history: Dict[int, List[Dict]] = field(default_factory=dict)

    def __post_init__(self):
        """Initialize layer_map if not provided."""
        if not self.layer_map and self.config:
            self.layer_map = build_layer_map(self.config.layers)


def create_routing_state(
    pcb_data: PCBData,
    config: GridRouteConfig,
    all_net_ids_to_route: List[int],
    base_obstacles,
    diff_pair_base_obstacles,
    diff_pair_extra_clearance: float,
    gnd_net_id: Optional[int],
    all_unrouted_net_ids: Set[int],
    total_routes: int,
    enable_layer_switch: bool = False,
    debug_lines: bool = False,
    target_swaps: Optional[Dict[str, str]] = None,
    target_swap_info: Optional[List[Dict]] = None,
    single_ended_target_swaps: Optional[Dict[str, str]] = None,
    single_ended_target_swap_info: Optional[List[Dict]] = None,
    all_segment_modifications: Optional[List] = None,
    all_swap_vias: Optional[List] = None,
    total_layer_swaps: int = 0,
    net_obstacles_cache: Optional[Dict] = None,
    working_obstacles: Any = None,
    cancel_check: Any = None,
    progress_callback: Any = None,
) -> RoutingState:
    """
    Create and initialize a RoutingState object.

    This is a convenience factory function that sets up the state with
    all the necessary initial values.
    """
    return RoutingState(
        pcb_data=pcb_data,
        config=config,
        remaining_net_ids=list(all_net_ids_to_route),
        base_obstacles=base_obstacles,
        diff_pair_base_obstacles=diff_pair_base_obstacles,
        diff_pair_extra_clearance=diff_pair_extra_clearance,
        gnd_net_id=gnd_net_id,
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps=target_swaps or {},
        target_swap_info=target_swap_info or [],
        single_ended_target_swaps=single_ended_target_swaps or {},
        single_ended_target_swap_info=single_ended_target_swap_info or [],
        all_segment_modifications=all_segment_modifications or [],
        all_swap_vias=all_swap_vias or [],
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=net_obstacles_cache or {},
        working_obstacles=working_obstacles,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )


def record_net_event(state: RoutingState, net_id: int, event: str, details: Dict = None):
    """
    Record an event in a net's history for debugging.

    Events:
      - "initial_route": Net was first routed successfully
      - "ripped_by": Net was ripped due to another net's routing
      - "reroute_attempt": Re-route was attempted
      - "reroute_failed": Re-route failed (details has blocking info)
      - "reroute_succeeded": Re-route succeeded
    """
    if net_id not in state.net_history:
        state.net_history[net_id] = []

    state.net_history[net_id].append({
        "event": event,
        "sequence": state.route_index,
        "details": details or {}
    })


def get_net_history_summary(state: RoutingState, net_id: int, pcb_data: 'PCBData') -> str:
    """
    Get a human-readable summary of a net's routing history.

    Returns a string describing what happened to the net.
    """
    if net_id not in state.net_history:
        return "No history recorded"

    history = state.net_history[net_id]
    net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"

    lines = []
    for entry in history:
        event = entry["event"]
        details = entry.get("details", {})
        seq = entry.get("sequence", "?")

        if event == "initial_route":
            route_type = details.get("type", "single-ended")
            lines.append(f"[{seq}] Initially routed ({route_type})")

        elif event == "ripped_by":
            ripper = details.get("ripping_net_name", "unknown")
            reason = details.get("reason", "rip-up retry")
            lines.append(f"[{seq}] Ripped by {ripper} ({reason})")

        elif event == "reroute_attempt":
            n_val = details.get("N", "?")
            lines.append(f"[{seq}] Re-route attempt (N={n_val})")

        elif event == "reroute_failed":
            reason = details.get("reason", "no path found")
            blockers = details.get("top_blockers", [])
            blocker_str = ", ".join(blockers[:3]) if blockers else "unknown"
            lines.append(f"[{seq}] Re-route FAILED: {reason}")
            if blockers:
                lines.append(f"       Blocked by: {blocker_str}")

        elif event == "reroute_succeeded":
            lines.append(f"[{seq}] Re-route succeeded")

        else:
            lines.append(f"[{seq}] {event}")

    return "\n".join(lines) if lines else "No events"


def print_failed_net_histories(state: RoutingState, failed_net_ids: List[int], pcb_data: 'PCBData'):
    """
    Print history summaries for all failed nets.
    """
    if not failed_net_ids:
        return

    print("\n" + "=" * 60)
    print("FAILED NET HISTORIES")
    print("=" * 60)

    for net_id in failed_net_ids:
        net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"net_{net_id}"
        print(f"\n{net_name}:")
        summary = get_net_history_summary(state, net_id, pcb_data)
        # Indent each line
        for line in summary.split("\n"):
            print(f"  {line}")

"""
Routing state management for PCB routing.

This module provides the RoutingState class that encapsulates all shared state
used during the routing process, enabling cleaner extraction of routing loops
into separate functions.
"""
from __future__ import annotations

import os
from typing import List, Dict, Set, Tuple, Optional, Any
from dataclasses import dataclass, field

from kicad_parser import PCBData
from routing_config import GridRouteConfig
from routing_utils import build_layer_map


# Issue #219: the diff-pair extension of the rip-up cycle-guard is OFF by default.
# The single-ended guard (record_rip_ancestry, commit e3e4b57's queue-level sibling)
# stays always-on; this flag gates only the diff-pair rip/reroute paths. A clean
# per-board A/B (committed vs guard, identical input) showed the diff-pair guard is
# a no-op whenever no diff-pair rip war occurs (the common case, incl. the
# motivating cparti board -- whose +1V8<->B_{n}ON war is single-ended) and a
# sign-unstable butterfly when one does: -13% router iterations on one congested
# config, +12% on another, and one config traded a DRC-clean board for a P/N
# self-crossing on a "successfully" rerouted pair. No measured win, so it ships
# disabled. Kept (not deleted) so it can be graded against a real diff-pair rip war
# if the stress corpus ever surfaces one. Enable with KICAD_DIFFPAIR_RIP_GUARD=1.
DIFF_PAIR_RIP_GUARD = os.environ.get(
    "KICAD_DIFFPAIR_RIP_GUARD", "").strip().lower() not in ("", "0", "false", "no")


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
    # Issue #134: nets whose stale copper would have shorted another net on
    # restore, so they were left ripped instead. Given a clean reroute pass
    # after Phase 3 so the collision-safe restore does not cost completion.
    collision_refused_net_ids: Set[int] = field(default_factory=set)
    # Rip-up cycle guard across the reroute QUEUE (extends the phase3 recursion
    # guard, commit e3e4b57, to the single-ended rip/reroute paths). Maps a net to
    # the set of nets that ripped it (transitively) to get here; that net must not
    # rip any of them back, which breaks the A->B->A rip-up oscillation that ends
    # in a restore-crossing (cparti +1V8<->B_{n}ON). No clearing needed: a net only
    # re-routes after being ripped, so its ancestry is always freshly set then.
    rip_ancestry: Dict[int, Any] = field(default_factory=dict)

    # Tracking sets
    polarity_swapped_pairs: Set[str] = field(default_factory=set)
    # Pairs where a polarity swap was WANTED but the per-pair policy forbade
    # it (#279, --polarity-swap-nets) - resolved by flip or failed honestly.
    polarity_swap_denied_pairs: Set[str] = field(default_factory=set)
    rip_and_retry_history: Set[Tuple] = field(default_factory=set)
    ripup_success_pairs: Set[str] = field(default_factory=set)
    rerouted_pairs: Set[str] = field(default_factory=set)
    # Diff-pair nets whose far-apart (uncoupled) terminal pads were peeled off the
    # coupled chain and must be connected single-ended afterward (issue #121).
    # net_id -> net_name.
    diff_pair_single_ended_nets: Dict[int, str] = field(default_factory=dict)

    # Casualty custody (diff_pair_custody.run_casualty_reconcile): every
    # COMMITTED rip (ripper's route landed; the ripped net's copper is gone
    # unless a later reroute succeeds) records rid -> (canonical_net_id,
    # saved_result, ripped_ids, was_in_results) so the end-of-run
    # casualties-only reconcile can restore the exact pre-rip copper if the
    # reroute never lands. Inert unless the engine runs the reconcile pass
    # (batch_route_diff_pairs does; route.py has its own #134 recovery).
    casualty_custody: Dict[int, Tuple] = field(default_factory=dict)
    # Per-pair failure diagnostics for JSON_SUMMARY pair_reports:
    # pair_name -> {'reason', 'stage', 'blocking_nets', 'casualty', ...}.
    pair_diagnostics: Dict[str, Dict] = field(default_factory=dict)

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
        # `x if x is not None else ...`, NEVER `x or ...`: the caller's lists
        # are ALIASES the engine appends into (stub-swap/relocation mods and
        # vias flow to the output writer through them). `or` silently breaks
        # the alias whenever the caller's list is EMPTY at creation time --
        # a run with no upfront layer swaps then drops every engine-side
        # swap from the written file (the phase-1 stub-switch keeps landed
        # in state while the writer saw a different, empty list).
        target_swaps=target_swaps if target_swaps is not None else {},
        target_swap_info=target_swap_info if target_swap_info is not None else [],
        single_ended_target_swaps=(single_ended_target_swaps
                                   if single_ended_target_swaps is not None else {}),
        single_ended_target_swap_info=(single_ended_target_swap_info
                                       if single_ended_target_swap_info is not None else []),
        all_segment_modifications=(all_segment_modifications
                                   if all_segment_modifications is not None else []),
        all_swap_vias=all_swap_vias if all_swap_vias is not None else [],
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=(net_obstacles_cache
                             if net_obstacles_cache is not None else {}),
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


def record_rip_ancestry(state: RoutingState, ripper_net_id: int, ripped_net_id: int):
    """Cycle guard for the reroute queue (extends phase3's recursion guard e3e4b57).

    Record that ``ripper`` ripped ``ripped`` -- so when ``ripped`` is later
    re-routed it will not rip ``ripper`` (or ripper's own ancestors) back. Without
    this, two nets contending for the same corridor rip each other forever and the
    war ends in a restore-crossing the obstacle map never saw. Ancestry is the
    ripper's chain plus the ripper itself; it REPLACES any prior value (a net only
    re-routes after being ripped, so this is always set fresh at use time)."""
    if ripper_net_id == ripped_net_id:
        return
    anc = (frozenset({ripper_net_id})
           | state.rip_ancestry.get(ripper_net_id, frozenset())) - {ripped_net_id}
    state.rip_ancestry[ripped_net_id] = anc


def record_pair_rip_ancestry(state: RoutingState, ripper_p_id: int,
                             ripper_n_id: int, ripped_net_id: int):
    """Diff-pair ripper variant of record_rip_ancestry (issue #219).

    OFF by default (see DIFF_PAIR_RIP_GUARD) -- a no-op unless explicitly enabled.
    When on: the ripper is a differential pair (both halves rip as one unit), so
    the ripped net must not rip back EITHER half (or their ancestors). Recording
    the two halves with two record_rip_ancestry calls would not work -- the second
    REPLACES the first -- so set the ripped net's ancestry to the UNION of both
    halves' exclude sets in one shot. Like record_rip_ancestry it replaces any
    prior value (a net only re-routes after being ripped)."""
    if not DIFF_PAIR_RIP_GUARD:
        return
    anc = (rip_exclude_set(state, ripper_p_id)
           | rip_exclude_set(state, ripper_n_id)) - {ripped_net_id}
    if anc:
        state.rip_ancestry[ripped_net_id] = anc


def diff_pair_rip_exclude(state: RoutingState, p_net_id: int, n_net_id: int) -> Set[int]:
    """Blocker-exclude set for a (re)routing diff pair (issue #219).

    Guard OFF (default): just the pair's own two nets, as before. Guard ON: also
    excludes the pair's rip-ancestry, so the pair won't rip a net that ripped it
    (the A->B->A cycle guard). Mirrors rip_exclude_set for the single-ended path."""
    if DIFF_PAIR_RIP_GUARD:
        return rip_exclude_set(state, p_net_id) | rip_exclude_set(state, n_net_id)
    return {p_net_id, n_net_id}


def rip_exclude_set(state: RoutingState, net_id: int) -> Set[int]:
    """Nets a (re)routing net must NOT rip: itself plus its rip-ancestry."""
    return {net_id} | set(state.rip_ancestry.get(net_id, frozenset()))


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

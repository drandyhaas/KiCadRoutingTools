"""
Batch Differential Pair PCB Router using Rust-accelerated A* - Routes differential pairs.

Usage:
    python py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"

All nets matching the patterns are treated as differential pairs (P/N pairs).
Nets with _P/_N, P/N, or +/- suffixes will be paired and routed together.
Nets only pair within the same suffix convention (e.g. CLK+ pairs with CLK-,
never with an unrelated CLK_N).

Requires the Rust router module. Build it with:
    python3 build_router.py
(never bare `cargo build` -- build_router.py also places the library and
verifies the version; see CLAUDE.md).
"""
from __future__ import annotations

import env_knobs
import sys
import os
from dataclasses import replace

# Run startup checks before other imports
from startup_checks import exit_on_error_if_main
# Stays at module scope, ABOVE the heavy imports, so a missing dep is
# reported before numpy/grid_router blow up with something cryptic. But it
# raises instead of exiting when this module is IMPORTED rather than run,
# so pytest can still collect a suite on a checkout with no built router
# (#457 item 3).
exit_on_error_if_main(__name__)

import time
import fnmatch
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import parse_kicad_pcb, PCBData, Pad
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr, generate_gr_text_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)
from output_writer import write_routed_output
from schematic_updater import apply_swaps_to_schematics

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord, DiffPairNet
import routing_defaults as defaults
from routing_utils import pos_key
from connectivity import (
    get_stub_endpoints, find_stub_free_ends, find_connected_groups,
    is_edge_stub, get_net_endpoints, find_connected_segment_positions
)
from net_queries import (
    find_differential_pairs, get_all_unrouted_net_ids, get_chip_pad_positions,
    compute_mps_net_ordering, find_pad_nearest_to_position,
    expand_net_patterns, matches_diff_pair_patterns,
    resolve_gnd_net_id, gnd_candidate_names, filter_routable_nets
)
from impedance import calculate_layer_widths_for_impedance, print_impedance_routing_plan
from pcb_modification import add_route_to_pcb_data, remove_route_from_pcb_data
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance,
    get_net_bounds,
    add_connector_region_via_blocking, add_diff_pair_own_stubs_as_obstacles,
    draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from obstacle_costs import (
    add_stub_proximity_costs, compute_track_proximity_for_net,
    merge_track_proximity_costs, add_cross_layer_tracks
)
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, update_net_obstacles_after_routing
)
from diff_pair_routing import route_diff_pair_with_obstacles, get_diff_pair_connector_regions
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from polarity_swap import apply_polarity_swap, get_canonical_net_id
from layer_swap_fallback import try_fallback_layer_swap, add_own_stubs_as_obstacles_for_diff_pair
from layer_swap_optimization import apply_diff_pair_layer_swaps
from routing_context import (
    build_diff_pair_obstacles,
    record_diff_pair_success,
    restore_ripped_net
)
from routing_state import RoutingState, create_routing_state
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb, estimate_track_proximity_cache_mb,
    estimate_routed_paths_mb, format_obstacle_map_stats
)
from diff_pair_loop import route_diff_pairs
from reroute_loop import run_reroute_loop
from length_matching import run_intra_pair_matching
from net_ordering import order_nets_mps, order_nets_inside_out, separate_nets_by_type
from routing_common import (
    setup_bga_exclusion_zones, resolve_net_ids, filter_already_routed,
    run_length_matching, sync_pcb_data_segments, get_common_config_kwargs,
    warn_targets_outside_board
)
import re
from terminal_colors import RED, GREEN, YELLOW, RESET

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap, GridRouter


#: Set when the --nets patterns resolved to no differential pair at all.
#: Read by main() to exit non-zero: the refusal used to print `Error:` and
#: return (0, 0, 0.0), which main discarded, so the process exited 0 and a
#: chained caller walked past an unrouted board.
_NO_PAIRS_MATCHED = False


def batch_route_diff_pairs(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                # #530: cap every auto-read net class at this clearance (the
                # explicit --clearance-ceiling). None = honour the classes.
                clearance_ceiling: Optional[float] = None,
                layer_costs: Optional[List[float]] = None,
                # #498: {layer: mm} per-layer clearance. None (both fronts) ->
                # auto-read the sibling .kicad_dru; explicit dict (tests) wins.
                layer_clearances: Optional[Dict[str, float]] = None,
                # {net_id: mm} track-to-track clearance map (#735); None ->
                # auto-read the sibling .kicad_dru track rules.
                track_clearances: Optional[Dict[int, float]] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                ripup_blocker_select: str = defaults.RIPUP_BLOCKER_SELECT,
                disable_bga_zones: Optional[List[str]] = None,
                track_width: float = defaults.TRACK_WIDTH,
                impedance: Optional[float] = None,
                coplanar_gap: float = 0.0,
                clearance: float = defaults.CLEARANCE,
                via_size: float = defaults.VIA_SIZE,
                via_drill: float = defaults.VIA_DRILL,
                grid_step: float = defaults.GRID_STEP,
                via_cost: int = defaults.VIA_COST,
                max_iterations: int = defaults.MAX_ITERATIONS,
                max_probe_iterations: int = defaults.MAX_PROBE_ITERATIONS,
                heuristic_weight: float = defaults.HEURISTIC_WEIGHT,
                turn_cost: int = defaults.TURN_COST,
                direction_preference_cost: int = defaults.DIRECTION_PREFERENCE_COST,
                bus_enabled: bool = False,
                bus_detection_radius: float = defaults.BUS_DETECTION_RADIUS,
                bus_attraction_radius: float = defaults.BUS_ATTRACTION_RADIUS,
                bus_attraction_bonus: int = defaults.BUS_ATTRACTION_BONUS,
                bus_min_nets: int = defaults.BUS_MIN_NETS,
                keepout_enabled: bool = False,
                keepout_layer: str = defaults.KEEPOUT_LAYER,
                proximity_heuristic_factor: float = defaults.PROXIMITY_HEURISTIC_FACTOR,
                stub_proximity_radius: float = defaults.STUB_PROXIMITY_RADIUS,
                stub_proximity_cost: float = defaults.STUB_PROXIMITY_COST,
                via_proximity_cost: float = defaults.VIA_PROXIMITY_COST,
                bga_proximity_radius: float = defaults.BGA_PROXIMITY_RADIUS,
                bga_proximity_cost: float = defaults.BGA_PROXIMITY_COST,
                track_proximity_distance: float = defaults.TRACK_PROXIMITY_DISTANCE,
                track_proximity_cost: float = defaults.TRACK_PROXIMITY_COST,
                diff_pair_gap: float = defaults.DIFF_PAIR_GAP,
                diff_pair_width_from_class: bool = False,
                diff_pair_gap_from_class: bool = False,
                diff_pair_centerline_setback: float = None,
                min_turning_radius: float = defaults.DIFF_PAIR_MIN_TURNING_RADIUS,
                debug_lines: bool = False,
                verbose: bool = False,
                polarity_swap_nets: Optional[List[str]] = None,
                max_rip_up_count: int = defaults.MAX_RIPUP,
                max_setback_angle: float = defaults.DIFF_PAIR_MAX_SETBACK_ANGLE,
                enable_layer_switch: bool = True,
                crossing_layer_check: bool = True,
                can_swap_to_top_layer: bool = False,
                swappable_net_patterns: Optional[List[str]] = None,
                crossing_penalty: float = defaults.CROSSING_PENALTY,
                mps_unroll: bool = True,
                skip_routing: bool = False,
                routing_clearance_margin: float = defaults.ROUTING_CLEARANCE_MARGIN,
                hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
                board_edge_clearance: float = defaults.BOARD_EDGE_CLEARANCE,
                # #581: > 0 keeps every placed via off same-net SMD pads at
                # this edge-to-edge clearance; None auto-reads the persisted
                # .kicad_pro record; 0 / -1 explicitly off.
                same_net_pad_clearance: Optional[float] = None,
                max_turn_angle: float = defaults.DIFF_PAIR_MAX_TURN_ANGLE,
                gnd_via_enabled: bool = True,
                vertical_attraction_radius: float = defaults.VERTICAL_ATTRACTION_RADIUS,
                vertical_attraction_cost: float = defaults.VERTICAL_ATTRACTION_COST,
                ripped_route_avoidance_radius: float = defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS,
                ripped_route_avoidance_cost: float = defaults.RIPPED_ROUTE_AVOIDANCE_COST,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = defaults.LENGTH_MATCH_TOLERANCE,
                meander_amplitude: float = defaults.MEANDER_AMPLITUDE,
                meander_spacing: float = defaults.MEANDER_SPACING,
                time_matching: bool = defaults.TIME_MATCHING,
                time_match_tolerance: float = defaults.TIME_MATCH_TOLERANCE,
                diff_chamfer_extra: float = defaults.DIFF_PAIR_CHAMFER_EXTRA,
                diff_pair_intra_match: bool = False,
                ac_couple_match: bool = False,
                debug_memory: bool = False,
                mps_reverse_rounds: bool = False,
                mps_layer_swap: bool = False,
                mps_segment_intersection: bool = False,
                keep_input_copper: bool = False,
                schematic_dir: Optional[str] = None,
                add_teardrops: bool = False,
                return_results: bool = False,
                pcb_data=None,
                net_clearances: dict = None,
                # Net-name globs to PROTECT for this run (reason 'user', #521):
                # rip machinery skips matches, and they persist to the output
                # .kicad_pro so later steps honor them without the flag.
                cancel_check=None,
                progress_callback=None) -> Tuple[int, int, float]:
    """
    Route differential pairs using the Rust router.

    All nets provided are treated as differential pairs. Nets with _P/_N, P/N, or +/-
    suffixes will be paired and routed together maintaining constant spacing.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route (all treated as differential pairs)
        layers: List of copper layers to route on
        diff_pair_gap: Gap between P and N traces of differential pairs in mm (default: 0.101)
        diff_pair_centerline_setback: Distance in front of stubs to start centerline (default: 2x P-N spacing)
        min_turning_radius: Minimum turning radius for pose-based routing in mm (default: 0.2)
        polarity_swap_nets: Glob patterns naming the pairs ALLOWED to resolve a
            P/N mismatch by swapping pad net assignments (#279). None/empty =
            no swaps ever (deny by default - a swap is only harmless when an
            endpoint can compensate); ['*'] = allow all pairs.
        gnd_via_enabled: Add GND vias near diff pair signal vias (default: True)
        diff_chamfer_extra: Chamfer multiplier for diff pair meanders (default: 1.5)
        diff_pair_intra_match: Enable intra-pair P/N length matching (default: False)
        ac_couple_match: End-to-end length-match AC-coupled diff pairs split by series
            DC-blocking caps, matching the concatenated P vs N path (#196) (default: False)
        return_results: If True, return results data instead of writing to file
        pcb_data: Optional pre-parsed PCBData (if None, loads from input_file)

    Returns:
        If return_results=False: (successful_count, failed_count, total_time)
        If return_results=True: (successful_count, failed_count, total_time, results_data)
    """
    # Diff-pair coupling gap may never sit below `clearance` (#441). KiCad grades a
    # pair's P<->N coupling under the plain copper-clearance rule (P and N are
    # different nets), so a gap tighter than clearance is flagged as a clearance
    # violation on every coupled segment (stm32g474_fc: USB gap 0.1 mm under
    # clearance 0.15 mm -> 9-14 clearance errors). We are NOT allowed to lower the
    # board-wide clearance, so raise the gap to the clearance floor. Done first --
    # before the parity dump, the --impedance width solve, and routing -- so the
    # impedance-derived width is computed at the ACTUAL (floored) gap and stays
    # on target, and every downstream consumer sees a single, consistent gap.
    if diff_pair_gap is not None and clearance and diff_pair_gap < clearance:
        print(f"Diff-pair gap {diff_pair_gap}mm is below clearance {clearance}mm; "
              f"raising gap to {clearance}mm (KiCad grades P<->N coupling as clearance).")
        diff_pair_gap = clearance

    # --- #381 D1: parameter-parity probe for the DIFF path, mirroring
    # batch_route's dump so the GUI/plan diff front can be diffed key-by-key
    # against `route_diff.py` on identical inputs (this is what would have caught
    # D1/D2 automatically -- previously only batch_route was instrumented).
    # Default: overwrite the file and RETURN without routing (single-call A/B).
    # CONTINUE mode (KICAD_DUMP_BATCH_KWARGS_CONTINUE=1): APPEND one JSONL line
    # per call and keep routing, so a whole multi-step GUI plan is captured.
    if env_knobs.DUMP_BATCH_KWARGS:
        import json as _json
        _snap = dict(locals())
        for _k in ('input_file', 'output_file', 'net_names', 'pcb_data',
                   'mem_start'):
            _snap.pop(_k, None)
        _dump = {}
        for _k, _v in sorted(_snap.items()):
            if callable(_v) or _k in ('cancel_check',
                                      'progress_callback'):
                continue
            try:
                _json.dumps(_v)
                _dump[_k] = _v
            except (TypeError, ValueError):
                _dump[_k] = repr(_v)
        _dump['net_names'] = net_names
        _dump['_engine'] = 'batch_route_diff_pairs'
        if env_knobs.DUMP_BATCH_KWARGS_CONTINUE:
            with open(env_knobs.DUMP_BATCH_KWARGS, 'a') as _f:
                _f.write(_json.dumps(_dump, sort_keys=True) + '\n')
            # fall through -- route normally
        else:
            with open(env_knobs.DUMP_BATCH_KWARGS, 'w') as _f:
                _json.dump(_dump, _f, indent=1, sort_keys=True)
            if return_results:
                return 0, 0, 0.0, {'results': [], 'segments_to_remove': [],
                                   'vias_to_remove': []}
            return 0, 0, 0.0

    # Board-setup copper-to-edge rule (#338): route to at least the sibling
    # .kicad_pro's min_copper_edge_clearance (engine-side, so the GUI and plan
    # replays inherit it; see batch_route).
    if input_file:
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            _eff_edge = effective_board_edge_clearance(input_file, board_edge_clearance)
            if _eff_edge > (board_edge_clearance or 0.0):
                print(f"Board edge clearance {_eff_edge}mm "
                      f"(project min_copper_edge_clearance)")
                board_edge_clearance = _eff_edge
        except Exception:
            pass

    # Track memory if debug_memory enabled
    mem_start = get_process_memory_mb() if debug_memory else 0.0
    if debug_memory:
        print(format_memory_stats("Initial memory", mem_start))

    # Load or use provided pcb_data
    if pcb_data is None:
        print(f"Loading {input_file}...")
        pcb_data = parse_kicad_pcb(input_file, keepout_layer=keepout_layer)
    else:
        print("Using provided PCB data...")

    # Route trace (#482, KICAD_ROUTE_TRACE=1): record diff-pair copper as it is
    # committed/ripped/restored for animating the run. Default-off; gated on a
    # real output path so an internal/dry-run call records nothing.
    if output_file:
        from route_trace import attach_trace as _attach_route_trace
        _attach_route_trace(pcb_data)

    # Cross-class clearance: auto-read non-Default netclasses from the sibling
    # .kicad_pro when no map was passed (KiCad pairwise max(classA, classB)). A
    # caller that resolved the map (route_diff main, the GUI) passes a dict so this
    # does not re-read. All-Default boards -> empty map -> inert.
    if net_clearances is None and input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_clearance_map_by_id
            net_clearances = net_clearance_map_by_id(
                input_file, {nid: n.name for nid, n in pcb_data.nets.items()})
            if net_clearances:
                # #439: cap each class at the routing clearance (stock classes are
                # aspirational). A caller that wants the full classes (routed without a
                # --clearance ceiling) passes an explicit uncapped map, so this internal
                # fallback always caps.
                if clearance_ceiling is not None:
                    net_clearances = {nid: min(clr, clearance_ceiling)
                                      for nid, clr in net_clearances.items()}
                    print(f"Auto-read netclass clearances for {len(net_clearances)} net(s), "
                          f"capped at the ceiling {clearance_ceiling}mm (#439; cross-class "
                          f"max(A,B) respected).")
                else:
                    print(f"Auto-read netclass clearances for {len(net_clearances)} net(s), "
                          f"honoured as declared (KiCad pairwise max).")
        except Exception as _e:
            print(f"Warning: could not auto-read netclass clearances ({_e}).")
            net_clearances = None

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']  # Default 4-layer signal stack
    print(f"Using {len(layers)} routing layers: {layers}")

    # Per-layer cost multipliers bias which layer(s) coupled diff pairs prefer
    # (issue #193). Default is 1.0x on every layer so behavior is unchanged unless
    # --layer-costs is given; unlike route.py there is no 2-layer F.Cu/B.Cu default.
    if not layer_costs:
        layer_costs = [1.0] * len(layers)
    # Full-stack normalization (mirrors batch_route): append board copper
    # layers the caller did not request as FORBIDDEN (-1) so vias always
    # respect copper on every layer (the butterstick DQ11 class -- a via
    # spans the whole stack whatever subset the run routes on).
    _board_cu = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    _missing_cu = [l for l in _board_cu if l not in layers]
    if _missing_cu:
        from routing_constants import FORBIDDEN_LAYER_COST
        layers = list(layers) + _missing_cu
        layer_costs = list(layer_costs) + [FORBIDDEN_LAYER_COST] * len(_missing_cu)
        print(f"  Full-stack: appended {len(_missing_cu)} unrequested copper layer(s) "
              f"as FORBIDDEN obstacles: {', '.join(_missing_cu)}")
    for i, cost in enumerate(layer_costs):
        if cost >= 0 and (cost < 1.0 or cost > 1000):
            from routing_exceptions import ConfigurationError
            layer_name = layers[i] if i < len(layers) else f"layer {i}"
            raise ConfigurationError(f"Layer cost for {layer_name} must be negative (forbidden) or "
                                     f"between 1.0 and 1000, got {cost}")
    if any(c != 1.0 for c in layer_costs):
        costs_str = ', '.join(f"{layers[i]}={layer_costs[i]}x" for i in range(min(len(layers), len(layer_costs))))
        print(f"  Layer costs: {costs_str}")

    # Calculate layer-specific widths for impedance-controlled routing
    # For diff pairs, we use the diff_pair_gap as the fixed spacing and calculate width.
    # #610: with --track-width OMITTED the impedance request sets the width floor
    # it implies (bounded below by the fab tier); an explicit --track-width stays
    # a verbatim floor. Clamps land in impedance_width_clamped -> JSON_SUMMARY.
    from impedance import impedance_width_floor
    imp_width_floor, imp_floor_desc = impedance_width_floor(
        track_width, diff_pair_width_from_class,
        len(getattr(pcb_data.board_info, 'copper_layers', None) or []))
    impedance_width_clamped = {}
    layer_widths = {}
    if impedance is None and coplanar_gap:
        # See route.py: the gap only selects the impedance model (#486).
        print("WARNING: --coplanar-gap given without --impedance; it only "
              "selects the impedance model, so it has NO effect here. Add "
              "--impedance <ohms> to route as a coplanar waveguide.")
    if impedance is not None:
        if not pcb_data.board_info.stackup:
            print("WARNING: No stackup found in PCB file. Using fixed track width.")
        else:
            print(f"\nCalculating trace widths for {impedance}Ω differential impedance...")
            print(f"Using diff pair spacing: {diff_pair_gap}mm ({diff_pair_gap * 39.3701:.2f} mil)")
            if diff_pair_width_from_class:
                print(f"  --track-width not given: solved widths floor at the "
                      f"fab-tier track minimum {imp_width_floor}mm, not the "
                      f"default pair width (#610)")
            layer_widths = calculate_layer_widths_for_impedance(
                pcb_data, layers, impedance,
                spacing=diff_pair_gap, is_differential=True,
                fallback_width=track_width,
                min_width=imp_width_floor,
                coplanar_gap=coplanar_gap,
                floor_desc=imp_floor_desc,
                clamp_report=impedance_width_clamped
            )
            print_impedance_routing_plan(pcb_data, layers, impedance,
                                        spacing=diff_pair_gap, is_differential=True,
                                        min_width=imp_width_floor,
                                        coplanar_gap=coplanar_gap)

    # #521: impedance declarations persist per net (see route.py's twin).
    # WITH --impedance: record the differential spec for every targeted member.
    # WITHOUT: if every targeted net shares one stored differential spec,
    # auto-reapply it CALL-level -- the diff engine bakes widths into its
    # obstacle stamps globally (#156 reserve_layer_widths), so per-net widths
    # are not expressible here; mixed specs warn and skip.
    _targets = resolve_net_ids(pcb_data, net_names) if net_names else []
    if impedance is not None:
        from protected_nets import note_impedance_specs
        note_impedance_specs({
            _nm: {'ohms': impedance, 'differential': True,
                  'pair_gap': diff_pair_gap,
                  'coplanar_gap': coplanar_gap or 0.0}
            for _nm, _nid in _targets})
    elif pcb_data.board_info.stackup and _targets:
        from protected_nets import read_impedance_for_pcb_data
        _stored = read_impedance_for_pcb_data(pcb_data, input_file)
        _specs = {(float(sp.get('ohms', 0) or 0), float(sp.get('pair_gap', 0) or 0),
                   float(sp.get('coplanar_gap', 0) or 0))
                  for nm, _nid in _targets
                  for sp in [_stored.get(nm)] if sp and sp.get('differential')}
        _uncovered = [nm for nm, _nid in _targets
                      if not (_stored.get(nm) or {}).get('differential')]
        if len(_specs) == 1 and not _uncovered:
            _ohms, _pgap, _cgap = next(iter(_specs))
            if _ohms:
                print(f"  Reapplying stored {_ohms:g} ohm differential impedance "
                      f"(gap {_pgap}mm{f', coplanar {_cgap}mm' if _cgap else ''}) "
                      f"from .kicad_pro (recorded by an earlier --impedance step)")
                # #610: same floor rule as a live --impedance solve, so the
                # reapplied widths really ARE the same widths.
                layer_widths = calculate_layer_widths_for_impedance(
                    pcb_data, layers, _ohms,
                    spacing=_pgap or diff_pair_gap, is_differential=True,
                    fallback_width=track_width, min_width=imp_width_floor,
                    coplanar_gap=_cgap,
                    floor_desc=imp_floor_desc,
                    clamp_report=impedance_width_clamped)
        elif _specs:
            print(f"  WARNING: targeted nets carry {len(_specs)} different stored "
                  f"impedance spec(s){' (plus unspecified nets)' if _uncovered else ''} "
                  f"-- cannot reapply call-level; pass --impedance explicitly")

    # Auto-detect BGA exclusion zones if not specified
    _sel_ids = [nid for _nm, nid in _targets]
    bga_exclusion_zones = setup_bga_exclusion_zones(
        pcb_data, disable_bga_zones, bga_exclusion_zones,
        selected_net_ids=_sel_ids)

    # Build config kwargs from common parameters plus diff-pair specific options
    config_kwargs = get_common_config_kwargs(
        track_width=track_width, clearance=clearance, via_size=via_size,
        via_drill=via_drill, grid_step=grid_step, via_cost=via_cost,
        layers=layers, max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations, heuristic_weight=heuristic_weight,
        turn_cost=turn_cost, direction_preference_cost=direction_preference_cost,
        bus_enabled=bus_enabled, bus_detection_radius=bus_detection_radius,
        bus_attraction_radius=bus_attraction_radius, bus_attraction_bonus=bus_attraction_bonus,
        bus_min_nets=bus_min_nets, proximity_heuristic_factor=proximity_heuristic_factor,
        keepout_enabled=keepout_enabled, keepout_layer=keepout_layer,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius, stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost, bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost, track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost, debug_lines=debug_lines, verbose=verbose,
        max_rip_up_count=max_rip_up_count, crossing_penalty=crossing_penalty,
        ripup_blocker_select=ripup_blocker_select,
        crossing_layer_check=crossing_layer_check, routing_clearance_margin=routing_clearance_margin,
        hole_to_hole_clearance=hole_to_hole_clearance, board_edge_clearance=board_edge_clearance,
        vertical_attraction_radius=vertical_attraction_radius,
        vertical_attraction_cost=vertical_attraction_cost,
        ripped_route_avoidance_radius=ripped_route_avoidance_radius,
        ripped_route_avoidance_cost=ripped_route_avoidance_cost,
        length_match_groups=length_match_groups,
        length_match_tolerance=length_match_tolerance, meander_amplitude=meander_amplitude,
        meander_spacing=meander_spacing,
        time_matching=time_matching, time_match_tolerance=time_match_tolerance,
        debug_memory=debug_memory
    )
    # Add diff-pair specific config options
    config_kwargs.update(
        diff_pair_gap=diff_pair_gap,
        diff_pair_centerline_setback=diff_pair_centerline_setback,
        min_turning_radius=min_turning_radius,
        max_setback_angle=max_setback_angle,
        max_turn_angle=max_turn_angle,
        gnd_via_enabled=gnd_via_enabled,
        diff_chamfer_extra=diff_chamfer_extra,
        diff_pair_intra_match=diff_pair_intra_match,
        ac_couple_match=ac_couple_match,
    )
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    if layer_widths:
        config_kwargs['layer_widths'] = layer_widths
        config_kwargs['impedance_target'] = impedance
    config_kwargs['layer_costs'] = layer_costs  # per-layer bias for coupled diff routing (#193)
    # #156: the diff engine keeps the mm-exact obstacle maps (per-layer
    # impedance width baked into every stamp) -- the pose router has no
    # track_margin channel to ride instead. Margin helpers computed against
    # this reserve are then 0 for impedance-width legs, i.e. today's behaviour.
    config_kwargs['reserve_layer_widths'] = True
    # #581: an active (> 0) same-net pad via clearance keeps every placed via
    # off same-net SMD pads. Explicit --same-net-pad-clearance wins (> 0
    # activates; 0 / -1 explicitly off); unset auto-reads the record an
    # earlier chain step persisted into the sibling .kicad_pro.
    if same_net_pad_clearance is None:
        from protected_nets import read_snpc_for_pcb_data as _read_snpc581
        _snpc581 = _read_snpc581(pcb_data, input_file)
        _snpc_src = "project record"
    else:
        _snpc581 = same_net_pad_clearance
        _snpc_src = "flag"
    if _snpc581 > 0:
        config_kwargs['same_net_pad_clearance'] = _snpc581
        print(f"Same-net pad via clearance {_snpc581:g}mm (from {_snpc_src}, "
              f"#581): vias stay off same-net pads")
    config = GridRouteConfig(**config_kwargs)

    # Find differential pairs from all provided nets
    diff_pairs: Dict[str, DiffPairNet] = find_differential_pairs(pcb_data, net_names)
    diff_pair_net_ids = set()  # Net IDs that are part of differential pairs

    # Detect AC-coupled XNets (#196): differential pairs split into two base-named
    # pairs by series DC-blocking caps, to be length-matched END-TO-END after
    # routing (see the AC-couple pass below). Pure read of pcb_data -- gathers a
    # side-structure only, never touching diff_pairs / net order / pcb_data, so the
    # routed result is byte-identical when --ac-couple-match is off.
    ac_xnets = []
    if config.ac_couple_match:
        from diff_xnet import find_ac_coupled_xnets
        ac_xnets, ac_warnings = find_ac_coupled_xnets(pcb_data, diff_pairs)
        for _w in ac_warnings:
            print(f"  WARNING: {_w}")
        for _xn in ac_xnets:
            print(f"  AC-coupled XNet: {'+'.join(_xn.base_names)} "
                  f"(coupled via {', '.join(_xn.bridge_refs)})")

    if not diff_pairs:
        # A precise sentinel. Inferring this from a (0, 0) return conflates it
        # with a legitimate run in which every pair DEFERRED to single-ended
        # routing -- also 0 routed, 0 failed, and not an error at all.
        global _NO_PAIRS_MATCHED
        _NO_PAIRS_MATCHED = True
        print(f"Error: No differential pairs found matching the patterns!")
        print("  Differential pairs must have _P/_N, P/N, or +/- suffixes.")
        print(f"  Patterns provided: {net_names}")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # Track which net IDs are part of pairs
    for pair in diff_pairs.values():
        diff_pair_net_ids.add(pair.p_net_id)
        diff_pair_net_ids.add(pair.n_net_id)

    print(f"Found {len(diff_pairs)} differential pair(s)")

    # Per-pair polarity-swap policy (#279): only pairs matching
    # --polarity-swap-nets may have their P/N pad nets swapped. No patterns =
    # no swaps (a swap is only harmless when an endpoint can compensate -
    # FPGA generic I/O, polarity-tolerant protocol - so deny by default);
    # '*' restores swap-everything.
    if polarity_swap_nets:
        for pair in diff_pairs.values():
            pair.polarity_swap_allowed = any(
                matches_diff_pair_patterns(nname, pair.base_name,
                                           polarity_swap_nets)
                for nname in (pair.p_net_name, pair.n_net_name) if nname)
        n_allowed = sum(p.polarity_swap_allowed for p in diff_pairs.values())
        print(f"  Polarity swaps allowed for {n_allowed}/{len(diff_pairs)} "
              f"pair(s) (--polarity-swap-nets)")
    else:
        print("  Polarity swaps disabled (no --polarity-swap-nets; pass '*' "
              "to allow all pairs)")

    # Build the net-id list from the diff pairs we found (both halves), so an
    # explicit base name ('/DVI_CK') or a one-sided glob ('*_P') still routes
    # both halves -- resolve_net_ids only matches exact full net names and would
    # drop the unmatched sibling (or a base name that names no net). (issue #120)
    net_ids = []
    seen_net_ids = set()
    for pair in diff_pairs.values():
        for nid, nname in ((pair.p_net_id, pair.p_net_name),
                           (pair.n_net_id, pair.n_net_name)):
            if nid is not None and nid not in seen_net_ids:
                net_ids.append((nname, nid))
                seen_net_ids.add(nid)
    if not net_ids:
        print("No valid nets to route!")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # Flag target pads at/over the board edge before routing, so an unroutable
    # off-board pad reads as a clear warning rather than a silent search failure
    # (issue #195).
    _edge_clear = board_edge_clearance if board_edge_clearance > 0 else clearance
    warn_targets_outside_board(pcb_data, net_ids,
                               edge_margin=_edge_clear + track_width / 2)

    net_ids, _ = filter_already_routed(pcb_data, net_ids, config)
    if not net_ids:
        print("All nets are already fully connected - nothing to route!")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        # Pass the board through unchanged so a chained pipeline never loses
        # its output file (#86/#90/#167 -- route.py has had this fallback all
        # along; this early-exit lacked it, so a retry step whose pairs turned
        # out already-connected broke the NEXT step with FileNotFoundError).
        if output_file:
            from pcb_io_utils import passthrough_copy
            if passthrough_copy(input_file, output_file):
                print(f"Wrote unchanged copy to {output_file} (nothing to route)")
        return 0, 0, 0.0

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    # Track all vias added during stub layer swapping
    all_swap_vias = []
    # pair_name -> {'vias': [...], 'stubs': [...]} for bare-pad target swaps, so a
    # failed pair can have its swap undone and be re-routed clean (issue #142).
    bare_pad_swaps = {}
    # Track new stub segments synthesized by bare-pad target swaps
    all_swap_segments = []
    # Track total number of layer swaps applied
    total_layer_swaps = 0

    # Identify which diff pairs we'll be routing BEFORE ordering
    # (Layer swaps must happen before MPS ordering since ordering depends on layers)
    diff_pair_ids_to_route_set = []  # (pair_name, pair) tuples - unordered
    processed_pair_net_ids_early = set()
    for net_name, net_id in net_ids:
        if net_id in diff_pair_net_ids and net_id not in processed_pair_net_ids_early:
            for pair_name, pair in diff_pairs.items():
                if pair.p_net_id == net_id or pair.n_net_id == net_id:
                    diff_pair_ids_to_route_set.append((pair_name, pair))
                    processed_pair_net_ids_early.add(pair.p_net_id)
                    processed_pair_net_ids_early.add(pair.n_net_id)
                    break

    if not diff_pair_ids_to_route_set:
        print("Error: No differential pairs to route!")
        print("  Ensure your net patterns match nets with _P/_N, P/N, or +/- suffixes.")
        if return_results:
            return 0, 0, 0.0, {'results': [], 'all_swap_vias': [], 'exclusion_zone_lines': [], 'boundary_debug_labels': []}
        return 0, 0, 0.0

    # ----- Fanout DRC pre-check (#242) ------------------------------------
    # A pair whose fanout escape stubs ALREADY pinch their own P/N below clearance
    # (a bga_fanout escape-fan defect) cannot be coupled-routed cleanly: routing it
    # only spreads or relocates the bad copper (a solo source switch silently moves
    # the overlap to another layer). Detect it up front from the existing stub
    # copper, skip the pair entirely (not routed here, and -- by dropping it from
    # the routed-net set -- left as an obstacle and not handed to the single-ended
    # follow-up), and report it. The fanout must be fixed, not routed over.
    from diff_pair_routing import _seg_to_seglist_min_edge as _fanout_edge

    def _fanout_self_overlaps(p_net_id, n_net_id):
        # Match check_drc's seg-seg verdict: it ignores an overlap <= clearance*5%
        # (a coupled escape laid right at the gap dips a hair below clearance at
        # bends but is DRC-clean). Only a REAL violation (e.g. a touching escape
        # fan, edge ~0) should skip the pair.
        thr = config.clearance * 0.95
        p_segs = [s for s in pcb_data.segments if s.net_id == p_net_id]
        n_segs = [s for s in pcb_data.segments if s.net_id == n_net_id]
        if not p_segs or not n_segs:
            return False
        return any(_fanout_edge(s.start_x, s.start_y, s.end_x, s.end_y, s.width, s.layer, n_segs)
                   < thr for s in p_segs)

    skipped_bad_fanout = [name for name, pair in diff_pair_ids_to_route_set
                          if _fanout_self_overlaps(pair.p_net_id, pair.n_net_id)]
    # (name, p_net, n_net) for the per-pair JSON reports - captured before the
    # skipped pairs are dropped from the route set.
    skipped_fanout_info = [(name, pair.p_net_name, pair.n_net_name)
                           for name, pair in diff_pair_ids_to_route_set
                           if name in set(skipped_bad_fanout)]
    if skipped_bad_fanout:
        skip_set = set(skipped_bad_fanout)
        skip_net_ids = {nid for name, pair in diff_pair_ids_to_route_set if name in skip_set
                        for nid in (pair.p_net_id, pair.n_net_id)}
        diff_pair_ids_to_route_set = [(n, p) for n, p in diff_pair_ids_to_route_set
                                      if n not in skip_set]
        net_ids = [(nm, nid) for nm, nid in net_ids if nid not in skip_net_ids]
        diff_pair_net_ids = {nid for nid in diff_pair_net_ids if nid not in skip_net_ids}
        print("\n" + "=" * 60)
        print(f"Skipping {len(skipped_bad_fanout)} pair(s): fanout escape stubs already pinch "
              f"their own P/N below clearance ({config.clearance}mm) -- fix the fanout (#242):")
        for n in skipped_bad_fanout:
            print(f"  {n}")
        print("=" * 60)

    # Apply target swaps for swappable-nets feature
    # This swaps net IDs at targets BEFORE routing, so routing sees the swapped configuration
    target_swaps: Dict[str, str] = {}  # pair_name -> swapped_target_pair_name
    target_swap_info: List[Dict] = []  # Info needed to apply swaps to output file
    boundary_debug_labels: List[Dict] = []  # Debug labels for boundary positions
    if swappable_net_patterns and diff_pair_ids_to_route_set:
        from target_swap import apply_target_swaps, generate_debug_boundary_labels, summarize_target_swaps
        from diff_pair_routing import get_diff_pair_endpoints

        swappable_diff_pairs = find_differential_pairs(pcb_data, swappable_net_patterns)
        # Only include pairs that are also being routed (not just in diff_pairs, but in the route set)
        pairs_being_routed = {name for name, pair in diff_pair_ids_to_route_set}
        swappable_pairs = [(name, pair) for name, pair in swappable_diff_pairs.items()
                          if name in pairs_being_routed]

        if len(swappable_pairs) >= 2:
            # Generate debug labels if requested (before swaps, so we see original positions)
            if debug_lines and mps_unroll:
                boundary_debug_labels = generate_debug_boundary_labels(
                    pcb_data, swappable_pairs,
                    lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
                )

            target_swaps, target_swap_info = apply_target_swaps(
                pcb_data, swappable_pairs, config,
                lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config),
                use_boundary_ordering=mps_unroll
            )

    # Generate boundary debug labels for all diff pairs if not already generated from swappable pairs
    if debug_lines and mps_unroll and not boundary_debug_labels and diff_pair_ids_to_route_set:
        from target_swap import generate_debug_boundary_labels
        from diff_pair_routing import get_diff_pair_endpoints
        boundary_debug_labels = generate_debug_boundary_labels(
            pcb_data, list(diff_pair_ids_to_route_set),
            lambda pair: get_diff_pair_endpoints(pcb_data, pair.p_net_id, pair.n_net_id, config)
        )

    # Cross-class clearance (PR392): install the per-net class map + routing-side
    # floor on config so BOTH the base obstacle maps and every incremental in-run
    # stamper (partner-leg segs/vias, phase-3 taps) price foreign copper at KiCad's
    # pairwise max(classA, classB). Floor is over the ROUTED nets. Inert when empty.
    config.set_net_clearances(net_clearances, [nid for _, nid in net_ids])
    # #498: per-layer .kicad_dru clearance rules, installed engine-side so the
    # GUI inherits them with no wiring (see kicad_dru.install_layer_clearances).
    from kicad_dru import install_layer_clearances, install_track_clearances
    install_layer_clearances(config, layer_clearances, input_file, pcb_data)
    # Track-scoped .kicad_dru rules (#735; raise-only on seg-vs-seg stamps).
    install_track_clearances(config, track_clearances, input_file, pcb_data,
                             routed_net_ids=[nid for _, nid in net_ids])

    # Upfront layer swap optimization: analyze all diff pairs and apply beneficial swaps
    # BEFORE MPS ordering, so ordering sees correct segment layers
    all_stubs_by_layer = {}
    stub_endpoints_by_layer = {}
    if enable_layer_switch and diff_pair_ids_to_route_set:
        # Probe obstacle map so a bare-pad target swap can fan onto an OPEN
        # launch layer rather than blindly the source layer (issue #121: lpddr4
        # DQ_S0's target was fanned to B.Cu, which is 100% blocked by a connector
        # pad wall, while the inner layers right there were empty).
        _swap_probe_clearance = (config.track_width + config.diff_pair_gap) / 2
        if progress_callback:
            progress_callback(0, 0, "Building layer-swap probe obstacle map...")
        swap_probe_obstacles = build_base_obstacle_map(
            pcb_data, config, [nid for _, nid in net_ids], _swap_probe_clearance,
            net_clearances=net_clearances, progress_callback=progress_callback)
        total_layer_swaps, all_stubs_by_layer, stub_endpoints_by_layer = apply_diff_pair_layer_swaps(
            pcb_data, config, diff_pair_ids_to_route_set, diff_pairs,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias,
            all_swap_segments=all_swap_segments, probe_obstacles=swap_probe_obstacles,
            bare_pad_swaps=bare_pad_swaps
        )

    # NOTE (#508 finding 10): apply_stub_layer_switch already appends each
    # swap via to pcb_data.vias itself -- the re-append loop route.py removed
    # (see its note at the apply_single_ended_layer_swaps call) survived here,
    # putting every pad swap via on the board TWICE (double obstacle stamp,
    # and the board carried one more via than the written file).

    # Skip (and loudly list) nets with <2 pads -- unroutable; do it before ordering.
    net_ids = filter_routable_nets(pcb_data, net_ids)

    # Apply net ordering strategy
    if ordering_strategy == "mps":
        net_ids, mps_layer_swaps = order_nets_mps(
            pcb_data=pcb_data,
            net_ids=net_ids,
            diff_pairs=diff_pairs,
            mps_unroll=mps_unroll,
            bga_exclusion_zones=bga_exclusion_zones,
            mps_reverse_rounds=mps_reverse_rounds,
            crossing_layer_check=crossing_layer_check,
            mps_segment_intersection=mps_segment_intersection,
            mps_layer_swap=mps_layer_swap,
            enable_layer_switch=enable_layer_switch,
            config=config,
            can_swap_to_top_layer=can_swap_to_top_layer,
            all_segment_modifications=all_segment_modifications,
            all_swap_vias=all_swap_vias,
            all_stubs_by_layer=all_stubs_by_layer,
            stub_endpoints_by_layer=stub_endpoints_by_layer,
            verbose=verbose
        )
        total_layer_swaps += mps_layer_swaps

    elif ordering_strategy == "inside_out" and bga_exclusion_zones:
        net_ids = order_nets_inside_out(pcb_data, net_ids, bga_exclusion_zones)

    elif ordering_strategy == "original":
        print("\nUsing original net order (no sorting)")

    # All nets are diff pairs - separate them from any non-diff-pair nets
    diff_pair_ids_to_route, single_ended_nets = separate_nets_by_type(
        net_ids, diff_pairs, diff_pair_net_ids
    )

    # Warn if any single-ended nets were found (they won't be routed)
    if single_ended_nets:
        print(f"\nWarning: Skipping {len(single_ended_nets)} non-differential-pair net(s):")
        for net_name, _ in single_ended_nets[:5]:
            print(f"  {net_name}")
        if len(single_ended_nets) > 5:
            print(f"  ... and {len(single_ended_nets) - 5} more")
        print("  Use route.py for single-ended routing.")

    total_routes = len(diff_pair_ids_to_route)

    results = []
    pad_swaps = []  # List of (pad1, pad2) tuples for nets that need swapping
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0

    # Skip routing if requested - just write output with swaps and debug info
    if skip_routing:
        print(f"\n--skip-routing: Skipping actual routing of {total_routes} diff pairs")
        print("Writing output file with swaps and debug labels only...")
        diff_pair_ids_to_route = []
    else:
        print(f"\nRouting {total_routes} differential pair(s)...")
        print("=" * 60)

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]
    print("Building base obstacle map...")
    base_start = time.time()
    base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route,
                                             net_clearances=net_clearances,
                                             progress_callback=progress_callback)
    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")
    if debug_memory:
        mem_after_base = get_process_memory_mb()
        print(format_memory_stats("After base obstacle map", mem_after_base, mem_after_base - mem_start))

    # Save original (pre-routing) segment signatures to preserve stubs during sync.
    # Keep the OBJECTS alive too: a bare id() set is only valid while the objects
    # live -- when routing replaces originals and they are garbage-collected,
    # CPython recycles their ids for NEW Segment objects, which sync then wrongly
    # treats as "original" (keeping them AND re-adding them from the results, so
    # the same object sits in pcb_data.segments twice -- the duplicate graph
    # nodes then let the graze prune approve removals that gut the net, #195).
    _original_segments_keepalive = list(pcb_data.segments)
    original_segment_ids = set(id(s) for s in _original_segments_keepalive)

    # Build separate base obstacle map with extra clearance for diff pair centerline routing
    # Extra clearance = spacing from centerline to P/N track center
    diff_pair_extra_clearance = (config.track_width + config.diff_pair_gap) / 2
    print(f"Building diff pair obstacle map (extra clearance: {diff_pair_extra_clearance:.3f}mm)...")
    if progress_callback:
        progress_callback(0, 0, "Building diff-pair obstacle map...")
    dp_base_start = time.time()
    diff_pair_base_obstacles = build_base_obstacle_map(pcb_data, config, all_net_ids_to_route,
                                                       diff_pair_extra_clearance,
                                                       net_clearances=net_clearances,
                                                       progress_callback=progress_callback)
    dp_base_elapsed = time.time() - dp_base_start
    print(f"Diff pair obstacle map built in {dp_base_elapsed:.2f}s")
    if debug_memory:
        mem_after_dp = get_process_memory_mb()
        print(format_memory_stats("After diff pair obstacle map", mem_after_dp, mem_after_dp - mem_start))

    # #435: per-pair impedance geometry. When --track-width / --diff-pair-gap were
    # NOT explicitly given, each pair falls back to its OWN netclass diff_pair_width
    # / diff_pair_gap (not the board Default), so a multi-class board routes every
    # pair to its own controlled-impedance geometry. Build one base obstacle map per
    # DISTINCT (width, gap) so each pair reserves exactly its own coupled channel;
    # block each pair's connector regions on ITS geometry map.
    pair_diff_geom = {}                       # {p_net_id: (width, gap)}; EMPTY when
                                              # geometry is explicit (loop stays inert)
    diff_pair_base_obstacles_by_geom = {}     # {(width, gap): obstacle_map}
    if diff_pair_ids_to_route:
        _global_geom = (round(config.track_width, 4), round(config.diff_pair_gap, 4))
        diff_pair_base_obstacles_by_geom[_global_geom] = diff_pair_base_obstacles
        # Only resolve per-pair geometry when a flag was OMITTED (from_class). When
        # both were explicit, pair_diff_geom stays empty and every pair routes at the
        # global config exactly as before -- no per-pair replace(), byte-identical.
        if diff_pair_width_from_class or diff_pair_gap_from_class:
            try:
                from list_nets import read_design_rules, resolve_net_class, fab_floors
                _rules = read_design_rules(input_file) if input_file else {}
                _classes = (_rules.get('classes') or {}) if _rules else {}
                _fab = fab_floors(len(getattr(pcb_data.board_info, 'copper_layers', None) or []) or 4)
                _wfloor, _gfloor = _fab.get('track_width', 0.0), _fab.get('clearance', 0.0)
            except Exception as _e:
                print(f"  (#435: could not read per-pair netclass diff geometry: {_e})")
                _classes, _rules, _wfloor, _gfloor = {}, {}, 0.0, 0.0
            for _pn, _pair in diff_pair_ids_to_route:
                _c = _classes.get(resolve_net_class(_pair.p_net_name, _rules), {}) if _classes else {}
                # --impedance derives width per layer, so a netclass width must
                # not override it (#610: from_class now purely means "the flag
                # was omitted", the impedance guard lives here).
                _w = (_c.get('diff_pair_width')
                      if diff_pair_width_from_class and impedance is None else None)
                _g = _c.get('diff_pair_gap') if diff_pair_gap_from_class else None
                _ew = max(_w if _w is not None else config.track_width, _wfloor)
                _eg = max(_g if _g is not None else config.diff_pair_gap, _gfloor)
                geom = (round(_ew, 4), round(_eg, 4))
                pair_diff_geom[_pair.p_net_id] = geom
                # Build one base obstacle map per DISTINCT geometry (dedup; ~1-3 classes).
                if geom not in diff_pair_base_obstacles_by_geom:
                    _ec = (geom[0] + geom[1]) / 2
                    print(f"  #435: building diff obstacle map for class geometry "
                          f"width={geom[0]} gap={geom[1]} (extra clearance {_ec:.3f}mm)...")
                    if progress_callback:
                        progress_callback(
                            0, 0, f"Building diff-pair obstacle map "
                                  f"(width {geom[0]} / gap {geom[1]})...")
                    diff_pair_base_obstacles_by_geom[geom] = build_base_obstacle_map(
                        pcb_data, replace(config, track_width=geom[0], diff_pair_gap=geom[1]),
                        all_net_ids_to_route, _ec, net_clearances=net_clearances,
                        progress_callback=progress_callback)
            if len(set(pair_diff_geom.values())) > 1:
                print(f"  #435: {len(diff_pair_base_obstacles_by_geom)} distinct diff-pair "
                      f"geometries across {len(pair_diff_geom)} pair(s).")

        # Block connector regions upfront, each on ITS geometry map with ITS config
        # (global map when geometry is explicit -- pair_diff_geom empty)
        # (vias span all layers, so connector regions must be blocked before routing).
        print(f"Blocking connector regions for {len(diff_pair_ids_to_route)} diff pair(s)...")
        for pair_name, pair in diff_pair_ids_to_route:
            geom = pair_diff_geom.get(pair.p_net_id, _global_geom)
            _dp_map = diff_pair_base_obstacles_by_geom[geom]
            _pcfg = config if geom == _global_geom else replace(
                config, track_width=geom[0], diff_pair_gap=geom[1])
            connector_info = get_diff_pair_connector_regions(pcb_data, pair, _pcfg)
            if connector_info:
                for end in ('src', 'tgt'):
                    dir_x, dir_y = connector_info[f'{end}_dir']
                    # Synthesized (bare-pad) directions may be flipped by the
                    # router, so block the connector region on both sides
                    signs = (1, -1) if connector_info[f'{end}_dir_synthesized'] else (1,)
                    for sign in signs:
                        add_connector_region_via_blocking(
                            _dp_map,
                            connector_info[f'{end}_center'][0], connector_info[f'{end}_center'][1],
                            dir_x * sign, dir_y * sign,
                            connector_info[f'{end}_setback'], connector_info['spacing_mm'], _pcfg
                        )

    # Get ALL unrouted nets in the PCB for stub proximity costs
    all_unrouted_net_ids = sorted(set(get_all_unrouted_net_ids(pcb_data)))
    print(f"Found {len(all_unrouted_net_ids)} unrouted nets in PCB for stub proximity")

    # Get exclusion zone lines for User.5 if debug_lines is enabled
    exclusion_zone_lines = []
    if debug_lines:
        all_unrouted_stubs = get_stub_endpoints(pcb_data, list(all_unrouted_net_ids))
        all_chip_pads = get_chip_pad_positions(pcb_data, list(all_unrouted_net_ids))
        all_proximity_points = all_unrouted_stubs + all_chip_pads
        exclusion_zone_lines = draw_exclusion_zones_debug(config, all_proximity_points)
        print(f"Will draw {len(config.bga_exclusion_zones)} BGA zones and {len(all_proximity_points)} stub/pad proximity circles on User.5")

    # Find GND net ID for GND via obstacle tracking (if GND vias enabled).
    # Match the GND family robustly ('/GND', 'GNDA', 'DGND', ... all count),
    # not a literal 'GND' that silently disabled --gnd-vias on most boards (#379).
    gnd_net_id = None
    if config.gnd_via_enabled:
        gnd_net_id, gnd_net_name = resolve_gnd_net_id(pcb_data)
        if gnd_net_id:
            print(f"GND net: '{gnd_net_name}' (id {gnd_net_id}) "
                  f"(GND vias will be added as obstacles)")
            # Split-ground board: each pair returns to ITS OWN domain, so say so
            # once here rather than per pair (#489 §5).
            from net_queries import describe_ground_domains
            _dom_note = describe_ground_domains(pcb_data)
            if _dom_note:
                print(_dom_note)
        else:
            cands = gnd_candidate_names(pcb_data)
            hint = f" Candidate nets: {', '.join(cands)}." if cands else ""
            print(f"WARNING: --gnd-vias is enabled but no GND net was found; "
                  f"GND return vias will NOT be placed.{hint}")

    # Pre-compute net obstacles for caching (speeds up per-route setup)
    print("Pre-computing net obstacle cache...")
    cache_start = time.time()
    net_obstacles_cache = precompute_all_net_obstacles(
        pcb_data, list(all_unrouted_net_ids), config,
        extra_clearance=0.0, diagonal_margin=defaults.DIAGONAL_MARGIN
    )
    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")
    if debug_memory:
        mem_after_cache = get_process_memory_mb()
        cache_size = estimate_net_obstacles_cache_mb(net_obstacles_cache)
        print(format_memory_stats("After net obstacles cache", mem_after_cache, mem_after_cache - mem_start))
        print(f"[MEMORY]   Cache estimated size: {cache_size:.1f} MB for {len(net_obstacles_cache)} nets")

    # Build working obstacle map (base + all nets) for incremental updates
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)
    working_obstacles.shrink_to_fit()
    if debug_memory:
        mem_after_working = get_process_memory_mb()
        print(format_memory_stats("After working obstacle map", mem_after_working, mem_after_working - mem_start))

    # Create routing state object to hold all shared state
    state = create_routing_state(
        pcb_data=pcb_data,
        config=config,
        all_net_ids_to_route=all_net_ids_to_route,
        base_obstacles=base_obstacles,
        diff_pair_base_obstacles=diff_pair_base_obstacles,
        diff_pair_extra_clearance=diff_pair_extra_clearance,
        gnd_net_id=gnd_net_id,
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps=target_swaps,
        target_swap_info=target_swap_info,
        single_ended_target_swaps={},  # Not used for diff pairs
        single_ended_target_swap_info=[],
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=net_obstacles_cache,
        working_obstacles=working_obstacles,
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )
    # #435: per-pair impedance geometry + per-(width,gap) base obstacle maps, read
    # by route_diff_pairs to route each pair at its own netclass geometry. Attached
    # post-hoc (RoutingState is a plain dataclass) so no signature churn; the loop
    # reads them via getattr and is inert when unset.
    state.pair_diff_geom = pair_diff_geom
    state.diff_pair_base_obstacles_by_geom = diff_pair_base_obstacles_by_geom

    # Create local aliases for frequently-used state fields
    routed_net_ids = state.routed_net_ids
    routed_net_paths = state.routed_net_paths
    routed_results = state.routed_results
    diff_pair_by_net_id = state.diff_pair_by_net_id
    track_proximity_cache = state.track_proximity_cache

    # BGA proximity costs live in the track-proximity cache under a reserved
    # key (soft-knobs review B1): stamped into the base map they were wiped
    # by prepare_obstacles_inplace's clear_stub_proximity before EVERY
    # single-ended net, so the knob silently no-op'd in the most common path.
    # The cache is re-merged on every prepare in every path (single-ended,
    # diff pair, Phase 3 via the working-map clone).
    from obstacle_costs import compute_bga_proximity_cost_cells, BGA_PROXIMITY_CACHE_KEY
    # #585 item 4: proximity fields for EVERY fine-pitch package (BGA/QFN/QFP)
    # with pad-count-scaled radii; the hard zones stay BGA-only.
    from routing_common import package_proximity_zones
    if env_knobs.PACKAGE_PROXIMITY:
        config.package_proximity_zones = package_proximity_zones(
            pcb_data, config.bga_proximity_radius)
    _bga_cells = compute_bga_proximity_cost_cells(config, len(config.layers))
    if len(_bga_cells):
        track_proximity_cache[BGA_PROXIMITY_CACHE_KEY] = _bga_cells

    # Congestion-aware soft costs (#424 Phase D): all-layer copper-density
    # field under a second reserved cache key; env-gated (KICAD_CONGESTION_
    # COST=0 default off). Vias in hot cells pay via_proximity_cost x the
    # cell cost via the Rust via branch.
    from congestion_field import register_congestion_field
    register_congestion_field(pcb_data, config, track_proximity_cache)

    # History congestion (#590): fresh per-cell conflict field for this call.
    from history_congestion import reset_history
    reset_history(config)
    layer_map = state.layer_map
    reroute_queue = state.reroute_queue
    polarity_swapped_pairs = state.polarity_swapped_pairs
    rip_and_retry_history = state.rip_and_retry_history
    ripup_success_pairs = state.ripup_success_pairs
    rerouted_pairs = state.rerouted_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results
    pad_swaps = state.pad_swaps

    route_index = 0

    # Route differential pairs
    dp_successful, dp_failed, dp_time, dp_iterations, route_index = route_diff_pairs(
        state, diff_pair_ids_to_route
    )
    successful += dp_successful
    failed += dp_failed
    total_time += dp_time
    total_iterations += dp_iterations

    # ----- Bare-pad target swap fallback (issue #142) ---------------------
    # A bare-pad target swap fans a surface connector pad onto an inner layer
    # via a synthesized short stub. For some geometries that stub pins the pair
    # into a forced P/N crossing, so the pair FAILS to route -- yet the very
    # same pair routes cleanly straight to the bare pads with a plain via.
    # Detect those failures, undo the swap (drop the synthesized vias + stubs),
    # and re-route the affected pairs once.
    retry_bare_pad = [
        (pair_name, pair)
        for pair_name, pair in diff_pair_ids_to_route
        if pair_name in bare_pad_swaps
        and not (pair.p_net_id in routed_results and pair.n_net_id in routed_results)
        and not (pair.p_net_id in state.diff_pair_single_ended_nets
                 or pair.n_net_id in state.diff_pair_single_ended_nets)
    ]
    if retry_bare_pad:
        print("\n" + "=" * 60)
        print(f"Retrying {len(retry_bare_pad)} pair(s) without bare-pad target swap "
              f"(swap pinned a P/N crossing): "
              + ", ".join(name for name, _ in retry_bare_pad))
        print("=" * 60)

        def _drop(obj_list, doomed):
            doomed_ids = set(id(o) for o in doomed)
            obj_list[:] = [o for o in obj_list if id(o) not in doomed_ids]

        # The synthesized via/stub live on the pair's OWN nets, which are excluded
        # from the obstacle maps while that pair routes - so dropping them from
        # pcb_data (which reverts the pair's endpoints to the bare pads) is enough;
        # no obstacle-map rebuild is needed.
        for pair_name, _pair in retry_bare_pad:
            info = bare_pad_swaps.pop(pair_name)
            _drop(pcb_data.vias, info['vias'])
            _drop(all_swap_vias, info['vias'])
            _drop(pcb_data.segments, info['stubs'])
            _drop(all_swap_segments, info['stubs'])

        rb_s, rb_f, rb_t, rb_i, route_index = route_diff_pairs(state, retry_bare_pad)
        # The retried pairs were already tallied as failures in the first pass;
        # move any that now route from the failed column to the success column.
        successful += rb_s
        failed -= rb_s
        total_time += rb_t
        total_iterations += rb_i

    # Report nets whose far-apart (uncoupled) terminal pads were peeled off the
    # coupled chain (issue #121). Those pads are not a coupled differential
    # connection (e.g. spread-out test points), so they are left for a separate
    # single-ended pass (route.py); the plan-pcb-routing workflow sequences that
    # after this step. Surfaced here and in JSON_SUMMARY so the follow-up knows
    # which nets still need P->P / N->N routing.
    single_ended_followup = sorted(state.diff_pair_single_ended_nets.values())
    if single_ended_followup:
        print("\n" + "=" * 60)
        print(f"{len(single_ended_followup)} net(s) have far-apart terminal pads "
              f"peeled from the coupled chain - route them single-ended next:")
        for nm in single_ended_followup:
            print(f"  {nm}")
        print("  e.g.  route.py <this_output> <out2> --nets " +
              " ".join(f"'{n}'" for n in single_ended_followup[:3]) +
              (" ..." if len(single_ended_followup) > 3 else ""))
        print("=" * 60)

    # Run reroute loop for nets that were ripped during diff pair routing
    rq_successful, rq_failed, rq_time, rq_iterations, route_index = run_reroute_loop(
        state, route_index_start=route_index,
        cancel_check=cancel_check, progress_callback=progress_callback,
        failed_so_far=failed
    )
    successful += rq_successful
    failed += rq_failed
    total_time += rq_time
    total_iterations += rq_iterations

    # ----- Casualties-only final reconciliation (depth 1) -------------------
    # Nets ripped during diff-pair routing whose reroute never landed used to
    # ship at ZERO copper with no custody. Restore-first: verify actually
    # broken, restore the original geometry (collision-aware - never leaves
    # restored copper colliding with newly routed copper), then re-route the
    # still-broken with the restored copper as obstacles. Engine-side, so the
    # GUI inherits it. No-op when no casualties occurred.
    from diff_pair_custody import (run_casualty_reconcile, audit_pair_members,
                                   build_pair_reports,
                                   run_single_ended_member_fallback)
    casualty_summary = run_casualty_reconcile(
        state, progress_callback=progress_callback, cancel_check=cancel_check)
    _recovered = (len(casualty_summary['restored'])
                  + len(casualty_summary['rerouted']))
    if _recovered:
        # Each recovered casualty was tallied failed when its reroute failed;
        # move it back to the success column.
        successful += _recovered
        failed = max(0, failed - _recovered)

    # ----- Single-ended member fallback (Class 2: BOTH members) -------------
    # Pairs with NO coupled result (deferred to "the single-ended follow-up",
    # or hard-failed) used to leave the diff step with neither member
    # attempted -- relying on a downstream route.py pass that a chain/GUI run
    # may not have, and under which one member could route while the other
    # was silently dropped (ulx3s FPDI_D1+). Attempt BOTH members single-ended
    # here (no rip-up; failures stay honestly listed for the downstream pass).
    # Engine-side, so the GUI diff tab inherits it.
    se_fallback_summary = run_single_ended_member_fallback(
        state, diff_pair_ids_to_route)

    # Apply length matching if configured
    if length_match_groups:
        run_length_matching(routed_results, length_match_groups, config, pcb_data)

    # Apply intra-pair P/N length matching if configured
    intra_pair_reports = []
    if config.diff_pair_intra_match:
        print("\n" + "=" * 60)
        print("Intra-pair P/N length matching")
        print("=" * 60)

        # AC-coupled (XNet) member pairs are matched end-to-end below, not
        # per-side; skip them here so the two passes don't fight
        # (double-meander). Gated on the flag so intra-pair behavior is
        # unchanged when --ac-couple-match is off.
        xnet_member_p_net_ids = set()
        if config.ac_couple_match:
            for _xn in ac_xnets:
                for _m in _xn.members:
                    xnet_member_p_net_ids.add(_m.p_net_id)
        # #766: drive off the PAIR LIST, not the shape of routed_results. The
        # old loop required each result to carry `is_diff_pair` + `p_net_id`,
        # which the direct-hybrid escape does not stamp -- so a hybrid-routed
        # pair was skipped without printing anything, and its P/N skew shipped
        # unmeasured while the run reported the pair routed.
        intra_pair_reports = run_intra_pair_matching(
            diff_pair_ids_to_route, routed_results, config, pcb_data,
            skip_p_net_ids=xnet_member_p_net_ids)
        # 'skipped' pairs are matched end-to-end by the AC-couple pass, so they
        # belong in neither column -- counting them either way misreports.
        _isk = [r for r in intra_pair_reports if r['status'] == 'skipped']
        _im = [r for r in intra_pair_reports
               if r['status'] in ('matched', 'within-tolerance')]
        _iu = [r for r in intra_pair_reports
               if r['status'] not in ('matched', 'within-tolerance', 'skipped')]
        print(f"\n  Intra-pair: {len(_im)}/{len(_im) + len(_iu)} pair(s) within "
              f"{config.length_match_tolerance}mm"
              + (f" ({len(_isk)} matched end-to-end by the AC-coupled pass)"
                 if _isk else ""))
        if _iu:
            print(f"  {RED}Intra-pair NOT matched: " + ", ".join(
                f"{r['pair']} ({r['reason'] or r['status']}"
                + (f", delta={r['delta_mm']:.3f}mm" if r['delta_mm'] is not None else "")
                + ")" for r in _iu) + RESET)

    # Apply end-to-end AC-coupled (XNet) length matching if configured (#196).
    # Runs AFTER group + intra-pair matching; for its member pairs it supersedes
    # per-side intra-pair (skipped above) by matching the concatenated P vs N path
    # and placing the compensating meanders on whichever segment has room.
    ac_coupled_summary = []
    if config.ac_couple_match and ac_xnets:
        from length_matching import apply_ac_coupled_length_matching
        print("\n" + "=" * 60)
        print("End-to-end AC-coupled diff-pair length matching (#196)")
        print("=" * 60)
        for _xn in ac_xnets:
            _skew = apply_ac_coupled_length_matching(_xn, routed_results, config, pcb_data)
            if _skew is not None:
                ac_coupled_summary.append({
                    'nets': "+".join(_xn.base_names),
                    'skew_mm': round(_skew, 4),
                    'bridges': _xn.bridge_refs,
                })

    # Sync pcb_data with length-matched segments
    sync_pcb_data_segments(pcb_data, routed_results, original_segment_ids, state, config)

    # #521: coupled pair copper is an invariant later chain steps cannot
    # reproduce (P/N geometry, gap, polarity) -- mark routed members protected
    # so downstream rip steps skip them. Engine-side: the GUI diff tab and the
    # AI-plan executor inherit the noting; the writeback next to the DRC-floor
    # persistence records it in the sibling .kicad_pro.
    from protected_nets import note_protection_candidates
    _prot = {}
    for _nid, _res in routed_results.items():
        if _res and _res.get('is_diff_pair') and not _res.get('failed'):
            _net = pcb_data.nets.get(_nid)
            if _net and _net.name:
                _prot[_net.name] = 'diff-pair'
    if _prot:
        note_protection_candidates(_prot)

    # ----- Post-route cleanup (#215): the ONE shared pipeline ---------------
    # Same passes, ordering and strip plumbing as route.py's single-ended
    # cleanup (cleanup_pipeline.py), scoped to the diff-pair nets so other
    # copper is untouched. snap/phantom/neck are off for parity with this
    # front's historical pass set. octolinear IS enabled (deliberately, #318):
    # its re-bend keeps the anchor endpoints coincident and its clears() gate
    # includes the partner net, so a grazing terminal jog the prune must now
    # KEEP (tightened coincidence gate) gets re-bent clear instead of shipping
    # as a violation.
    from cleanup_pipeline import run_post_route_cleanup
    # ONLY fully-routed pairs: a failed / single-ended-deferred pair's fanout stubs
    # have free ends by design (the single-ended follow-up connects them), so a
    # dead-end sweep there would strip copper the next pass needs.
    dp_scope = set()
    from diff_pair_custody import _member_connected as _dpc_member_connected
    for _pn, _pair in diff_pair_ids_to_route:
        _rr_p = routed_results.get(_pair.p_net_id)
        _rr_n = routed_results.get(_pair.n_net_id)
        if _rr_p is None or _rr_n is None:
            continue
        if (_rr_p.get('se_member_fallback') or _rr_n.get('se_member_fallback')):
            # Single-ended-fallback copper: sweep it only when BOTH members
            # ended fully connected -- a 'partial' member's remaining stubs
            # are load-bearing for the downstream pass, exactly like a
            # deferred pair's.
            if not (_dpc_member_connected(pcb_data, _pair.p_net_id)
                    and _dpc_member_connected(pcb_data, _pair.n_net_id)):
                continue
        dp_scope.add(_pair.p_net_id)
        dp_scope.add(_pair.n_net_id)
    _dp_cleanup = run_post_route_cleanup(
        results, pcb_data, dp_scope, config,
        label='Diff-pair ', snap=False, phantom=False, neck=False,
        keep_input_copper=keep_input_copper,
        progress_callback=progress_callback)
    cleanup_input_strip = _dp_cleanup.input_strip_segments
    # #508 finding 11: the pipeline's removed input VIAS were dropped on the
    # floor here -- the orphan-island sweep deletes them from pcb_data, but
    # neither the writer nor the GUI ever heard, so the output kept via
    # barrels the router had withdrawn (firing: cm4_underwater step4_diff).
    cleanup_input_strip_vias = list(
        getattr(_dp_cleanup, 'input_strip_vias', None) or [])

    # Build summary data
    import json
    # Member audit: verify BOTH members' connectivity claims against the actual
    # pad connectivity on the final copper (post-sync, post-cleanup) - known
    # cases exist of one member silently incomplete. Feeds the per-pair reports.
    member_audit = audit_pair_members(pcb_data, diff_pair_ids_to_route)
    pair_reports = build_pair_reports(state, diff_pair_ids_to_route,
                                      member_audit,
                                      skipped_fanout=skipped_fanout_info)
    _audit_mismatches = [r for r in pair_reports if r['member_audit_mismatch']]
    for _r in _audit_mismatches:
        # The pair CLAIMED 'coupled'; build_pair_reports has already demoted
        # its outcome to 'incomplete' (#602), so name the claim, not the
        # post-demotion value.
        print(f"{RED}MEMBER AUDIT MISMATCH: {_r['pair']} reported "
              f"'coupled' but member(s) with disconnected pads: "
              f"{', '.join(_r['incomplete_members'])} "
              f"-> outcome '{_r['outcome']}'{RESET}")
    # #602: the audit's own verdict as a machine-readable field. Every pair
    # with ANY disconnected member pad lands here -- both the contradicted
    # 'coupled' claims above and the by-design 'partial' pairs whose peeled
    # terminals the single-ended follow-up still has to close -- so a caller
    # can gate on it without deciding which of those it is, and without
    # parsing the MEMBER AUDIT prose out of the log.
    _member_incomplete = sorted({r['pair'] for r in pair_reports
                                 if r.get('incomplete_members')})
    routed_diff_pairs = []
    failed_diff_pairs = []
    single_ended_diff_pairs = []
    partial_diff_pairs = []
    # #514 (-> #261): the MEMBER AUDIT already knows when a "routed" pair left
    # member pads disconnected, but the summary lists ignored it -- so
    # JSON_SUMMARY said successful while check_connected showed 0% members
    # (peaksat_comms CAN_A/CAN_B/RX09; muzy_zynq4, lwdo_sdr, sipm_bias).
    # A pair the audit flags is reported PARTIAL, never routed.
    _partial_by_audit = {r['pair'] for r in pair_reports
                         if r.get('member_audit_mismatch')}
    for pair_name, pair in diff_pair_ids_to_route:
        _rr_p = routed_results.get(pair.p_net_id)
        _rr_n = routed_results.get(pair.n_net_id)
        if (_rr_p is not None and _rr_n is not None
                and not (_rr_p.get('se_member_fallback')
                         or _rr_n.get('se_member_fallback'))):
            # A member routed by the single-ended member fallback is NOT a
            # coupled route -- without the flag check, a deferred/failed pair
            # whose members were both connected single-ended would count as a
            # routed diff pair.
            if pair_name in _partial_by_audit:
                partial_diff_pairs.append(pair_name)
            else:
                routed_diff_pairs.append(pair_name)
        elif (pair.p_net_id in state.diff_pair_single_ended_nets
              or pair.n_net_id in state.diff_pair_single_ended_nets):
            # Intentionally left for single-ended routing (electrically short, or
            # fully peeled) - not a coupled route, but not a failure either.
            single_ended_diff_pairs.append(pair_name)
        else:
            failed_diff_pairs.append(pair_name)

    # A pair the audit demoted to PARTIAL was already counted in `successful`
    # by the coupled-route loop, so the headline and the JSON key still claimed
    # it. Take it back here, after the audit has spoken and BEFORE either the
    # printed summary or the returned tuple reads the counter -- the GUI reads
    # `successful` raw (differential_gui.py), so fixing only the print site
    # would leave the GUI and JSON_SUMMARY saying different things than the
    # member audit sitting next to them.
    if partial_diff_pairs:
        successful = max(0, successful - len(partial_diff_pairs))

    # Count total vias from results
    total_vias = sum(len(r.get('new_vias', [])) for r in results)

    # Print human-readable summary
    print("\n" + "=" * 60)
    print("Routing complete")
    print("=" * 60)
    if failed_diff_pairs:
        print(f"  {RED}Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed ({len(failed_diff_pairs)} FAILED){RESET}")
    else:
        print(f"  Diff pairs:    {len(routed_diff_pairs)}/{len(diff_pair_ids_to_route)} routed")
    if partial_diff_pairs:
        print(f"  {RED}Partial:       {len(partial_diff_pairs)} pair(s) whose coupled "
              f"route left member pads disconnected (see MEMBER AUDIT): "
              f"{', '.join(sorted(partial_diff_pairs))}{RESET}")
    if _member_incomplete:
        # Counted separately from the buckets above: a by-design 'partial'
        # pair is still in routed_diff_pairs (its trunk IS coupled), so the
        # coupled count alone never tells you its pads are open (#602).
        print(f"  Member-incomplete: {len(_member_incomplete)} pair(s) with "
              f"disconnected member pads at this step "
              f"({', '.join(_member_incomplete)}) — JSON_SUMMARY."
              f"member_incomplete_pairs")
    # #766: pairs routed by the hybrid escape are fully routed and keep their
    # credit, but their TERMINAL legs are point-to-point single-ended copper --
    # not a coupled pair end to end. Name them, because 'coupled' in the JSON
    # says the opposite and the terminals are where P/N skew is born.
    _hyb = sorted(r['pair'] for r in pair_reports if r.get('escape') == 'hybrid')
    if _hyb:
        print(f"  {YELLOW}Hybrid escape: {len(_hyb)} pair(s) routed as a coupled middle "
              f"+ point-to-point terminal legs (terminals NOT coupled): "
              f"{', '.join(_hyb)} — JSON_SUMMARY.pair_reports[].coupled_terminals{RESET}")
    if ripup_success_pairs:
        print(f"  Rip-up success: {len(ripup_success_pairs)} (routes that ripped blockers)")
    if rerouted_pairs:
        print(f"  Rerouted:      {len(rerouted_pairs)} (ripped nets re-routed)")
    if polarity_swapped_pairs:
        print(f"  Polarity swaps: {len(polarity_swapped_pairs)}")
    if state.polarity_swap_denied_pairs:
        print(f"  Polarity swaps denied: {len(state.polarity_swap_denied_pairs)} "
              f"(mismatch found but pair not in --polarity-swap-nets): "
              f"{', '.join(sorted(state.polarity_swap_denied_pairs))}")
    if target_swaps:
        swap_pairs = summarize_target_swaps(target_swaps)
        print(f"  Target swaps:  {len(swap_pairs)}")
    if single_ended_diff_pairs:
        print(f"  Single-ended:  {len(single_ended_diff_pairs)} (electrically short - "
              f"deferred to single-ended routing)")
        # ...and say how those members ACTUALLY came out. The headline above
        # reads `Diff pairs: 0/2 routed` on a step where every member net got
        # copper and connected, so a log tail -- which is what a grep, a CI
        # summary or a person reads -- shows total failure off a fully
        # successful step. Run 14 had to open the JSON to find out otherwise.
        _fb = se_fallback_summary or {}
        _rt, _pt, _fl = (len(_fb.get('routed') or []),
                         len(_fb.get('partial') or []),
                         len(_fb.get('failed') or []))
        if _rt or _pt or _fl:
            _tail = (f", {_pt} partial" if _pt else '') + \
                    (f", {RED}{_fl} FAILED{RESET}" if _fl else '')
            print(f"  SE fallback:   {_rt}/{_rt + _pt + _fl} member net(s) "
                  f"routed{_tail}")
    if skipped_bad_fanout:
        print(f"  {RED}Skipped:       {len(skipped_bad_fanout)} (fanout stubs self-overlap - "
              f"fix the fanout, #242){RESET}")
    print(f"  Total vias:    {total_vias}")
    print(f"  Total time:    {total_time:.2f}s")
    print(f"  Iterations:    {total_iterations:,}")
    from target_swap import summarize_target_swaps  # cycle-safe swap list (#380)
    summary = {
        'routed_diff_pairs': routed_diff_pairs,
        'failed_diff_pairs': failed_diff_pairs,
        'partial_diff_pairs': sorted(partial_diff_pairs),
        # #602: every pair the member audit found with a disconnected member
        # pad, whatever bucket it landed in. Gate on THIS for "did the pair's
        # copper actually reach its pads", instead of inferring it from the
        # coupled count (which by design still includes a partial pair whose
        # trunk is coupled and whose peeled terminals close later).
        'member_incomplete_pairs': _member_incomplete,
        'single_ended_diff_pairs': single_ended_diff_pairs,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'polarity_swapped_pairs': sorted(polarity_swapped_pairs),
        'polarity_swap_denied_pairs': sorted(state.polarity_swap_denied_pairs),
        'single_ended_followup_nets': sorted(state.diff_pair_single_ended_nets.values()),
        'skipped_bad_fanout': sorted(skipped_bad_fanout),
        'target_swaps': [{'pair1': k, 'pair2': v} for k, v in summarize_target_swaps(target_swaps)],
        'layer_swaps': total_layer_swaps,
        # #766: per-pair intra-pair P/N matching outcome. Present whenever
        # --diff-pair-intra-match ran; one record per pair in the run, so a
        # caller can see which pairs shipped unmatched (and their skew) instead
        # of inferring it from `successful`, which says nothing about skew.
        'intra_pair_matching': intra_pair_reports,
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias,
        # Smallest copper clearance any step actually routed at (e.g. fine-pitch
        # taps below the nominal). Grade/check_drc the board at this floor.
        'min_clearance_used': __import__('clearance_ledger').effective(clearance),
        # Per-pair structured records (additive): outcome, failure reason code
        # + stage, blocking nets, member connectivity audit. See
        # diff_pair_custody.build_pair_reports for the schema.
        'pair_reports': pair_reports,
        # Casualties-only final reconciliation tally (additive): nets ripped
        # during routing whose reroute failed - restored / rerouted / partial /
        # unrecovered.
        'casualty_reconcile': casualty_summary,
        # Single-ended member fallback tally (additive): members of non-coupled
        # pairs attempted P->P / N->N in-run (Class 2 - both members owned).
        'single_ended_fallback': se_fallback_summary,
    }
    if ac_coupled_summary:
        summary['ac_coupled_xnets'] = ac_coupled_summary
    if impedance_width_clamped:
        # #610: layers whose impedance-solved width was clamped UP to the
        # width floor, {layer: [solved_mm, floor_mm]} -- those layers will NOT
        # meet the impedance request. Key absent when no clamp fired.
        summary['impedance_width_clamped'] = impedance_width_clamped
    try:                       # #653: env knobs into the machine-readable
        import env_knobs as _ek653   # summary, so a harness can detect a
        summary['env_knobs'] = _ek653.active_env_knobs()   # dirty baseline
    except Exception:          # without re-reading logs
        pass
    print(f"JSON_SUMMARY: {json.dumps(summary)}")

    # Write output file or return results for direct application
    if return_results:
        # Return results data for direct application (e.g., KiCad plugin).
        # pad_swaps / target_swap_info / all_segment_modifications must be
        # applied to the live board just like write_routed_output applies them
        # to the output file - otherwise polarity/target swaps leave the routed
        # tracks pointing at pads that still carry the old net.
        results_data = {
            'results': results,
            'all_swap_vias': all_swap_vias,
            'all_swap_segments': all_swap_segments,
            'pad_swaps': pad_swaps,
            'target_swap_info': target_swap_info,
            'all_segment_modifications': all_segment_modifications,
            'exclusion_zone_lines': exclusion_zone_lines if debug_lines else [],
            'boundary_debug_labels': boundary_debug_labels if debug_lines else [],
            # Input-file copper the cleanup pipeline removed -- the CLI writer
            # strips these blocks (segments_to_remove in the write path);
            # without this key the GUI kept them on the live board (parity gap
            # found in the #319 restructure audit).
            'segments_to_remove': cleanup_input_strip,
            # #508 finding 11: removed input vias, same contract as
            # segments_to_remove (route.py parity; differential_gui applies).
            'vias_to_remove': cleanup_input_strip_vias,
            # successful/failed only count pairs that were coupled-routed or
            # outright failed -- electrically-short pairs deferred to the
            # single-ended pass, and pairs skipped for self-overlapping fanout,
            # land in neither bucket. Without these the GUI's "0 routed, 0
            # failed" result looks like nothing happened.
            'single_ended_diff_pairs': single_ended_diff_pairs,
            'skipped_bad_fanout': sorted(skipped_bad_fanout),
            # Per-pair diagnostics + casualty custody tally (additive), same
            # data as the CLI JSON_SUMMARY keys of the same names.
            'pair_reports': pair_reports,
            'casualty_reconcile': casualty_summary,
            'single_ended_fallback': se_fallback_summary,
        }
    else:
        wrote = write_routed_output(
            input_file=input_file,
            output_file=output_file,
            results=results,
            all_segment_modifications=all_segment_modifications,
            all_swap_vias=all_swap_vias,
            all_swap_segments=all_swap_segments,
            target_swap_info=target_swap_info,
            single_ended_target_swap_info=[],
            pad_swaps=pad_swaps,
            pcb_data=pcb_data,
            debug_lines=debug_lines,
            exclusion_zone_lines=exclusion_zone_lines,
            boundary_debug_labels=boundary_debug_labels,
            skip_routing=skip_routing,
            add_teardrops=add_teardrops,
            segments_to_remove=cleanup_input_strip or None,
            vias_to_remove=cleanup_input_strip_vias or None
        )
        # When no coupled copper was written -- every pair was deferred to
        # single-ended (electrically short), OR a pair could not be routed at all
        # (terminal/launch escape exhausted, issue #167) -- still pass the board
        # through unchanged so the pipeline never loses its output file. The
        # unrouted nets are picked up by the downstream single-ended route.py pass
        # (the chains route '*' from the diff step's output); without this the
        # whole chain FileNotFoundErrors on the missing board (issues #90, #167).
        if wrote and output_file:
            # Board-vs-file ledger (KICAD_BOARD_LEDGER=1, #508): the written
            # file must match pcb_data for every routed pair's nets -- this
            # engine had NO ledger call (findings 10/11 sat here). No-op
            # unless the env var is set.
            from cleanup_pipeline import verify_written_file_parity
            verify_written_file_parity(output_file, pcb_data, sorted(dp_scope),
                                       label=' diff')
        if not wrote and output_file:
            from pcb_io_utils import passthrough_copy
            passthrough_copy(input_file, output_file)
            if state.diff_pair_single_ended_nets:
                print(f"\nAll diff pairs deferred to single-ended (no coupled copper "
                      f"added); wrote board through to {output_file} for the "
                      f"single-ended pass")
            else:
                print(f"\nNo diff pair could be coupled-routed; wrote board through "
                      f"unchanged to {output_file} so the pipeline can continue "
                      f"(route the pair single-ended next)")

    # Update schematics with swap info if directory specified
    if schematic_dir and (target_swap_info or pad_swaps):
        schematic_swaps = []

        # Diff pair target swaps (P pads swap, N pads swap)
        for info in target_swap_info:
            if all(k in info for k in ['p1_p_pad', 'p2_p_pad', 'p1_n_pad', 'p2_n_pad']):
                p1_p = info['p1_p_pad']
                p2_p = info['p2_p_pad']
                p1_n = info['p1_n_pad']
                p2_n = info['p2_n_pad']
                # P pads swap
                if p1_p and p2_p and p1_p.component_ref == p2_p.component_ref:
                    schematic_swaps.append({
                        'component_ref': p1_p.component_ref,
                        'pad1': p1_p.pad_number,
                        'pad2': p2_p.pad_number
                    })
                # N pads swap
                if p1_n and p2_n and p1_n.component_ref == p2_n.component_ref:
                    schematic_swaps.append({
                        'component_ref': p1_n.component_ref,
                        'pad1': p1_n.pad_number,
                        'pad2': p2_n.pad_number
                    })

        # Polarity swaps (P/N swap within same diff pair)
        for swap_info in pad_swaps:
            if 'pad_p' in swap_info and 'pad_n' in swap_info:
                pad_p = swap_info['pad_p']
                pad_n = swap_info['pad_n']
                if pad_p and pad_n and pad_p.component_ref == pad_n.component_ref:
                    schematic_swaps.append({
                        'component_ref': pad_p.component_ref,
                        'pad1': pad_p.pad_number,
                        'pad2': pad_n.pad_number
                    })

        if schematic_swaps:
            apply_swaps_to_schematics(schematic_dir, schematic_swaps, verbose=verbose)

    # Route trace dump (#482): diff-pair per-copper timeline for animate_route.py.
    from route_trace import dump_trace as _dump_route_trace
    _dump_route_trace(pcb_data, output_file)

    # Final memory summary
    if debug_memory:
        final_mem = get_process_memory_mb()
        print("\n" + "=" * 60)
        print("[MEMORY] Final Memory Summary")
        print("=" * 60)
        print(f"  Process RSS: {final_mem:.1f} MB (delta: {final_mem - mem_start:+.1f} MB)")
        print(f"  Net obstacles cache: {estimate_net_obstacles_cache_mb(state.net_obstacles_cache):.1f} MB ({len(state.net_obstacles_cache)} nets)")
        print(f"  Track proximity cache: {estimate_track_proximity_cache_mb(state.track_proximity_cache):.1f} MB ({len(state.track_proximity_cache)} nets)")
        print(f"  Routed paths: {estimate_routed_paths_mb(state.routed_net_paths):.1f} MB ({len(state.routed_net_paths)} nets)")
        print(f"  Routed results: {len(state.routed_results)} nets")
        print(format_obstacle_map_stats(state.working_obstacles))
        print("=" * 60)

    # Obstacle-map ref-count integrity audit (#309), same invariant as route.py:
    # the diff loop maintains its own persistent working map + per-net caches.
    if env_knobs.OBSTACLE_AUDIT:
        from obstacle_cache import run_obstacle_audit
        run_obstacle_audit(base_obstacles, state.working_obstacles,
                           state.net_obstacles_cache, label="route_diff")

    if return_results:
        return successful, failed, total_time, results_data
    return successful, failed, total_time

if __name__ == "__main__":
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    # CMD/EXIT self-echo (run-3 B1); see route.py for the external-kill caveat.
    # CLI-`__main__`-only: the GUI imports the engine function.
    import cli_banner
    cli_banner.install()
    import argparse
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set

    parser = argparse.ArgumentParser(
        description="Batch Differential Pair PCB Router - Routes differential pairs using Rust-accelerated A*",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "*lvds*"             - matches any net containing lvds
  "Net-(U1*DQS*)"      - matches Net-(U1-DQS_P), Net-(U1-DQS_N), etc.

All nets matching the patterns are treated as differential pairs.
Nets with _P/_N, P/N, or +/- suffixes will be paired and routed together.

Examples:
  python py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"
  python py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "Net-(U1*DQS*)" "Net-(U1*CK_*)"
"""
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    parser.add_argument("--output", metavar="FILE",
                        help="Output KiCad PCB file (named alias for the positional output_file)")
    parser.add_argument("net_patterns", nargs="*", help="Net names or wildcard patterns to route (default: '*' = all nets)")
    parser.add_argument("--nets", "-n", nargs="+", help="Net names or wildcard patterns to route (alternative to positional args)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")

    # Ordering and strategy options
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original"],
                        default="mps",
                        help="Net ordering strategy: mps (default, crossing conflicts), inside_out, or original")
    parser.add_argument("--direction", "-d", choices=["forward", "backward"],
                        default=None,
                        help="Direction search order for each net route")
    parser.add_argument("--no-bga-zones", nargs="*", default=None,
                        help="Disable BGA exclusion zones. No args = disable all. With component refs (e.g., U1 U3) = disable only those.")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=['F.Cu', 'B.Cu'],
                        help="Routing layers to use (default: F.Cu B.Cu)")
    parser.add_argument("--layer-costs", nargs="+", type=float, default=None,
                        help="Per-layer cost multipliers (1.0-1000, or any negative value e.g. -1 = "
                             "forbidden: the layer is an obstacle / via span but carries no routed copper), one per --layers "
                             "entry, to bias which layer(s) coupled diff pairs prefer (e.g. '1 1 1 3' "
                             "to discourage the 4th layer). Default: 1.0 on every layer "
                             "(behavior unchanged). Matches route.py / route_planes.")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=None,
                        help="Differential-pair leg width in mm (default: the board Default "
                             "net-class diff_pair_width, else 0.3). With --impedance it is the "
                             "min-width floor and the no-stackup fallback (per-layer impedance "
                             "width can only widen it), not ignored.")
    parser.add_argument("--impedance", type=float, default=None,
                        help="Target differential impedance in ohms (e.g., 100). Calculates track width per layer from board stackup using --diff-pair-gap as spacing.")
    parser.add_argument("--coplanar-gap", type=float, default=defaults.COPLANAR_GAP,
                        help="Declare that the pairs in this call run through a ground "
                             "pour on their OWN layer, this far (mm) from each trace's "
                             "OUTER edge. Outer layers then use the coplanar-waveguide-"
                             "over-ground model instead of microstrip, giving a NARROWER "
                             "trace for the same target ohms. Applies to EVERY pair in "
                             "the call (unlike route.py there is no --coplanar-nets: the "
                             "diff engine bakes one width per layer into the obstacle "
                             "map). Split interfaces into separate calls to mix. The pour "
                             "does not exist yet, so this is a DECLARATION: pour with a "
                             "matching 'route_planes --zone-clearance' and verify with "
                             "'check_impedance.py --coplanar-gap'. Requires --impedance.")
    parser.add_argument("--clearance", type=float, default=None,
                        help="Copper clearance of the DEFAULT net class for this run, in mm; "
                             "other classes route at their own clearance (pairwise max). When "
                             f"OMITTED, the board's Default class, else {defaults.CLEARANCE}. "
                             "--clearance-ceiling caps every class (the old #439 behaviour). "
                             "Use --net-clearances <json> for explicit per-net values.")
    parser.add_argument("--net-clearances", metavar="JSON", default=None,
                        help="Explicit override for the cross-class clearance map: a JSON object "
                             "mapping net name -> that net's net-class clearance in mm. When OMITTED, "
                             "the map is AUTO-READ from the sibling .kicad_pro's non-Default "
                             "netclasses (all-Default boards -> empty -> inert). Every pre-placed AND "
                             "in-run obstacle of a different class is priced at max(routing floor, "
                             "that obstacle net's own clearance) = KiCad's cross-class max(A,B). The "
                             "GUI derives the same map from the board's live net classes.")
    parser.add_argument("--via-size", type=float, default=None,
                        help="Via outer diameter in mm (default: the board Default net-class via, else 0.5)")
    parser.add_argument("--via-drill", type=float, default=None,
                        help="Via drill size in mm (default: the board Default net-class via drill, else 0.3)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help="Grid resolution in mm (default: 0.1)")
    parser.add_argument("--via-cost", type=int, default=defaults.VIA_COST,
                        help=f"Penalty for placing a via, in 0.1mm grid steps (default: {defaults.VIA_COST}, doubled for diff pairs; mm-equivalent at any --grid-step)")
    parser.add_argument("--via-proximity-cost", type=int, default=defaults.VIA_PROXIMITY_COST,
                        help="Via cost multiplier in stub/BGA proximity zones (default: 10, 0=no extra cost)")
    parser.add_argument("--max-iterations", type=int, default=defaults.MAX_ITERATIONS,
                        help="Max A* iterations before giving up (default: 200000)")
    parser.add_argument("--max-probe-iterations", type=int, default=defaults.MAX_PROBE_ITERATIONS,
                        help="Max iterations for quick probe phase per direction (default: 5000)")
    parser.add_argument("--heuristic-weight", type=float, default=defaults.HEURISTIC_WEIGHT,
                        help=f"A* heuristic weight, higher=faster but less optimal (default: {defaults.HEURISTIC_WEIGHT})")
    parser.add_argument("--proximity-heuristic-factor", type=float,
                        default=defaults.PROXIMITY_HEURISTIC_FACTOR,
                        help=f"Factor for proximity-aware A* heuristic (default: {defaults.PROXIMITY_HEURISTIC_FACTOR}, 0=disabled)")
    parser.add_argument("--turn-cost", type=int, default=defaults.TURN_COST,
                        help="Penalty for direction changes, encourages straighter paths (default: 1000)")
    parser.add_argument("--direction-preference-cost", type=int, default=defaults.DIRECTION_PREFERENCE_COST,
                        help=f"Penalty for non-preferred layer direction, 0=disabled (default: {defaults.DIRECTION_PREFERENCE_COST})")
    parser.add_argument("--bus", action="store_true",
                        help="Enable auto-detection and routing of bus groups (nets with clustered endpoints)")
    parser.add_argument("--bus-detection-radius", type=float, default=defaults.BUS_DETECTION_RADIUS,
                        help=f"Max endpoint distance to form bus in mm (default: {defaults.BUS_DETECTION_RADIUS})")
    parser.add_argument("--bus-attraction-radius", type=float, default=defaults.BUS_ATTRACTION_RADIUS,
                        help=f"Attraction radius from neighbor track in mm (default: {defaults.BUS_ATTRACTION_RADIUS})")
    parser.add_argument("--bus-attraction-bonus", type=int, default=defaults.BUS_ATTRACTION_BONUS,
                        help=f"Cost bonus for staying near neighbor track (default: {defaults.BUS_ATTRACTION_BONUS})")
    parser.add_argument("--bus-min-nets", type=int, default=defaults.BUS_MIN_NETS,
                        help=f"Minimum nets to form a bus group (default: {defaults.BUS_MIN_NETS})")
    parser.add_argument("--keepout", action="store_true",
                        help="Keep routed tracks out of polygons drawn on a User layer (issue #27)")
    parser.add_argument("--keepout-layer", type=str, default=defaults.KEEPOUT_LAYER,
                        help=f"User layer the keepout polygons are drawn on (default: {defaults.KEEPOUT_LAYER})")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=defaults.STUB_PROXIMITY_RADIUS,
                        help="Radius around stubs to penalize routing in mm (default: 2.0)")
    parser.add_argument("--stub-proximity-cost", type=float, default=defaults.STUB_PROXIMITY_COST,
                        help="Cost penalty near stubs in mm equivalent (default: 0.2)")

    # BGA proximity penalty
    parser.add_argument("--bga-proximity-radius", type=float, default=defaults.BGA_PROXIMITY_RADIUS,
                        help="Radius around BGA edges to penalize routing in mm (default: 7.0)")
    parser.add_argument("--bga-proximity-cost", type=float, default=defaults.BGA_PROXIMITY_COST,
                        help="Cost penalty near BGA edges in mm equivalent (default: 0.2)")

    # Track proximity penalty (same layer only)
    parser.add_argument("--track-proximity-distance", type=float, default=defaults.TRACK_PROXIMITY_DISTANCE,
                        help="Radius around routed tracks in mm, same layer only (0 = disabled, default: 2.0)")
    parser.add_argument("--track-proximity-cost", type=float, default=defaults.TRACK_PROXIMITY_COST,
                        help="Cost penalty near routed tracks (0 = disabled, default: 0.0)")

    # Differential pair routing options
    parser.add_argument("--diff-pair-gap", type=float, default=None,
                        help="Gap between P and N traces of differential pairs in mm "
                             "(default: the board Default net-class diff_pair_gap, else 0.101)")
    parser.add_argument("--diff-pair-centerline-setback", type=float, default=None,
                        help="Distance in front of stubs to start centerline route in mm (default: 2x P-N spacing)")
    parser.add_argument("--min-turning-radius", type=float, default=0.2,
                        help="Minimum turning radius for pose-based routing in mm (default: 0.2)")
    parser.add_argument("--polarity-swap-nets", nargs="+", metavar="PATTERN",
                        help="Glob patterns naming the diff pairs ALLOWED to resolve a P/N "
                             "polarity mismatch by swapping pad net assignments (#279). "
                             "Omit for no swaps ever (default - a swap is only harmless when "
                             "an endpoint can compensate, e.g. FPGA generic I/O; never USB); "
                             "pass '*' to allow all pairs. Same matcher as --nets.")
    parser.add_argument("--no-stub-layer-swap", action="store_true",
                        help="Disable stub layer switching optimization (enabled by default)")
    parser.add_argument("--no-crossing-layer-check", action="store_true",
                        help="Count crossings regardless of layer overlap (by default, only same-layer crossings count)")
    parser.add_argument("--no-gnd-vias", action="store_true",
                        help="Disable GND via placement near diff pair signal vias (enabled by default)")
    parser.add_argument("--can-swap-to-top-layer", action="store_true",
                        help="Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.")
    parser.add_argument("--swappable-nets", nargs="+",
                        help="Glob patterns for diff pair nets that can have targets swapped (e.g., 'rx1_*')")
    parser.add_argument("--schematic-dir", default=None,
                        help="Directory containing .kicad_sch files to update with pad swaps (default: no schematic update)")
    parser.add_argument("--crossing-penalty", type=float, default=defaults.CROSSING_PENALTY,
                        help="Penalty for crossing assignments in target swap optimization (default: 1000.0)")
    parser.add_argument("--mps-reverse-rounds", action="store_true",
                        help="Reverse MPS round order: route most-conflicting groups first instead of least-conflicting")
    parser.add_argument("--mps-layer-swap", action="store_true",
                        help="Enable MPS-aware layer swaps to reduce crossing conflicts")
    parser.add_argument("--mps-segment-intersection", action="store_true",
                        help="Force MPS to use segment intersection for crossing detection")
    parser.add_argument("--keep-input-copper", action="store_true",
                        help="Treat the input file's own copper as read-only in the post-route "
                             "cleanup passes (see route.py --keep-input-copper)")

    # Length matching options
    parser.add_argument("--length-match-group", action="append", nargs="+", dest="length_match_groups",
                        help="Net patterns to length-match as a group (can be repeated). Use 'auto' for DDR4 auto-grouping")
    parser.add_argument("--length-match-tolerance", type=float, default=defaults.LENGTH_MATCH_TOLERANCE,
                        help="Acceptable length variance within group in mm (default: 0.1)")
    parser.add_argument("--meander-amplitude", type=float, default=defaults.MEANDER_AMPLITUDE,
                        help="Height of meander perpendicular to trace in mm (default: 1.0)")
    parser.add_argument("--meander-spacing", type=float, default=defaults.MEANDER_SPACING,
                        help="Centre-to-centre spacing of adjacent single-ended meander arms, in "
                             "multiples of the net's routed track width; used for intra-pair P/N "
                             "and AC-coupled matching (default: 2.0 = 2W). Pair-centerline "
                             "meanders scale with the pair gap instead (--diff-chamfer-extra).")

    # Time matching options (alternative to length matching)
    parser.add_argument("--time-matching", action="store_true",
                        help="Match by propagation time instead of length (accounts for layer dielectric)")
    parser.add_argument("--time-match-tolerance", type=float, default=defaults.TIME_MATCH_TOLERANCE,
                        help="Acceptable time variance in picoseconds (default: 1.0)")

    parser.add_argument("--diff-chamfer-extra", type=float, default=1.5,
                        help="Chamfer multiplier for diff pair meanders (default: 1.5, >1 avoids P/N crossings)")
    parser.add_argument("--diff-pair-intra-match", action="store_true",
                        help="Enable intra-pair P/N length matching (add meanders to shorter track of each diff pair)")
    parser.add_argument("--ac-couple-match", action="store_true",
                        help="End-to-end length-match AC-coupled differential pairs split by series DC-blocking "
                             "caps (#196): auto-detect the cap chain, match the concatenated P path vs the N path, "
                             "and place the compensating meanders on whichever segment has room. Off by default.")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=defaults.MAX_RIPUP,
                        help="Maximum blockers to rip up at once during rip-up and retry (default: 3)")
    parser.add_argument("--ripup-blocker-select",
                        choices=list(defaults.RIPUP_BLOCKER_SELECT_CHOICES),
                        default=defaults.RIPUP_BLOCKER_SELECT,
                        help="""Blocker SELECTION algorithm for the rip-up ladder (see route.py --help / docs/rip-up-reroute.md)""")
    parser.add_argument("--max-setback-angle", type=float, default=45.0,
                        help="Maximum angle (degrees) for setback position search (default: 45.0)")
    parser.add_argument("--routing-clearance-margin", type=float, default=defaults.ROUTING_CLEARANCE_MARGIN,
                        help="Multiplier on track-via clearance (1.0 = minimum DRC)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=None,
                        help="Minimum clearance between drill holes in mm. Default: the "
                             f"board's own min_hole_to_hole constraint, else {defaults.HOLE_TO_HOLE_CLEARANCE}.")
    parser.add_argument("--board-edge-clearance", type=float, default=None,
                        help="Clearance from board edge in mm. Default: the board's own "
                             f"min_copper_edge_clearance constraint, else {defaults.BOARD_EDGE_CLEARANCE}.")
    parser.add_argument("--same-net-pad-clearance", type=float, default=None,
                        help="Edge-to-edge clearance (mm) between EVERY placed via and "
                             "same-net pads (#581). > 0 keeps vias off same-net SMD pads "
                             "(escape vias, via-in-pad rescue, tap vias) and is recorded "
                             "in the sibling .kicad_pro so later chain steps inherit it; "
                             "-1 explicitly allows via-in-pad. Default: the project's "
                             "recorded value, else via-in-pad allowed.")
    parser.add_argument("--max-turn-angle", type=float, default=180.0,
                        help="Max cumulative turn angle (degrees) before reset, to prevent U-turns (default: 180)")

    # Vertical alignment attraction options
    parser.add_argument("--vertical-attraction-radius", type=float, default=defaults.VERTICAL_ATTRACTION_RADIUS,
                        help="Radius in mm for cross-layer track attraction (0 = disabled, default: 1.0)")
    parser.add_argument("--vertical-attraction-cost", type=float, default=defaults.VERTICAL_ATTRACTION_COST,
                        help="Cost bonus for aligning with tracks on other layers (0 = disabled, default: 0.0)")

    # Ripped route avoidance options
    parser.add_argument("--ripped-route-avoidance-radius", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS,
                        help=f"Radius in mm around ripped route segments/vias for soft penalty (default: {defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS})")
    parser.add_argument("--ripped-route-avoidance-cost", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_COST,
                        help=f"Soft penalty cost for routing through ripped corridors (0 = disabled, default: {defaults.RIPPED_ROUTE_AVOIDANCE_COST})")

    # Debug options
    parser.add_argument("--debug-lines", action="store_true",
                        help="Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8 (simplified), User.9 (raw A*)")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed diagnostic output (setback checks, etc.)")
    parser.add_argument("--skip-routing", action="store_true",
                        help="Skip actual routing, only do swaps and write debug info")
    parser.add_argument("--debug-memory", action="store_true",
                        help="Print memory usage statistics at key points during routing")
    parser.add_argument("--add-teardrops", action="store_true",
                        help="Add teardrop settings to all pads and vias in output file")
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    # #439: the PRESENCE of --clearance is the clamp switch (see route.py). Given ->
    # non-Default classes capped at min(class, --clearance) + writeback clamps.
    # Omitted -> honor classes: base = board Default net-class clearance, classes
    # uncapped, writeback preserves. --hole-to-hole-clearance / --board-edge-clearance
    # default to the board's own constraint minimum when omitted. Resolved before
    # enforce_fab_floors; _clamp_netclasses is stashed for drc_fix_kwargs.
    from list_nets import (board_default_netclass_clearance, board_default_netclass_param,
                           resolve_cli_floor)
    # #435: whether the diff geometry was EXPLICITLY set on the CLI. If NOT, each
    # pair falls back engine-side to its OWN netclass diff_pair_gap/width (not the
    # board Default class), so a multi-class board routes every pair to its own
    # impedance geometry. --impedance no longer counts as explicit here (#610):
    # the engine guards the netclass-width path itself when impedance is set, and
    # uses this bit to floor impedance-solved widths at the fab tier instead of
    # the resolved default pair width.
    _dp_width_explicit = args.track_width is not None
    _dp_gap_explicit = args.diff_pair_gap is not None
    # --track-width IS the diff-pair LEG WIDTH here: when omitted, default to the
    # board's OWN Default net-class diff_pair_width (else routing_defaults), so a
    # bare diff route uses the board's own differential geometry -- parity with the
    # GUI diff tab's per-control override. (May still be overridden by --impedance.)
    if args.track_width is None:
        _v = board_default_netclass_param(args.input_file, 'diff_pair_width')
        args.track_width = _v if _v is not None else defaults.DIFF_PAIR_WIDTH
        print(f"--track-width not given; using "
              f"{'the board Default net-class diff-pair-width' if _v is not None else 'the fallback'} "
              f"{args.track_width}mm.")
    # via_size / via_drill: when omitted, default to the board's OWN Default
    # net-class value (else the routing_defaults constant).
    for _pname, _nckey, _fallback in (('via_size', 'via_diameter', defaults.VIA_SIZE),
                                      ('via_drill', 'via_drill', defaults.VIA_DRILL)):
        if getattr(args, _pname) is None:
            _v = board_default_netclass_param(args.input_file, _nckey)
            setattr(args, _pname, _v if _v is not None else _fallback)
            print(f"--{_pname.replace('_', '-')} not given; using "
                  f"{'the board Default net-class' if _v is not None else 'the fallback'} "
                  f"{getattr(args, _pname)}mm.")
    # --diff-pair-gap: when omitted, default to the board's OWN Default net-class
    # diff_pair_gap (else the routing_defaults constant).
    if args.diff_pair_gap is None:
        _g = board_default_netclass_param(args.input_file, 'diff_pair_gap')
        _gap_fallback = getattr(defaults, 'DIFF_PAIR_GAP', 0.101)
        args.diff_pair_gap = _g if _g is not None else _gap_fallback
        print(f"--diff-pair-gap not given; using "
              f"{'the board Default net-class diff-pair-gap' if _g is not None else 'the fallback'} "
              f"{args.diff_pair_gap}mm.")
    # --clearance given -> pure ceiling on EVERY class (Default included): base =
    # min(Default class, ceiling), non-Default capped at the ceiling. Omitted -> no
    # ceiling: each net routes at its own class (base = board Default class).
    # #530 (decision 2): --clearance sets the Default class for the run; the
    # cap-every-class behaviour (#439) is the explicit --clearance-ceiling.
    _ceiling = getattr(args, 'clearance_ceiling', None)   # None iff omitted
    args._clamp_netclasses = _ceiling is not None
    args._clearance_ceiling = _ceiling
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.input_file)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    _dflt_clr = board_default_netclass_clearance(args.input_file)
    if args.clearance is None:
        args.clearance = _dflt_clr if _dflt_clr is not None else defaults.CLEARANCE
        print(f"--clearance not given; honoring net classes with base = "
              f"{'the board Default net-class' if _dflt_clr is not None else 'the fallback'} "
              f"clearance {args.clearance}mm.")
    else:
        print(f"--clearance {args.clearance}: the Default net class routes at it this run; "
              f"other classes are honoured (pass --clearance-ceiling to cap every class).")
    if _ceiling is not None:
        args.clearance = min(args.clearance, _ceiling)
        print(f"--clearance-ceiling {_ceiling}: every net class is capped at it (#439).")
    # #441: a diff-pair coupling gap below clearance is graded as a clearance
    # violation by KiCad (P<->N are different nets). Raise the gap to the clearance
    # floor now that both are resolved -- BOTH the engine call below and the
    # post-route .kicad_pro writeback then record the same floored gap, so the
    # written diff_pair_gap never drops under the class clearance either. (The
    # engine floors again as a safety net for direct callers.)
    if args.diff_pair_gap is not None and args.clearance and args.diff_pair_gap < args.clearance:
        print(f"Diff-pair gap {args.diff_pair_gap}mm is below clearance "
              f"{args.clearance}mm; raising gap to clearance (#441).")
        args.diff_pair_gap = args.clearance
    # Shared resolver: a declared 0 is UNSET, the same rule the placement half
    # of the loop applies (list_nets.resolve_cli_floor).
    args.hole_to_hole_clearance = resolve_cli_floor(
        args.input_file, 'hole_to_hole', args.hole_to_hole_clearance,
        defaults.HOLE_TO_HOLE_CLEARANCE, '--hole-to-hole-clearance')
    args.board_edge_clearance = resolve_cli_floor(
        args.input_file, 'board_edge_clearance', args.board_edge_clearance,
        defaults.BOARD_EDGE_CLEARANCE, '--board-edge-clearance')
    set_default_fab_tier(*fab_tier_from_args(args))
    __import__('fab_tiers').set_policy_from_args(args, args.input_file)  # #857
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None),
        board_edge_clearance=getattr(args, 'board_edge_clearance', None),
        # The diff-pair P/N gap is copper-to-copper spacing; floor it at the fab
        # copper-clearance min too (parity with the GUI). Impedance may raise the
        # width/gap per layer above this, never below.
        diff_pair_gap=getattr(args, 'diff_pair_gap', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps.
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    # --output is a named alias for the positional output_file; reject giving both differently.
    if args.output is not None:
        if args.output_file is not None and args.output_file != args.output:
            parser.error("specify the output path once: positional output_file OR --output, not both")
        args.output_file = args.output

    # Handle output file: use --overwrite, explicit output, or auto-generate with _routed suffix
    if args.output_file is None:
        if args.overwrite:
            args.output_file = args.input_file
        else:
            # Auto-generate output filename: input.kicad_pcb -> input_routed.kicad_pcb
            base, ext = os.path.splitext(args.input_file)
            args.output_file = base + '_routed' + ext
            print(f"Output file: {args.output_file}")

    # Load PCB to expand wildcards
    print(f"Loading {args.input_file} to expand net patterns...")
    pcb_data = parse_kicad_pcb(args.input_file)

    # Combine positional net_patterns and --nets argument
    all_patterns = list(args.net_patterns) if args.net_patterns else []
    if args.nets:
        all_patterns.extend(args.nets)

    # Accept comma-separated patterns inside one token, e.g.
    # --nets "/DVI_CK_P,/DVI_CK_N" (issue #143): split so each is matched on its
    # own instead of as a single glob containing a comma (which matches nothing).
    all_patterns = [p.strip() for token in all_patterns for p in token.split(',') if p.strip()]

    # Default to "*" (all nets) if no patterns specified
    if not all_patterns:
        all_patterns = ["*"]

    # Warn loudly about any pattern that matches no REAL net, instead of silently
    # routing fewer pairs than asked for (issue #143). expand_net_patterns passes
    # an exact name through even when the board has no such net, so check the
    # expansion against the actual net names.
    real_names = {n.name for n in pcb_data.nets.values() if n.name}
    for pat in all_patterns:
        if pat == "*":
            continue
        if not any(nm in real_names for nm in expand_net_patterns(pcb_data, [pat])):
            print(f"WARNING: net pattern '{pat}' matched no net in this board.")

    # Expand patterns to net names
    net_names = expand_net_patterns(pcb_data, all_patterns)

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    print(f"Routing {len(net_names)} nets as differential pairs: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    # Cross-class clearance map (KiCad max(classA, classB)): explicit --net-clearances
    # JSON overrides; otherwise auto-read the board's non-Default netclasses from the
    # sibling .kicad_pro. All-Default boards -> empty map -> inert (byte-identical).
    _net_clearances_map = None
    if args.net_clearances:
        import json as _jc
        with open(args.net_clearances, encoding="utf-8") as _f:
            _name_to_clr = _jc.load(_f)
        _net_clearances_map = {}
        for _nid, _net in pcb_data.nets.items():
            if _net.name in _name_to_clr:
                _net_clearances_map[_nid] = float(_name_to_clr[_net.name])
        print(f"Loaded per-net clearances for {len(_net_clearances_map)}/{len(pcb_data.nets)} nets "
              f"from {args.net_clearances}")
    else:
        from list_nets import net_clearance_map_by_id
        _net_clearances_map = net_clearance_map_by_id(
            args.input_file, {_nid: _net.name for _nid, _net in pcb_data.nets.items()})
        # #439: when --clearance was GIVEN it is the ceiling -- cap each class at
        # min(class, --clearance) (stock classes are aspirational). When omitted the
        # classes are honored in full.
        if _net_clearances_map and args._clamp_netclasses:
            _net_clearances_map = {nid: min(clr, args._clearance_ceiling)
                                   for nid, clr in _net_clearances_map.items()}
        if _net_clearances_map:
            _classes = sorted({round(v, 4) for v in _net_clearances_map.values()})
            _mode = (f"capped at --clearance {args._clearance_ceiling}"
                     if args._clamp_netclasses
                     else "honored in full (--clearance omitted)")
            print(f"Netclass clearances for {len(_net_clearances_map)} net(s), {_mode} "
                  f"(mm: {_classes}); diff-pair routing respects cross-class max(A,B). "
                  f"Override with --net-clearances.")

    batch_route_diff_pairs(args.input_file, args.output_file, net_names,
                net_clearances=_net_clearances_map,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones,
                layers=args.layers,
                layer_costs=args.layer_costs,
                track_width=args.track_width,
                impedance=args.impedance,
                coplanar_gap=args.coplanar_gap,
                clearance=args.clearance,
                via_size=args.via_size,
                via_drill=args.via_drill,
                grid_step=args.grid_step,
                via_cost=args.via_cost,
                max_iterations=args.max_iterations,
                max_probe_iterations=args.max_probe_iterations,
                heuristic_weight=args.heuristic_weight,
                proximity_heuristic_factor=args.proximity_heuristic_factor,
                turn_cost=args.turn_cost,
                direction_preference_cost=args.direction_preference_cost,
                bus_enabled=args.bus,
                bus_detection_radius=args.bus_detection_radius,
                bus_attraction_radius=args.bus_attraction_radius,
                bus_attraction_bonus=args.bus_attraction_bonus,
                bus_min_nets=args.bus_min_nets,
                keepout_enabled=args.keepout,
                keepout_layer=args.keepout_layer,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                via_proximity_cost=args.via_proximity_cost,
                bga_proximity_radius=args.bga_proximity_radius,
                bga_proximity_cost=args.bga_proximity_cost,
                track_proximity_distance=args.track_proximity_distance,
                track_proximity_cost=args.track_proximity_cost,
                diff_pair_gap=args.diff_pair_gap,
                diff_pair_width_from_class=not _dp_width_explicit,
                diff_pair_gap_from_class=not _dp_gap_explicit,
                diff_pair_centerline_setback=args.diff_pair_centerline_setback,
                min_turning_radius=args.min_turning_radius,
                debug_lines=args.debug_lines,
                verbose=args.verbose,
                polarity_swap_nets=args.polarity_swap_nets,
                max_rip_up_count=args.max_ripup,
                ripup_blocker_select=args.ripup_blocker_select,
                max_setback_angle=args.max_setback_angle,
                enable_layer_switch=not args.no_stub_layer_swap,
                crossing_layer_check=not args.no_crossing_layer_check,
                can_swap_to_top_layer=args.can_swap_to_top_layer,
                swappable_net_patterns=args.swappable_nets,
                crossing_penalty=args.crossing_penalty,
                skip_routing=args.skip_routing,
                routing_clearance_margin=args.routing_clearance_margin,
                hole_to_hole_clearance=args.hole_to_hole_clearance,
                board_edge_clearance=args.board_edge_clearance,
                same_net_pad_clearance=args.same_net_pad_clearance,
                max_turn_angle=args.max_turn_angle,
                gnd_via_enabled=not args.no_gnd_vias,
                vertical_attraction_radius=args.vertical_attraction_radius,
                vertical_attraction_cost=args.vertical_attraction_cost,
                ripped_route_avoidance_radius=args.ripped_route_avoidance_radius,
                ripped_route_avoidance_cost=args.ripped_route_avoidance_cost,
                length_match_groups=args.length_match_groups,
                length_match_tolerance=args.length_match_tolerance,
                meander_amplitude=args.meander_amplitude,
                meander_spacing=args.meander_spacing,
                time_matching=args.time_matching,
                time_match_tolerance=args.time_match_tolerance,
                diff_chamfer_extra=args.diff_chamfer_extra,
                diff_pair_intra_match=args.diff_pair_intra_match,
                ac_couple_match=args.ac_couple_match,
                debug_memory=args.debug_memory,
                mps_reverse_rounds=args.mps_reverse_rounds,
                mps_layer_swap=args.mps_layer_swap,
                mps_segment_intersection=args.mps_segment_intersection,
                keep_input_copper=args.keep_input_copper,
                schematic_dir=args.schematic_dir,
                add_teardrops=args.add_teardrops)

    # Make the output project's DRC design rules consistent with the floors we
    # just routed to (issue #160), mirroring route.py, so a manual DRC in KiCad
    # flags only genuine problems instead of stock-default noise.
    if not args.no_fix_drc_settings and not args.skip_routing \
            and args.output_file and os.path.isfile(args.output_file):
        try:
            import clearance_ledger
            eff_clearance = clearance_ledger.effective(args.clearance)
            if eff_clearance < args.clearance:
                print(f"  Min clearance used: {eff_clearance:.4g} mm "
                      f"(below nominal {args.clearance:.4g}) - grading at this floor")
            from fix_kicad_drc_settings import fix_project_for_output, drc_fix_kwargs
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=eff_clearance, hole_to_hole=args.hole_to_hole_clearance,
                edge_clearance=args.board_edge_clearance, track_width=args.track_width,
                via_diameter=args.via_size, via_drill=args.via_drill,
                diff_pair_gap=args.diff_pair_gap, diff_pair_width=args.track_width,
                **drc_fix_kwargs(args))
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")
        # #521: record this step's protection-worthy nets (routed diff pairs,
        # matched groups) and impedance declarations in the output project so
        # later steps refuse to rip the former and redo the latter at the same
        # widths.
        try:
            from protected_nets import (consume_protection_candidates,
                                        consume_impedance_specs,
                                        persist_protected_nets,
                                        persist_impedance_specs, pro_path_for_board)
            _pro = pro_path_for_board(args.output_file)
            persist_protected_nets(_pro, consume_protection_candidates())
            persist_impedance_specs(_pro, consume_impedance_specs())
            # #581: an explicit active flag on this step is recorded so later
            # chain steps keep their vias off same-net pads too.
            if getattr(args, 'same_net_pad_clearance', None) is not None \
                    and args.same_net_pad_clearance > 0:
                from protected_nets import persist_same_net_pad_clearance
                persist_same_net_pad_clearance(_pro, args.same_net_pad_clearance)
        except Exception as e:
            print(f"  (skipped protected-nets record: {e})")

    # The other false success -- the patterns
    # matched no pair at all, which used to print `Error:` and exit 0 because
    # main called the router as a bare statement. Measured: a shell rewrote
    # every `/IO_Banks/Z*` argument into a Windows path and 14 patterns matched
    # nothing.
    if _NO_PAIRS_MATCHED:
        print("route_diff: the --nets patterns matched no differential pair, "
              "so nothing was routed. Exiting non-zero so a chained caller "
              "does not read this as success.", file=sys.stderr)
        sys.exit(2)

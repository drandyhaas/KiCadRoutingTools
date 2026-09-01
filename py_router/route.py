"""
Batch PCB Router using Rust-accelerated A* - Routes single-ended nets sequentially.

For differential pair routing, use route_diff.py instead.

Usage:
    python py_router/route.py input.kicad_pcb output.kicad_pcb --nets "Net-(U2A-*)"

Requires the Rust router module. Build it with:
    python3 build_router.py
(never bare `cargo build` -- build_router.py also places the library and
verifies the version; see CLAUDE.md).
"""
from __future__ import annotations

import env_knobs
import sys
import os
import copy

# Run startup checks before other imports
from startup_checks import exit_on_error_if_main
# Stays at module scope, ABOVE the heavy imports, so a missing dep is
# reported before numpy/grid_router blow up with something cryptic. But it
# raises instead of exiting when this module is IMPORTED rather than run,
# so pytest can still collect a suite on a checkout with no built router
# (#457 item 3).
exit_on_error_if_main(__name__)

import time
import json
from typing import List, Optional, Tuple, Dict, Set

from kicad_parser import parse_kicad_pcb, PCBData, Pad
from kicad_writer import (
    generate_segment_sexpr, generate_via_sexpr, generate_gr_line_sexpr, generate_gr_text_sexpr,
    swap_segment_nets_at_positions, swap_via_nets_at_positions, swap_pad_nets_in_content,
    modify_segment_layers
)
from output_writer import write_routed_output
from cleanup_pipeline import (run_post_route_cleanup, verify_board_file_parity,
                              verify_written_file_parity)
from schematic_updater import apply_swaps_to_schematics

# Import from refactored modules
from routing_config import GridRouteConfig, GridCoord
from connectivity import (
    get_stub_endpoints, find_stub_free_ends, find_connected_groups,
    is_edge_stub, get_net_endpoints, find_connected_segment_positions
)
from net_queries import (
    get_all_unrouted_net_ids, get_chip_pad_positions,
    compute_mps_net_ordering, find_pad_nearest_to_position,
    expand_net_patterns, find_single_ended_nets, identify_power_nets,
    filter_routable_nets, nets_for_components, suggest_component_refs
)
from impedance import calculate_layer_widths_for_impedance, print_impedance_routing_plan
from obstacle_map import (
    build_base_obstacle_map, add_net_stubs_as_obstacles, add_net_pads_as_obstacles,
    add_net_vias_as_obstacles, add_same_net_via_clearance,
    get_net_bounds,
    draw_exclusion_zones_debug, add_vias_list_as_obstacles, add_segments_list_as_obstacles
)
from obstacle_costs import (
    add_stub_proximity_costs, compute_track_proximity_for_net,
    merge_track_proximity_costs, add_cross_layer_tracks
)
from obstacle_cache import (
    precompute_all_net_obstacles, build_working_obstacle_map, update_net_obstacles_after_routing
)
from single_ended_routing import (route_net_with_obstacles,
                                   route_multipoint_taps, build_corridor_waypoints)
from blocking_analysis import analyze_frontier_blocking, print_blocking_analysis, filter_rippable_blockers
from rip_up_reroute import rip_up_net, restore_net
from layer_swap_optimization import apply_single_ended_layer_swaps
from routing_context import (
    build_single_ended_obstacles,
    record_single_ended_success,
    restore_ripped_net
)
from routing_state import RoutingState, create_routing_state, print_failed_net_histories
from memory_debug import (
    get_process_memory_mb, format_memory_stats,
    estimate_net_obstacles_cache_mb, estimate_track_proximity_cache_mb,
    estimate_routed_paths_mb, format_obstacle_map_stats
)
from single_ended_loop import route_single_ended_nets
from reroute_loop import run_reroute_loop
from phase3_routing import run_phase3_tap_routing
from net_ordering import order_nets_mps, order_nets_inside_out
from routing_common import (
    setup_bga_exclusion_zones, resolve_net_ids, filter_already_routed,
    run_length_matching, sync_pcb_data_segments,
    get_common_config_kwargs, warn_targets_outside_board
)
import routing_defaults as defaults
import re
from terminal_colors import RED, RESET
from routing_constants import DEFAULT_4_LAYER_STACK, POWER_NET_EXCLUSION_PATTERNS

# Import Rust router (startup_checks ensures it's available and up-to-date)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'rust_router')))
import rust_alloc  # noqa: E402,F401  # issue #419: set MIMALLOC_PURGE_DELAY before grid_router loads
from grid_router import GridObstacleMap, GridRouter


def _compute_stale_input_copper(orig_by_net, scope_ids, final_copper,
                                emitted_copper, full_sig, pos_sig,
                                final_ids=None):
    """Shared core for #284 stale-copper stripping (vias and segments).

    The output writer copies the input file verbatim, then APPENDS the routing
    results' copper (each result's new segments/vias plus stub layer-swap
    copper). So an original item of a ripped/re-routed net can ship TWICE, or
    next to a replacement, leaving redundant same-net copper stacked on itself.

    Strip an in-scope net's original item when EITHER:

    - it is no longer on the final (frozen committed) board -- the net was
      rerouted away or ripped-and-not-restored. With ``final_ids`` (object
      identity at freeze time) the test is exact; rip/restore preserves object
      identity (restore_net re-adds the saved objects), so a kept original
      always passes. The signature fallback (when ``final_ids`` is None) has a
      TWIN-SHIELDING hole: an original whose net was ripped and re-routed onto
      the byte-identical span is "matched" by the ROUTED twin's signature and
      kept -- and when a later cleanup pass (the cycle prune) removes that
      routed twin, the original ships as copper the board model no longer has
      (neo6502 +3.3V slivers, found by KICAD_BOARD_LEDGER).
    - the writer will RE-EMIT an item for the SAME net at the SAME POSITION: the
      re-emitted item is authoritative, so the verbatim original is the redundant
      copy (this covers a byte-identical replacement, which a full-signature match
      alone treats as "kept").

    ``full_sig`` distinguishes a superseded item from a kept one; ``pos_sig`` is
    the coarser locus a replacement lands on (via drill hole / segment span).
    """
    final = {}
    if final_ids is None:
        for c in final_copper:
            final.setdefault(c.net_id, set()).add(full_sig(c))
    emit = {}
    for c in emitted_copper:
        emit.setdefault(c.net_id, set()).add(pos_sig(c))
    stale = []
    for nid in scope_ids:
        nfinal = final.get(nid, ())
        nemit = emit.get(nid, ())
        for c in orig_by_net.get(nid, []):
            on_board = (id(c) in final_ids) if final_ids is not None \
                else (full_sig(c) in nfinal)
            if not on_board or pos_sig(c) in nemit:
                stale.append(c)
    return stale


def _via_full_sig(v):
    return (round(v.x, 3), round(v.y, 3), round(v.size, 3), round(v.drill, 3))


def _via_pos_sig(v):
    # A via's drill hole is a point: two same-net vias at one (x, y) overlap
    # regardless of size, and the drill hole-to-hole check is net-independent.
    return (round(v.x, 3), round(v.y, 3))


def _seg_span_sig(s):
    a = (round(s.start_x, 3), round(s.start_y, 3))
    b = (round(s.end_x, 3), round(s.end_y, 3))
    return (min(a, b), max(a, b), s.layer)


def compute_stale_input_vias(orig_via_by_net, scope_ids, final_vias, emitted_vias,
                             final_ids=None):
    """Original input vias to strip from the verbatim output copy (issue #284).

    A ripped/re-routed net's reroute often lands a via-in-pad via at the EXACT
    position of the net's original via; without stripping the original, the
    output ships two same-net vias in one hole -- and the drill hole-to-hole
    check is net-independent, so this violates DRC whether the two are the same
    size or not. See ``_compute_stale_input_copper``.
    """
    return _compute_stale_input_copper(
        orig_via_by_net, scope_ids, final_vias, emitted_vias,
        full_sig=_via_full_sig, pos_sig=_via_pos_sig, final_ids=final_ids)


def compute_stale_input_segments(orig_seg_by_net, scope_ids, final_segments,
                                 emitted_segments, final_ids=None):
    """Original input segments to strip from the verbatim output copy (issue
    #284, segment twin of ``compute_stale_input_vias``).

    Unlike vias, two exactly-overlapping same-net segments are DRC-benign (KiCad
    permits same-net copper overlap and there is no net-independent segment
    check), so this is a cleanliness fix, not a DRC one -- it keeps a ripped/
    re-routed net from shipping a verbatim original segment next to a byte-
    identical re-emitted copy. A segment is keyed by span+layer for both roles
    (the reroute either reproduces the span exactly or routes a different one).
    """
    return _compute_stale_input_copper(
        orig_seg_by_net, scope_ids, final_segments, emitted_segments,
        full_sig=_seg_span_sig, pos_sig=_seg_span_sig, final_ids=final_ids)


def _write_passthrough_output(input_file: str, output_file: str) -> None:
    """Write the output as an unchanged copy of the input (issue #86).

    When routing produced nothing - no valid nets, or everything already
    connected - the result is still representable: the board is unchanged.
    Pipelines that chain output->input then keep working instead of dying on
    a missing file. Skipped when output is the input (in-place).
    """
    from pcb_io_utils import passthrough_copy
    if passthrough_copy(input_file, output_file):
        print(f"Wrote unchanged copy to {output_file} (nothing to route)")


def _dump_engine_config(engine, cfg):
    """Config-parity probe for the plane engines (#362), mirroring batch_route's
    dump. Only active in APPEND/CONTINUE mode (KICAD_DUMP_BATCH_KWARGS +
    KICAD_DUMP_BATCH_KWARGS_CONTINUE=1): writes one JSONL line per engine call
    and never alters routing, so a whole GUI plan run is captured in one pass."""
    if not (env_knobs.DUMP_BATCH_KWARGS and env_knobs.DUMP_BATCH_KWARGS_CONTINUE):
        return
    import json as _json
    d = {'_engine': engine}
    for k, v in cfg.items():
        # Skip the board payload and non-config callables. all_layers/plane_layers
        # ARE kept: layer ORDER is a live GUI/CLI divergence class (the pcbnew
        # layer-ID vs stackup-order bug), so it must be visible in the dump.
        # progress_callback is skipped by NAME, not just callable(): the CLI
        # passes None (not callable, would dump as null) while the GUI passes
        # a function (skipped) -- a phantom key diff in the parity harness.
        if k in ('input_file', 'output_file', 'pcb_data',
                 'progress_callback', 'cancel_check') or callable(v):
            continue
        try:
            _json.dumps(v)
            d[k] = v
        except (TypeError, ValueError):
            d[k] = repr(v)
    try:
        with open(env_knobs.DUMP_BATCH_KWARGS, 'a') as _f:
            _f.write(_json.dumps(d, sort_keys=True) + '\n')
    except Exception:
        pass


def _empty_results_data() -> dict:
    """The return_results contract with every field empty (#382 E5).

    batch_route's early-return paths (nothing to route, KWARGS-dump exit) used
    to emit ad-hoc subsets of these keys, so a GUI caller that iterated a key
    the full path always provides would KeyError on an early exit. This is the
    single source for the empty shape; it must carry EXACTLY the keys the full
    path builds (see the `if return_results:` block), all as empty lists.
    """
    return {
        'results': [],
        'all_swap_vias': [],
        'all_swap_segments': [],
        'pad_swaps': [],
        'single_ended_target_swap_info': [],
        'all_segment_modifications': [],
        'exclusion_zone_lines': [],
        'boundary_debug_labels': [],
        'segments_to_remove': [],
        'vias_to_remove': [],
        'blockers': [],
        'boxed_in': [],
        'pad_pairs_open': [],
    }


# --json-out collects every JSON_SUMMARY this process emits -- the first pass
# and, when it fires, the reconciliation sub-run's -- so the file carries ONE
# merged tally instead of whichever emission a reader happened to scrape.
_SUMMARY_SINK: List[dict] = []
_RECONCILE_RAISED = [False]

# #562 finalize re-entry guard. batch_route's plane finalize calls
# repair_planes, whose own rip-casualty / pad-repair sub-runs call batch_route
# again -- if such a sub-run finalizes, the two recurse without bound
# (measured 57 levels and ~7 GB on one corpus board). The sub-runs now pass
# final_reconcile=False, and this flag is the structural backstop so a future
# caller that forgets cannot reintroduce the recursion.
_PLANE_FINALIZE_DEPTH = 0


def _finalize_depth(delta: int) -> None:
    global _PLANE_FINALIZE_DEPTH
    _PLANE_FINALIZE_DEPTH = max(0, _PLANE_FINALIZE_DEPTH + delta)


def _plane_finalize_active() -> bool:
    return _PLANE_FINALIZE_DEPTH > 0


def _emit_summary_min(gate_report: Optional[dict] = None,
                      status: Optional[str] = None) -> None:
    """Print the ONE compact verdict line for an outermost run.

    Shared by the normal end-of-run path and the two "nothing to route" early
    returns. The contract a consumer relies on is "exactly one
    JSON_SUMMARY_MIN per outermost run", and a step that legitimately did
    nothing -- "All nets are already fully connected", the routine state a
    replay or a retry step lands in -- must still satisfy it. A missing line
    there is indistinguishable from a crash, which is the one reading that
    would make an agent do the wrong thing.

    `status` names WHY the tally is empty on those paths; it is absent on an
    ordinary run.
    """
    try:
        from route_summary import merge_summaries as _ms, summary_min
        _m = _ms(list(_SUMMARY_SINK), _RECONCILE_RAISED[0])
        if _m is None and status is not None:
            # The early returns fire before any summary is built, so there is
            # nothing in the sink to merge: synthesize the empty tally rather
            # than print nothing.
            _m = {'successful': 0, 'failed': 0}
        if not _m:
            return
        _min = summary_min(_m)
        if status is not None:
            _min['status'] = status
        if gate_report and gate_report.get('verdict') == 'reject':
            _min['improvement_gate'] = 'reverted'
        print("JSON_SUMMARY_MIN: " + json.dumps(_min, sort_keys=True))
    except Exception as _e:                                     # noqa: BLE001
        print(f"  WARNING: could not emit JSON_SUMMARY_MIN: {_e}")


def _late_orphan_sweep659(pcb_data, output_file, return_results, results_data,
                          protect_unfinished, keep_input_copper, skip_routing):
    """Sweep pad-less copper islands off the FINAL board (#659).

    run_post_route_cleanup is the only whole-scope cleanup and it runs long
    before the plane finalize and the final reconciliation, both of which lay
    copper WITH RIP AUTHORITY. Their rip-restore debris therefore lands after
    the last pass that could remove it, and the only cleanups that DO run
    later are the reconcile sub-runs', scoped to the reconcile subset. Same
    staleness shape as #650's stale oracle audit.

    Measured: daisho's /ddr2/DM6 carried a 7.35mm pad-less fragment -- born at
    step 1, a BGA fanout, which runs no cleanup pipeline at all -- through
    eleven chain steps and a full `--nets '*'` route; its step-9 log printed no
    "Orphan islands" line at all. spartan6_4layer step 6 shipped 10 bare vias a
    failed reroute left behind.

    Deletion is safe by construction: an island reaching no pad and no zone of
    its net conducts nothing, so removing it cannot change any pad's
    connectivity. --keep-input-copper still makes input copper read-only, and
    UNFINISHED nets are scoped out exactly as the main pipeline scopes them
    (#473: a fragment graded "orphan" on a net this run could not finish is the
    copper the NEXT chain step welds to).

    Called from BOTH the normal end of a run and the "nothing to route" early
    return -- the fragment gate now diverts pad-less-only nets to this sweep,
    and a step whose whole scope is diverted takes that early path, where a
    sweep that only ran at the normal end would silently never happen.

    Cost: ~1s on a 12k-segment board. KICAD_LATE_ORPHAN_SWEEP=0 disables.
    """
    if (skip_routing or keep_input_copper
            or os.environ.get('KICAD_LATE_ORPHAN_SWEEP', '1') == '0'):
        return
    try:
        from pcb_modification import remove_orphan_islands as _roi659

        def _scope(_p):
            # Derived from the board being SWEPT, not from pcb_data: the CLI
            # branch re-parses the written file, and a scope built from a
            # different board object is only accidentally right.
            if not protect_unfinished:
                return None
            return ({s.net_id for s in _p.segments}
                    | {v.net_id for v in _p.vias}) - set(protect_unfinished)

        if return_results:
            # GUI: the copper lives in the write-lists + pcb_data. The pass
            # drops this-run copper from `results` in place and hands back the
            # INPUT copper for the applier's removal lists (the same channels
            # the reconciliation sub-run's strips ride).
            _n, _s, _strip, _vstrip, _nv = _roi659(
                (results_data or {}).get('results') or [], pcb_data,
                _scope(pcb_data))
            if _n and results_data is not None:
                if _strip:
                    results_data.setdefault('segments_to_remove', []).extend(_strip)
                if _vstrip:
                    results_data.setdefault('vias_to_remove', []).extend(_vstrip)
        elif output_file and os.path.exists(output_file):
            # CLI: the finalize and the reconcile wrote through to the file, so
            # the board on disk -- not pcb_data -- is the final state. Re-parse
            # it, and strip what the sweep condemns from the file text (the
            # oracle's own debris pass works the same way).
            from kicad_parser import parse_kicad_pcb as _pk, is_kicad_10 as _k10
            from kicad_writer import (remove_segments_from_content as _rsc,
                                      remove_vias_from_content as _rvc)
            _pcb = _pk(output_file)
            _n, _s, _strip, _vstrip, _nv = _roi659([], _pcb, _scope(_pcb))
            if _n and (_strip or _vstrip):
                with open(output_file, 'r', encoding='utf-8') as _f:
                    _c = _f.read()
                _map = ({nid: n.name for nid, n in _pcb.nets.items()}
                        if _k10(_c) else None)
                if _strip:
                    _c, _ = _rsc(_c, _strip, net_id_to_name=_map)
                if _vstrip:
                    _c, _ = _rvc(_c, _vstrip, net_id_to_name=_map)
                with open(output_file, 'w', encoding='utf-8') as _f:
                    _f.write(_c)
        else:
            return
        if _n:
            print(f"  Late orphan sweep (#659): removed {_n} pad-less copper "
                  f"island(s) the finalize/reconcile left behind "
                  f"({_s} segment(s), {_nv} via(s))")
    except Exception as _e:
        print(f"  (late orphan sweep skipped: {_e})")


def batch_route(input_file: str, output_file: str, net_names: List[str],
                layers: List[str] = None,
                bga_exclusion_zones: Optional[List[Tuple[float, float, float, float]]] = None,
                direction_order: str = None,
                ordering_strategy: str = "inside_out",
                disable_bga_zones: Optional[List[str]] = None,
                track_width: float = defaults.TRACK_WIDTH,
                track_width_from_class: bool = False,
                impedance: Optional[float] = None,
                coplanar_gap: float = 0.0,
                coplanar_nets: Optional[List[str]] = None,
                power_nets: Optional[List[str]] = None,
                power_nets_widths: Optional[List[float]] = None,
                power_tap_neckdown: bool = True,
                neckdown_length: float = 2.5,
                neckdown_taper_length: float = 0.5,
                json_out: Optional[str] = None,
                clearance: float = defaults.CLEARANCE,
                via_size: float = defaults.VIA_SIZE,
                via_drill: float = defaults.VIA_DRILL,
                grid_step: float = 0.1,
                via_cost: int = 50,
                max_iterations: int = 200000,
                max_probe_iterations: int = 5000,
                heuristic_weight: float = defaults.HEURISTIC_WEIGHT,
                turn_cost: int = 1000,
                direction_preference_cost: int = defaults.DIRECTION_PREFERENCE_COST,
                bus_enabled: bool = False,
                bus_detection_radius: float = 5.0,
                bus_attraction_radius: float = 5.0,
                bus_attraction_bonus: int = 5000,
                bus_min_nets: int = 2,
                guide_corridor_enabled: bool = False,
                guide_corridor_layer: str = "User.1",
                guide_corridor_spacing: float = 0.0,
                keepout_enabled: bool = False,
                keepout_layer: str = "User.2",
                proximity_heuristic_factor: float = defaults.PROXIMITY_HEURISTIC_FACTOR,
                stub_proximity_radius: float = 2.0,
                stub_proximity_cost: float = 0.2,
                via_proximity_cost: float = 10.0,
                bga_proximity_radius: float = 7.0,
                bga_proximity_cost: float = 0.2,
                track_proximity_distance: float = 2.0,
                track_proximity_cost: float = defaults.TRACK_PROXIMITY_COST,
                debug_lines: bool = False,
                verbose: bool = False,
                max_rip_up_count: int = defaults.MAX_RIPUP,
                ripup_abandon_metric: str = 'stranded',
                ripup_blocker_select: str = 'count',
                enable_layer_switch: bool = True,
                crossing_layer_check: bool = True,
                can_swap_to_top_layer: bool = False,
                swappable_net_patterns: Optional[List[str]] = None,
                crossing_penalty: float = 1000.0,
                mps_unroll: bool = True,
                skip_routing: bool = False,
                routing_clearance_margin: float = 1.0,
                hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
                board_edge_clearance: float = 0.0,
                vertical_attraction_radius: float = 1.0,
                vertical_attraction_cost: float = 0.0,
                ripped_route_avoidance_radius: float = 1.0,
                ripped_route_avoidance_cost: float = 0.1,
                length_match_groups: Optional[List[List[str]]] = None,
                length_match_tolerance: float = 0.1,
                meander_amplitude: float = 1.0,
                meander_spacing: float = 2.0,
                time_matching: bool = False,
                time_match_tolerance: float = 1.0,
                debug_memory: bool = False,
                mps_reverse_rounds: bool = False,
                mps_layer_swap: bool = False,
                mps_segment_intersection: bool = False,
                minimal_obstacle_cache: bool = False,
                schematic_dir: Optional[str] = None,
                layer_costs: Optional[List[float]] = None,
                # #498: {layer: mm} per-layer clearance. None (both fronts) ->
                # auto-read the sibling .kicad_dru; explicit dict (tests) wins.
                layer_clearances: Optional[Dict[str, float]] = None,
                # {net_id: mm} track-to-track clearance (#735; effective
                # per-obstacle map). None (both fronts) -> auto-read the
                # sibling .kicad_dru track rules; explicit dict (tests) wins.
                track_clearances: Optional[Dict[int, float]] = None,
                final_reconcile: bool = True,
                rip_preexisting: Optional[bool] = None,
                add_teardrops: bool = False,
                collect_stats: bool = False,
                cancel_check=None,
                progress_callback=None,
                return_results: bool = False,
                pcb_data=None,
                net_clearances: dict = None,
                keep_input_copper: bool = False,
                smoothing: bool = True,
                rip_existing_nets: Optional[List[str]] = None,
                # Net name patterns whose matches are protected for THIS run
                # (reason 'user') and persisted to the output .kicad_pro like
                # a routed diff pair's record: no rip glob, --rip-blocker-nets
                # pick, in-run phase-3 ladder or seam re-ask may touch them.
                # Override = naming the net EXACTLY (no glob), same as every
                # non-'locked' reason. This is the missing guard for
                # single-ended constrained nets (a committed QSPI wrap, a
                # flat USB pair) that no match group ever writes (#521 covers
                # pairs and matched groups only; test-board run 5 journal
                # [10]: "the only guard is discipline").
                force_reroute: bool = False,
                # The RAW --nets patterns as the operator typed them, BEFORE
                # main()'s expand_net_patterns. #521's protection override is
                # "named EXACTLY, no glob" -- but expansion turns '/LED_*' into
                # exact names, so the engine can't tell a typed name from a
                # glob hit without this. None (the GUI, tests) = net_names ARE
                # the operator's literal selection (per-net checkboxes).
                net_name_patterns: Optional[List[str]] = None,
                # #540 item 2: corridor ghosts of nets ripped by an OUTER pass
                # (the plane repair) and still awaiting reconnect --
                # {net_id: {'new_segments': [...], 'new_vias': [...]}}. Seeded
                # into the ripped-route avoidance dicts so every search in
                # this batch prices the vacated corridors exactly like an
                # internal rip's; keys must not be nets routed by this batch.
                external_ripped_ghosts: Optional[Dict[int, dict]] = None,
                # #562 GUI parity: zero-arg callable returning a path to a
                # freshly saved copy of the caller's LIVE board (the GUI's
                # pcbnew.SaveBoard). Only the GUI can produce one, and the
                # plane finalize's ORACLE leg needs a real file because
                # kicad-cli computes the exact fill. With it the GUI runs the
                # oracle IN-RUN at the CLI's own sequence point -- so oracle
                # copper lands before the final reconciliation and its
                # unroutable links feed custody, exactly as in file mode.
                # Without it (tests, headless callers) the leg is skipped.
                # NOTE: it must save the LIVE board, not re-read input_file --
                # the GUI's input_file is the ORIGINAL on disk and does not
                # carry copper earlier chain steps applied in-session.
                stage_board_fn=None,
                # #572: exact-fill links the plane-finalize oracle left
                # "unroutable without rip authority", forwarded by the outer
                # run into its final-reconciliation sub-run --
                # remaining_links-shaped entries [net_name,
                # [ax, ay, layer, kind], [bx, by, layer, kind]]. Nets named
                # here bypass the model-credit already-routed skip and route
                # their EXACT links as endpoint overrides with the full rip
                # ladder behind them (fix direction 2). INTERNAL: set only
                # by the finalize custody handoff; deliberately no CLI flag.
                oracle_links: Optional[List] = None,
                # #581: edge-to-edge clearance between every via this run
                # places and same-net SMD pads. None (default) -> auto-read
                # the persisted .kicad_pro record; > 0 activates (and is
                # persisted for later chain steps); 0 / -1 explicitly OFF.
                same_net_pad_clearance: Optional[float] = None) -> Tuple[int, int, float]:
    """
    Route single-ended nets using the Rust router.

    For differential pair routing, use route_diff.py instead.

    Every run ends with the in-run PLANE FINALIZE (#562, gate
    KICAD_PLANE_FINALIZE=1 default ON): for zone nets in this run's net
    scope it runs the repair engine (taps + region joins; rip authority
    gated by KICAD_FINALIZE_RIP=1, default ON), the shared
    cleanup pipeline, and the kicad-cli oracle verify, then folds stubborn
    oracle links into the final reconciliation as custody. The standalone
    repair step no longer exists in the chain; a pour is only welded by a
    route step whose --nets cover the plane nets.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to route
        layers: List of copper layers to route on (must be specified - cannot auto-detect
                which layers are ground planes vs signal layers)
        bga_exclusion_zones: Optional list of BGA exclusion zones (auto-detected if None)
        direction_order: Direction search order - "forward" or "backward"
                        (None = use GridRouteConfig default)
        ordering_strategy: Net ordering strategy:
            - "mps": Use Maximum Planar Subset algorithm to minimize crossing conflicts (default)
            - "inside_out": Sort BGA nets by distance from BGA center
            - "original": Keep nets in original order
        track_width: Track width in mm (default: defaults.TRACK_WIDTH, 0.3)
        track_width_from_class: True = the width was NOT explicitly set by the
            operator (--track-width omitted / GUI override unchecked). Without
            impedance, each net then routes at its OWN netclass width (#435);
            with impedance, solved widths floor at the fab tier instead of
            track_width (#610).
        clearance: Clearance between tracks in mm (default: defaults.CLEARANCE, 0.25)
        via_size: Via outer diameter in mm (default: defaults.VIA_SIZE, 0.5)
        via_drill: Via drill size in mm (default: defaults.VIA_DRILL, 0.3)
        grid_step: Grid resolution in mm (default: 0.1)
        via_cost: Penalty for placing a via in 0.1mm grid steps (default: 50 = 5mm; mm-equivalent at any grid_step)
        max_iterations: Max A* iterations before giving up (default: 200000)
        heuristic_weight: A* heuristic weight, higher=faster but less optimal (default: routing_defaults.HEURISTIC_WEIGHT)
        stub_proximity_radius: Radius around stubs to penalize in mm (default: 2.0)
        stub_proximity_cost: Cost penalty near stubs in mm equivalent (default: 0.2)
        bga_proximity_radius: Radius around BGA edges to penalize in mm (default: 7.0)
        bga_proximity_cost: Cost penalty near BGA edges in mm equivalent (default: 0.2)
        debug_lines: Output debug geometry on User.3 (connectors), User.4
                     (stub dirs), User.8 (simplified), User.9 (raw A*)
        minimal_obstacle_cache: If True, only build obstacle cache for nets being routed
                               (faster when re-routing a small number of nets)
        cancel_check: Optional callable returning True if routing should be cancelled
        progress_callback: Optional callable(current, total, net_name) for progress updates
        return_results: If True, return results data instead of writing to file
        stage_board_fn: Optional zero-arg callable returning a path to a staged
            copy of the CURRENT board (the GUI passes a live-board saver). The
            in-run plane finalize's kicad-cli oracle runs on this file instead of
            input_file; None = CLI behavior (post-write oracle on the output).

    Returns:
        If return_results=False: (successful_count, failed_count, total_time)
        If return_results=True: (successful_count, failed_count, total_time, results_data)
    """
    # Snapshot of THIS call's parameters, taken before any body code runs:
    # the end-of-run reconciliation self-invocation forwards every parameter
    # verbatim (only overriding the self-referential ones) so a rescue pass
    # can never route with different rules than the run it is rescuing --
    # forwarding a hand-picked subset silently dropped board_edge_clearance,
    # impedance, net_clearances, ordering and more (review finding).
    _reconcile_kwargs = dict(locals())
    # net_name_patterns must not forward either: the reconcile sub-run scopes
    # by exact retried-net names, which are then the correct override source.
    # json_out joins them: the sub-run must not write the parent's file. The
    # OUTERMOST call owns it, and writes the MERGED tally after reconciliation.
    for _k in ('input_file', 'output_file', 'net_names', 'pcb_data',
               'net_name_patterns', 'json_out',
               # #572: links describe ONE board state; only the finalize
               # custody handoff may set them on a sub-run, never blanket
               # forwarding.
               'oracle_links'):
        _reconcile_kwargs.pop(_k, None)
    # #572 lap-authority channel: cleared at ENTRY so an early return can
    # never leave a previous invocation's hints for the caller to harvest.
    batch_route._forced_link_hints = {}
    batch_route._forced_link_landed = []
    # Per-run reset of the failure-hint rationale latch: each run reprints the
    # full explanations once, rather than inheriting "already said that" from a
    # previous board in the same (GUI) process.
    from routing_diagnostics import reset_hint_condenser
    reset_hint_condenser()
    # Every OUTERMOST entry starts the summary sink clean, not only a
    # --json-out one: the GUI calls batch_route once per click in ONE
    # process with no json_out, and a sink that survived the previous
    # click would merge the next run's JSON_SUMMARY_MIN onto the previous
    # run's summaries (effort keys summed, pad-pair attribution from the
    # old run) and stamp its summaries 'reconciliation-subset'. Nested
    # sub-runs pass final_reconcile=False and so keep the live sink.
    if json_out or final_reconcile:
        _SUMMARY_SINK.clear()
        _RECONCILE_RAISED[0] = False
    if not _SUMMARY_SINK:
        # First (outermost) entry of this process's run: start the
        # protected-net refusal record clean. The reconciliation sub-run
        # re-enters here with the sink non-empty and must NOT reset it --
        # its own refusals are part of the same run's report.
        try:
            from protected_nets import clear_skipped
            clear_skipped()
        except Exception:
            pass
    if env_knobs.DUMP_BATCH_KWARGS:
        # Parameter-parity probe: dump THIS call's full parameter set so the
        # CLI front (argparse->main) and the GUI front (plan setters->tab
        # config->call site) can be diffed key by key on identical inputs.
        # Default: overwrite the file and RETURN without routing (single-call
        # A/B). CONTINUE mode (KICAD_DUMP_BATCH_KWARGS_CONTINUE=1): APPEND one
        # JSONL line per call and keep routing, so a whole multi-step GUI plan
        # run can be captured in one pass without breaking the chain (#362).
        import json as _json
        _dump = {}
        for _k, _v in sorted(_reconcile_kwargs.items()):
            if callable(_v) or _k in ('cancel_check',
                                      'progress_callback'):
                continue
            try:
                _json.dumps(_v)
                _dump[_k] = _v
            except (TypeError, ValueError):
                _dump[_k] = repr(_v)
        _dump['net_names'] = net_names
        if env_knobs.DUMP_BATCH_KWARGS_CONTINUE:
            with open(env_knobs.DUMP_BATCH_KWARGS, 'a') as _f:
                _f.write(_json.dumps(_dump, sort_keys=True) + '\n')
            # fall through -- route normally
        else:
            with open(env_knobs.DUMP_BATCH_KWARGS, 'w') as _f:
                _json.dump(_dump, _f, indent=1, sort_keys=True)
            if return_results:
                return 0, 0, 0.0, _empty_results_data()
            return 0, 0, 0.0

    # Board-setup copper-to-edge rule (#338): KiCad enforces the sibling
    # .kicad_pro's min_copper_edge_clearance, so route to at least it. Done in
    # the ENGINE (not main()) so the GUI and manifest/plan replays inherit it;
    # a missing project reads 0.0 (no-op) and an explicit larger
    # --board-edge-clearance still wins (max).
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
        # Carry the RESOLVED value into the end-of-run reconciliation kwargs:
        # the snapshot above was taken before this resolution, and the
        # reconciliation self-invocation reads the OUTPUT file, whose sibling
        # .kicad_pro does not exist yet (main() writes it after batch_route
        # returns) -- so the sub-run re-resolved 0.0 and stamped its board-edge
        # band at the track-clearance fallback (ottercast_audio BT_PCM_DIN/
        # BT_PCM_SYNC: 16 board-edge violations laid by the reconciliation's
        # phase-1/phase-3 routes inside the 0.5mm project edge band).
        _reconcile_kwargs['board_edge_clearance'] = board_edge_clearance

    # Track memory if debug_memory enabled
    mem_start = get_process_memory_mb() if debug_memory else 0.0
    if debug_memory:
        print(format_memory_stats("Initial memory", mem_start))

    if pcb_data is None:
        print(f"Loading {input_file}...")
        pcb_data = parse_kicad_pcb(input_file, guide_layer=guide_corridor_layer,
                                   keepout_layer=keepout_layer)
    else:
        print("Using provided PCB data...")

    # KICAD_DUP_TRAP=1: report the call site that re-appends the SAME copper
    # object to pcb_data. Inert otherwise. Armed here so it covers the whole
    # run, including the cleanup pipeline and the in-run plane finalize.
    from dup_trap import install as _install_dup_trap
    _install_dup_trap(pcb_data)

    # Canonicalise the STARTING copper order, BEFORE any decision reads it.
    # The GUI adds tracks to a live pcbnew board and the CLI writer emits them
    # from its own lists, so the two fronts hand the engine identical copper in
    # different ORDER (measured on eth_tap: the same 1159-segment set in three
    # different orders across two GUI chains and the CLI). List position then
    # leaks into decisions -- representative endpoints, connected-group order,
    # stub free ends, MST terminal labelling, unit distances -- each of which
    # was fixed individually before the next one surfaced. Doing it once, here,
    # closes the class.
    #
    # Placement matters: this MUST precede apply_single_ended_layer_swaps and
    # the MPS ordering (both far above the routing state setup). An earlier
    # attempt sat next to create_routing_state and had NO effect, because the
    # swap phase had already consumed the un-canonical order.
    from kicad_parser import canonicalize_pcb_data_order
    canonicalize_pcb_data_order(pcb_data)

    # Cross-class clearance: when no map was passed (net_clearances is None -- e.g.
    # the plane routers reroute ripped nets by calling batch_route directly), AUTO-
    # READ the board's non-Default netclasses from the sibling .kicad_pro. #439:
    # cap each class at the routing `clearance` (min) -- stock classes are often
    # aspirational, so the CLI ceiling applies here too. A caller that already
    # resolved the map (route.py main, the GUI) passes a dict (possibly empty) so
    # this does not re-read.
    if net_clearances is None and input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_clearance_map_by_id
            net_clearances = net_clearance_map_by_id(
                input_file, {nid: n.name for nid, n in pcb_data.nets.items()})
            if net_clearances:
                net_clearances = {nid: min(clr, clearance)
                                  for nid, clr in net_clearances.items()}
                print(f"Auto-read netclass clearances for {len(net_clearances)} net(s), "
                      f"capped at clearance {clearance}mm (#439).")
        except Exception as _e:
            print(f"Warning: could not auto-read netclass clearances ({_e}); "
                  f"routing at the uniform clearance.")
            net_clearances = None

    # #435 companion: when --track-width was OMITTED, route each net at its OWN
    # netclass track width (a controlled-impedance signal class or a power class,
    # each with a different width), not the single Default-class width. Explicit
    # --track-width is honored verbatim for all nets (this stays empty). Floored at
    # the fab track minimum; a manual --power-nets-widths override still wins.
    net_track_widths = {}
    netclass_width_floors = {}
    if input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_track_width_map_by_id, fab_floors
            from fab_tiers import fab_floors as _tier_floors, \
                warn_fab_escalation as _wfe435
            _ncl = len(getattr(pcb_data.board_info, 'copper_layers', None)
                       or []) or 4
            _twfloor = fab_floors(_ncl).get('track_width', 0.0)
            # Clamp at the ADVANCED-tier floor, not the standard one
            # (2026-08-06): a netclass width is DESIGNER INTENT and may
            # legitimately sit between the two tiers (ecp5 /PF37- at
            # 0.0762 vs standard 0.0889 -- the GUI honored it and routed;
            # the old standard-floor clamp silently widened it CLI-side, a
            # cross-front divergence AND the reason no in-run ladder could
            # reach the width that closes the net). Escalation floor rule:
            # min(nominal, fab_track, netclass width), never below the
            # advanced tier; dipping below standard prints the same fab
            # warning the via rungs use.
            _adv_tw = _tier_floors(_ncl, tier='advanced') \
                .get('track_width', _twfloor)
            net_track_widths = {}
            for nid, w in net_track_width_map_by_id(
                    input_file,
                    {nid: n.name for nid, n in pcb_data.nets.items()}
            ).items():
                w2 = max(w, _adv_tw)
                if w2 < _twfloor - 1e-9:
                    _wfe435(f"netclass width net_{nid}")
                netclass_width_floors[nid] = w2
            # Nominal per-net widths only when no explicit --track-width
            # (the operator's flag wins for ROUTING); the floors above are
            # for the escalation ladders regardless. --impedance derives
            # width per layer, so netclass widths must not override it
            # (#610: this guard lives HERE now, so track_width_from_class
            # itself purely means "--track-width omitted" and can drive the
            # impedance width floor below).
            if track_width_from_class and impedance is None:
                net_track_widths = dict(netclass_width_floors)
            if net_track_widths:
                print(f"Auto-read netclass track widths for {len(net_track_widths)} "
                      f"non-Default net(s) (#435 single-ended companion).")
        except Exception as _e:
            print(f"Warning: could not auto-read netclass track widths ({_e}).")
            net_track_widths = {}

    # #600: cleared at the START of every run so a previous call's verdict can
    # never leak into this one (this is a function attribute, and the GUI calls
    # batch_route repeatedly in one process). main() reads it to skip the
    # post-passes that would otherwise mutate a reverted board.
    batch_route._improvement_gate_reverted = False

    # Issue #8: snapshot the input board's copper per net BEFORE any routing.
    # The final connectivity reconciliation reports against the copper that will
    # be WRITTEN (this original copper + the write-list's new copper), not against
    # pcb_data -- which accumulates orphan copper from rip/reroute that never
    # reaches the write-list and would make a net look connected when the output
    # has it split (glasgow /IO_Banks/IO_Buffer_A/P1).
    _orig_seg_by_net: Dict[int, list] = {}
    for _s in pcb_data.segments:
        _orig_seg_by_net.setdefault(_s.net_id, []).append(_s)
    _orig_via_by_net: Dict[int, list] = {}
    for _v in pcb_data.vias:
        _orig_via_by_net.setdefault(_v.net_id, []).append(_v)

    # Layers must be specified - we can't auto-detect which are ground planes
    if layers is None:
        layers = DEFAULT_4_LAYER_STACK
    print(f"Using {len(layers)} routing layers: {layers}")

    # Set default layer costs if not specified
    # 4+ layers: all 1.0 (inner layers available for routing)
    # 2 layers: F.Cu=1.0, B.Cu=3.0 (prefer top layer)
    if not layer_costs:
        if len(layers) >= 4:
            layer_costs = [1.0] * len(layers)
        else:
            layer_costs = [1.0 if layer == 'F.Cu' else 3.0 for layer in layers]

    # Full-stack normalization: the config must ALWAYS carry every board
    # copper layer -- copper on a layer absent from config.layers is invisible
    # to the obstacle maps, yet a via spans the whole stack, so a run invoked
    # with a layer subset on a 6/8-layer board could drop vias straight onto
    # unseen inner copper (butterstick DQ11: a rescue via on In3 +3V3 copper,
    # a real kicad clearance violation). Board layers the caller did not
    # request are APPENDED with FORBIDDEN cost (-1): no routed copper, but
    # their copper blocks vias and through-vias may span them (the documented
    # --layer-costs -1 semantics). Requested layers keep their order and
    # costs, so index-derived behavior (H/V direction preferences) is
    # unchanged for them.
    _board_cu = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
    _missing_cu = [l for l in _board_cu if l not in layers]
    if _missing_cu:
        from routing_constants import FORBIDDEN_LAYER_COST
        layers = list(layers) + _missing_cu
        layer_costs = list(layer_costs) + [FORBIDDEN_LAYER_COST] * len(_missing_cu)
        print(f"  Full-stack: appended {len(_missing_cu)} unrequested copper layer(s) "
              f"as FORBIDDEN obstacles (no routing, vias respect their copper): "
              f"{', '.join(_missing_cu)}")

    # Validate layer costs: any negative = forbidden (no copper placed; still an
    # obstacle), otherwise a multiplier in [1.0, 1000].
    for i, cost in enumerate(layer_costs):
        if cost >= 0 and (cost < 1.0 or cost > 1000):
            layer_name = layers[i] if i < len(layers) else f"layer {i}"
            from routing_exceptions import ConfigurationError
            raise ConfigurationError(f"Layer cost for {layer_name} must be negative (forbidden) or "
                                     f"between 1.0 and 1000, got {cost}")

    costs_str = ', '.join(f"{layers[i]}={layer_costs[i]}x" for i in range(min(len(layers), len(layer_costs))))
    print(f"  Layer costs: {costs_str}")

    # Calculate layer-specific widths for impedance-controlled routing.
    # #610: with --track-width OMITTED the impedance request sets the width
    # floor it implies (bounded below by the fab tier); an explicit
    # --track-width stays a verbatim floor. Clamps land in
    # impedance_width_clamped -> JSON_SUMMARY so pipelines can detect a run
    # that cannot meet its impedance request.
    from impedance import impedance_width_floor
    imp_width_floor, imp_floor_desc = impedance_width_floor(
        track_width, track_width_from_class,
        len(getattr(pcb_data.board_info, 'copper_layers', None) or []))
    impedance_width_clamped: Dict[str, List[float]] = {}
    layer_widths = {}
    coplanar_layer_widths = {}
    coplanar_net_ids = set()
    if impedance is None and coplanar_gap:
        # The coplanar gap only changes which IMPEDANCE formula picks the width.
        # With no target it has nothing to act on, and silently ignoring it
        # would leave the caller believing they routed a CPW (#486).
        print("WARNING: --coplanar-gap given without --impedance; it only "
              "selects the impedance model, so it has NO effect here. Add "
              "--impedance <ohms> to route as a coplanar waveguide.")
    if impedance is not None:
        if not pcb_data.board_info.stackup:
            print("WARNING: No stackup found in PCB file. Using fixed track width.")
        else:
            # #486: which nets run through a ground pour on their own layer?
            # An empty coplanar_nets with a gap set means "all of them", so the
            # base width dict itself is CPW-derived; a net list means only
            # those get CPW widths and everyone else stays microstrip.
            _cop_all = bool(coplanar_gap and coplanar_gap > 0 and not coplanar_nets)
            _cop_some = bool(coplanar_gap and coplanar_gap > 0 and coplanar_nets)

            print(f"\nCalculating trace widths for {impedance}Ω single-ended impedance...")
            if track_width_from_class:
                print(f"  --track-width not given: solved widths floor at the "
                      f"fab-tier track minimum {imp_width_floor}mm, not the "
                      f"default track width (#610)")
            layer_widths = calculate_layer_widths_for_impedance(
                pcb_data, layers, impedance,
                spacing=0.0, is_differential=False,
                fallback_width=track_width,
                min_width=imp_width_floor,
                coplanar_gap=coplanar_gap if _cop_all else 0.0,
                floor_desc=imp_floor_desc,
                clamp_report=impedance_width_clamped
            )
            print_impedance_routing_plan(pcb_data, layers, impedance, is_differential=False,
                                        min_width=imp_width_floor,
                                        coplanar_gap=coplanar_gap if _cop_all else 0.0)

            if _cop_some:
                coplanar_net_ids = {nid for _nm, nid in
                                    resolve_net_ids(pcb_data, coplanar_nets)}
                if not coplanar_net_ids:
                    print(f"WARNING: --coplanar-nets matched no nets: "
                          f"{' '.join(coplanar_nets)} (widths stay microstrip)")
                else:
                    coplanar_layer_widths = calculate_layer_widths_for_impedance(
                        pcb_data, layers, impedance,
                        spacing=0.0, is_differential=False,
                        fallback_width=track_width,
                        min_width=imp_width_floor,
                        coplanar_gap=coplanar_gap,
                        floor_desc=imp_floor_desc,
                        clamp_report=impedance_width_clamped
                    )
                    print(f"\n{len(coplanar_net_ids)} net(s) declared coplanar "
                          f"(gap {coplanar_gap}mm):")
                    print_impedance_routing_plan(pcb_data, layers, impedance,
                                                 is_differential=False,
                                                 min_width=imp_width_floor,
                                                 coplanar_gap=coplanar_gap)

    # #521: impedance declarations persist per net. A step routed WITH
    # --impedance records {net: ohms/pair_gap/coplanar_gap} (persisted next to
    # the DRC writeback); a later step touching those nets WITHOUT --impedance
    # recomputes the SAME widths from the stackup and applies them per-net, so
    # a rip/reroute keeps the impedance geometry instead of silently dropping
    # to this call's default width.
    net_layer_widths_map: Dict[int, Dict[str, float]] = {}
    _targets = resolve_net_ids(pcb_data, net_names) if net_names else []
    if impedance is not None:
        from protected_nets import note_impedance_specs
        note_impedance_specs({
            _nm: {'ohms': impedance, 'differential': False,
                  'coplanar_gap': (coplanar_gap if (coplanar_gap and coplanar_gap > 0
                                   and (not coplanar_nets or _nid in coplanar_net_ids))
                                   else 0.0)}
            for _nm, _nid in _targets})
    elif pcb_data.board_info.stackup:
        from protected_nets import read_impedance_for_pcb_data
        _stored = read_impedance_for_pcb_data(pcb_data, input_file)
        _redo = [(nm, nid, _stored[nm]) for nm, nid in _targets if nm in _stored]
        if _redo:
            # One width map per distinct declaration; per-net application via
            # config.net_layer_widths (single-ended engine margins ride
            # get_net_track_width, so per-net widths need no stamp change).
            _by_spec: Dict[tuple, list] = {}
            for nm, nid, sp in _redo:
                key = (float(sp.get('ohms', 0) or 0), bool(sp.get('differential')),
                       float(sp.get('pair_gap', 0) or 0), float(sp.get('coplanar_gap', 0) or 0))
                _by_spec.setdefault(key, []).append((nm, nid))
            for (ohms, is_diff, pair_gap, cop_gap), members in sorted(_by_spec.items()):
                if not ohms:
                    continue
                # #610: same floor rule as a live --impedance solve, so the
                # reapplied widths really ARE "the SAME widths" -- the old
                # min_width=track_width re-clamped them to THIS step's width.
                widths = calculate_layer_widths_for_impedance(
                    pcb_data, layers, ohms,
                    spacing=pair_gap, is_differential=is_diff,
                    fallback_width=track_width, min_width=imp_width_floor,
                    coplanar_gap=cop_gap,
                    floor_desc=imp_floor_desc,
                    clamp_report=impedance_width_clamped)
                for _nm, _nid in members:
                    net_layer_widths_map[_nid] = widths
                kind = 'differential' if is_diff else 'single-ended'
                cop = f", coplanar gap {cop_gap}mm" if cop_gap else ""
                print(f"  Reapplying stored {ohms:g} ohm {kind} impedance widths to "
                      f"{len(members)} net(s){cop} (recorded in .kicad_pro by an "
                      f"earlier --impedance step)")

    # Auto-detect BGA exclusion zones if not specified
    _sel_ids = [nid for _nm, nid in _targets]
    bga_exclusion_zones = setup_bga_exclusion_zones(
        pcb_data, disable_bga_zones, bga_exclusion_zones,
        selected_net_ids=_sel_ids)

    config_kwargs = get_common_config_kwargs(
        track_width=track_width, clearance=clearance, via_size=via_size,
        via_drill=via_drill, grid_step=grid_step, via_cost=via_cost,
        layers=layers, max_iterations=max_iterations,
        max_probe_iterations=max_probe_iterations, heuristic_weight=heuristic_weight,
        turn_cost=turn_cost, direction_preference_cost=direction_preference_cost,
        bus_enabled=bus_enabled, bus_detection_radius=bus_detection_radius,
        bus_attraction_radius=bus_attraction_radius, bus_attraction_bonus=bus_attraction_bonus,
        bus_min_nets=bus_min_nets,
        guide_corridor_enabled=guide_corridor_enabled, guide_corridor_layer=guide_corridor_layer,
        guide_corridor_spacing=guide_corridor_spacing,
        keepout_enabled=keepout_enabled, keepout_layer=keepout_layer,
        proximity_heuristic_factor=proximity_heuristic_factor,
        bga_exclusion_zones=bga_exclusion_zones,
        stub_proximity_radius=stub_proximity_radius, stub_proximity_cost=stub_proximity_cost,
        via_proximity_cost=via_proximity_cost, bga_proximity_radius=bga_proximity_radius,
        bga_proximity_cost=bga_proximity_cost, track_proximity_distance=track_proximity_distance,
        track_proximity_cost=track_proximity_cost, debug_lines=debug_lines, verbose=verbose,
        max_rip_up_count=max_rip_up_count, ripup_abandon_metric=ripup_abandon_metric,
        ripup_blocker_select=ripup_blocker_select,
        crossing_penalty=crossing_penalty,
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
        debug_memory=debug_memory, layer_costs=layer_costs
    )
    config_kwargs['power_tap_neckdown'] = power_tap_neckdown
    config_kwargs['neckdown_length'] = neckdown_length
    config_kwargs['neckdown_taper_length'] = neckdown_taper_length
    if direction_order is not None:
        config_kwargs['direction_order'] = direction_order
    if layer_widths:
        config_kwargs['layer_widths'] = layer_widths
        config_kwargs['impedance_target'] = impedance
    if coplanar_gap:
        config_kwargs['coplanar_gap'] = coplanar_gap
    if coplanar_net_ids and coplanar_layer_widths:
        config_kwargs['coplanar_net_ids'] = coplanar_net_ids
        config_kwargs['coplanar_layer_widths'] = coplanar_layer_widths
    if net_layer_widths_map:
        config_kwargs['net_layer_widths'] = net_layer_widths_map
    if collect_stats:
        config_kwargs['collect_stats'] = collect_stats
    # #581: an active (> 0) same-net pad via clearance keeps EVERY via this
    # run places off same-net SMD pads -- escape vias, the #189 via-in-pad
    # rescue, swap pad vias, finalize taps. Resolution: an explicit
    # --same-net-pad-clearance wins (> 0 activates; 0 / -1 explicitly off);
    # unset auto-reads the record an earlier chain step persisted into the
    # sibling .kicad_pro (kicad_routing_tools.same_net_pad_clearance).
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
    # The in-loop stub-debris trim must treat input copper as read-only in
    # --keep-input-copper runs; the loop only sees config, so carry it there.
    if keep_input_copper:
        object.__setattr__(config, '_keep_input_copper', True)

    try:
        config.bus_rip_resistance = float(
            env_knobs.BUS_RIP_RESISTANCE or config.bus_rip_resistance)
    except ValueError:
        pass
    if config.bus_rip_resistance != 1.0:
        print(f"Bus rip resistance: {config.bus_rip_resistance}x "
              f"(bus members deprioritized in the rip ladder)")
    # The SE loop needs the strategy to apply the explicit 'bus' ordering.
    config.ordering_strategy = ordering_strategy
    # #622 victim-priority restore scoping: the endgame sweep fires only in
    # a reconcile SUB-run (final_reconcile=False), where no deeper recovery
    # tier exists to pre-empt -- measured, firing in the outermost run's
    # loop or drain regressed a clean control both times (yz2, za2).
    config.final_reconcile = final_reconcile
    if config.ripup_blocker_select != 'count':
        print(f"Rip-up blocker-select algorithm: {config.ripup_blocker_select}")

    # Build guide-corridor waypoints once (issue #7). These steer the per-segment
    # A* through a user-drawn polyline; empty when the feature is off / no guide.
    config.corridor_waypoints = build_corridor_waypoints(pcb_data, config)
    if config.corridor_waypoints:
        print(f"Guide corridor: steering routes through {len(config.corridor_waypoints)} "
              f"waypoint(s) from {len(pcb_data.guide_paths)} polyline(s) on {config.guide_corridor_layer}")

    # Report keepout zones (issue #27): tracks are blocked from these polygons.
    if config.keepout_enabled and pcb_data.keepout_zones:
        print(f"Keepout: blocking routes from {len(pcb_data.keepout_zones)} "
              f"polygon(s) on {config.keepout_layer}")

    # Per-net netclass clearances (#326 B5): carried on the config so the
    # per-net obstacle cache and the same-run copper stamps reserve each net's
    # OWN class clearance (the base map additionally applies the max-flatten
    # below). net_id-keyed, GUI-fed today; harmless when empty.
    if net_clearances:
        config.net_clearances = {nid: c for nid, c in net_clearances.items()
                                 if c and c > 0}

    # #658 in-run river packing (KICAD_PACK_INLINE=1): each routed bus
    # member's runs are packed against already-committed sibling runs at
    # the copper choke point (add_route_to_pcb_data), BEFORE the route
    # becomes an obstacle -- the vacated lane is free for every LATER net
    # in this same pass, where the pack_river post-pass frees it only for
    # the next chain step. Groups precomputed once here.
    if env_knobs.PACK_INLINE:
        from pack_river import bus_groups
        _pg658 = bus_groups(pcb_data)
        _member658 = {}
        for _refs658, _mem658 in _pg658.items():
            for _nid658 in _mem658:
                _member658[_nid658] = set(_mem658) - {_nid658}
        pcb_data._pack_inline = {
            'clearance': config.clearance,
            'net_clearances': dict(config.net_clearances or {}),
            'members': _member658}
        print(f"In-run river packing: {len(_member658)} bus member net(s) "
              f"in {len(_pg658)} group(s)")

    # #435 companion: per-net netclass track widths (auto-read above when
    # --track-width was omitted). get_net_track_width() routes each net at its own
    # class width; a manual --power-nets-widths override below still wins.
    if net_track_widths:
        config.net_track_widths = dict(net_track_widths)
    if netclass_width_floors:
        config.netclass_width_floors = dict(netclass_width_floors)

    # Identify power nets and set up per-net widths
    if power_nets and power_nets_widths:
        if len(power_nets) != len(power_nets_widths):
            raise ValueError(f"--power-nets ({len(power_nets)}) and --power-nets-widths ({len(power_nets_widths)}) must have same length")
        power_net_widths = identify_power_nets(pcb_data, power_nets, power_nets_widths)
        if power_net_widths:
            config.power_net_widths = power_net_widths
            print(f"\nPower net width assignments ({len(power_net_widths)} nets):")
            # Group by width for summary
            width_groups: Dict[float, List[str]] = {}
            for net_id, width in power_net_widths.items():
                net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
                if width not in width_groups:
                    width_groups[width] = []
                width_groups[width].append(net_name)
            for width, names in sorted(width_groups.items()):
                if len(names) <= 5:
                    print(f"  {width}mm: {', '.join(names)}")
                else:
                    print(f"  {width}mm: {len(names)} nets ({', '.join(names[:3])}...)")

    # Find net IDs and filter already-routed nets
    net_ids = resolve_net_ids(pcb_data, net_names)
    # Flag target pads that sit at/over the board edge before routing, so an
    # unroutable off-board pad reads as a clear warning rather than a silent
    # exhaustive-search failure (issue #195).
    _edge_clear = board_edge_clearance if board_edge_clearance > 0 else clearance
    warn_targets_outside_board(pcb_data, net_ids,
                               edge_margin=_edge_clear + track_width / 2)
    # Every net in this run's --nets filter, by name (not just the routable ones
    # resolve_net_ids keeps). The dead-end sweep uses this so it also cleans
    # inherited stubs on in-filter nets it did not actively route -- single-pad,
    # already-connected, or failed nets -- while still excluding nets the user
    # left out (GND / power planes routed in a later stage). Issue #84.
    _scope_names = set(net_names or [])
    # #369 A16: resolve_net_ids returns (name, id) TUPLES -- the old
    # `or set(net_ids)` fallback filled the scope with tuples that can never
    # equal an int net_id, silently no-op'ing the dead-end sweep and the
    # stale-copper strips whenever the name scope came up empty. Union the
    # resolved ids in directly (as ints): pad-only nets, present in
    # pads_by_net but absent from pcb.nets, match no pcb.nets name and fell
    # out of scope entirely.
    sweep_scope_ids = ({nid for nid, net in pcb_data.nets.items()
                        if net.name in _scope_names}
                       | {nid for _name, nid in net_ids})
    if not net_ids:
        # The caller named nets and NOT ONE of them exists on this board. That
        # is an input error, not a no-op: the old soft return wrote an
        # unchanged passthrough copy and reported rc 0, so a net list whose
        # names all missed (run 11: a CRLF file gave every one of 88 names a
        # trailing \r) looked exactly like "nothing left to do". Raise, so the
        # CLI exits non-zero and the GUI surfaces a real error, instead of both
        # of them reporting success over an untouched board.
        if net_names:
            from routing_exceptions import NetNotFoundError
            raise NetNotFoundError(
                list(net_names),
                f"None of the {len(net_names)} requested net name(s) exist on "
                f"this board -- nothing was routed and no copper was written. "
                f"Check for stray whitespace (a CRLF net list gives every name "
                f"a trailing carriage return) or a stale net list. "
                f"First few: {', '.join(repr(n) for n in list(net_names)[:5])}")
        print("No valid nets to route!")
        if final_reconcile:
            _emit_summary_min(status='no_valid_nets')
        if return_results:
            return 0, 0, 0.0, _empty_results_data()
        _write_passthrough_output(input_file, output_file)
        return 0, 0, 0.0

    # #515 follow-up (PR #533): --force-reroute is an explicit request to
    # REPLACE the selected nets' existing routes. Strip their copper BEFORE the
    # already-connected filter, so they re-enter the route set and replan from
    # scratch. The originals were snapshotted into _orig_seg_by_net/_orig_via_
    # by_net above, so on success the #220/#284 stale strip drops them from the
    # written file; a net whose replan lands NO new copper gets them restored
    # (identity-preserved) before the freeze -- see the custody block ahead of
    # the #209 gate. Protection follows the rip-up rules exactly (#521):
    # matched groups / diff pairs are skipped unless named EXACTLY (no glob) in
    # --nets, KiCad-locked copper is never stripped, and plane (zone-owning)
    # nets belong to route_planes.
    force_ripped: Dict[int, Tuple[list, list]] = {}
    if force_reroute and not net_names:
        print("WARNING: --force-reroute requires an explicit --nets scope "
              "(it rips every selected net); ignoring it for this run.")
    elif force_reroute:
        _fr_zone_nids = {z.net_id for z in pcb_data.zones}
        _fr_copper_nids = ({s.net_id for s in pcb_data.segments}
                           | {v.net_id for v in pcb_data.vias})
        _fr_cand, _fr_skipped_plane = [], []
        for _name, _nid in net_ids:
            if _nid not in _fr_copper_nids:
                continue  # no existing copper -- routes normally anyway
            if _nid in _fr_zone_nids:
                _fr_skipped_plane.append(_name)
                continue
            _fr_cand.append((_name, _nid))
        if _fr_skipped_plane:
            print(f"--force-reroute: skipping {len(_fr_skipped_plane)} plane "
                  f"(zone-owning) net(s) -- planes belong to route_planes: "
                  f"{', '.join(_fr_skipped_plane[:6])}"
                  f"{', ...' if len(_fr_skipped_plane) > 6 else ''}")
        if _fr_cand:
            from protected_nets import (protection_map, filter_rippable_names,
                                        stash_rip_overrides)
            _fr_prot = protection_map(pcb_data, input_file)
            _fr_overrides = list(net_name_patterns
                                 if net_name_patterns is not None
                                 else net_names)
            # Run-6 z2 fix: the in-run ladders must see the same exact-name
            # overrides this pre-run filter honors.
            stash_rip_overrides(pcb_data, _fr_overrides)
            _fr_keep = set(_n for _n, _ in _fr_cand)
            if _fr_prot:
                _fr_keep = set(filter_rippable_names(
                    [_n for _n, _ in _fr_cand], _fr_prot,
                    override_patterns=_fr_overrides,
                    context="--force-reroute"))
            for _name, _nid in _fr_cand:
                if _name not in _fr_keep:
                    continue
                force_ripped[_nid] = (
                    [s for s in pcb_data.segments if s.net_id == _nid],
                    [v for v in pcb_data.vias if v.net_id == _nid])
        if force_ripped:
            pcb_data.segments = [s for s in pcb_data.segments
                                 if s.net_id not in force_ripped]
            pcb_data.vias = [v for v in pcb_data.vias
                             if v.net_id not in force_ripped]
            sweep_scope_ids |= set(force_ripped)
            print(f"--force-reroute: stripped "
                  f"{sum(len(s) for s, _ in force_ripped.values())} segment(s) / "
                  f"{sum(len(v) for _, v in force_ripped.values())} via(s) from "
                  f"{len(force_ripped)} net(s) for a from-scratch re-route")

    # #572: custody nets carry an exact-fill OPEN verdict from the outer
    # run's oracle -- the model-credit skip below must not clear them.
    # ANY net with links present bypasses: the punt measured OPEN on this
    # exact board, and no geometric heuristic may overrule it (a strapped-
    # copper check was tried and false-positived on eis -- the exact-fill
    # anchor coincided with a pre-existing track endpoint, which is a KISS,
    # not a weld). Nets whose links a previous lap welded are pruned from
    # oracle_links by the CALLER (the lap loop's landed-export), so re-laps
    # skip them the normal way.
    _assume_open572 = {l[0] for l in (oracle_links or []) if l and l[0]}
    # fragment_gate (#578): a zone-less net whose copper KiCad holds in
    # pieces must not be skipped as "Already fully connected". route_diff
    # deliberately keeps the default (a fragmented net entering the diff
    # engine is a separate behavior question).
    net_ids, _already_routed = filter_already_routed(
        pcb_data, net_ids, config, assume_open=_assume_open572,
        fragment_gate=True)
    # #515: --rip-existing-nets only rips copper that BLOCKS a net being
    # routed; a net dropped here as already-connected never routes, so naming
    # it in both --nets and --rip-existing-nets is a no-op. Warn instead of
    # staying silent.
    if rip_existing_nets and net_names:
        from net_queries import matches_net_filter as _mnf_ripwarn
        _rip_noop = [n for n, _reason in _already_routed
                     if not _reason.startswith('Only')
                     and _mnf_ripwarn(n, rip_existing_nets)]
        if _rip_noop:
            print(f"WARNING: {len(_rip_noop)} net(s) named in both --nets and "
                  f"--rip-existing-nets are already fully connected and will "
                  f"NOT be re-routed ({', '.join(_rip_noop[:6])}"
                  f"{', ...' if len(_rip_noop) > 6 else ''}). "
                  f"--rip-existing-nets only rips nets that block another "
                  f"route; pass --force-reroute to rip and re-route them "
                  f"from scratch (#515).")
    if not net_ids:
        print("All nets are already fully connected - nothing to route!")
        if final_reconcile:
            _emit_summary_min(status='already_connected')
        # The sweep runs HERE too (#659). The fragment gate diverts a net whose
        # extra fragments are all pad-less to this sweep instead of the router,
        # so a step whose WHOLE scope is diverted lands on this early return --
        # and the debris the gate's message promises to remove would ship
        # untouched. Measured on spartan6_4layer: three diverted nets took the
        # run from 448s to 6s, and every one of their bare vias survived
        # because the sweep only ran at the normal end of a run.
        if return_results:
            _rd659 = _empty_results_data()
            _late_orphan_sweep659(pcb_data, output_file, True, _rd659,
                                  None, keep_input_copper, skip_routing)
            return 0, 0, 0.0, _rd659
        _write_passthrough_output(input_file, output_file)
        _late_orphan_sweep659(pcb_data, output_file, False, None,
                              None, keep_input_copper, skip_routing)
        return 0, 0, 0.0

    # Track all segment layer modifications for file output
    all_segment_modifications = []
    all_swap_segments = []  # new copper from swap via-reuse connectors (#340)
    # Track all vias added during stub layer swapping
    all_swap_vias = []
    # Track total number of layer swaps applied
    total_layer_swaps = 0

    # Apply target swaps for single-ended swappable-nets
    single_ended_target_swaps: Dict[str, str] = {}
    single_ended_target_swap_info: List[Dict] = []
    boundary_debug_labels: List[Dict] = []  # Debug labels for boundary positions
    if swappable_net_patterns:
        from target_swap import apply_single_ended_target_swaps, summarize_target_swaps

        # Find matching single-ended nets
        swappable_se_nets = find_single_ended_nets(
            pcb_data,
            swappable_net_patterns,
            exclude_net_ids=set()
        )

        if len(swappable_se_nets) >= 2:
            print(f"\nAnalyzing target swaps for {len(swappable_se_nets)} single-ended net(s)...")
            single_ended_target_swaps, single_ended_target_swap_info = apply_single_ended_target_swaps(
                pcb_data, swappable_se_nets, config,
                lambda net_id: get_net_endpoints(pcb_data, net_id, config),
                use_boundary_ordering=mps_unroll
            )

    # Single-ended layer swap optimization (before MPS ordering)
    all_stubs_by_layer = {}
    stub_endpoints_by_layer = {}
    if enable_layer_switch and net_ids:
        total_layer_swaps += apply_single_ended_layer_swaps(
            pcb_data, config, net_ids,
            can_swap_to_top_layer, all_segment_modifications, all_swap_vias,
            verbose=verbose, all_swap_segments=all_swap_segments
        )
        # NOTE: apply_stub_layer_switch already appends each swap via to
        # pcb_data.vias itself -- re-appending all_swap_vias here put every
        # pad swap via on the board TWICE (double obstacle stamp, and the
        # board carried one more via than the written file; found by the
        # FILE_LEDGER audit on ottercast AP_WAKE_BT et al).

    # Skip (and loudly list) nets with <2 pads -- unroutable, and attempting
    # them wastes the router/ordering. Do this BEFORE ordering so MPS never
    # sees them.
    net_ids = filter_routable_nets(pcb_data, net_ids)

    # Apply net ordering strategy
    if ordering_strategy in ("mps", "bus"):
        # 'bus' = mps base order; the SE loop then moves bus groups to the
        # front (members middle-out). Explicit form of what --bus implied.
        net_ids, mps_layer_swaps = order_nets_mps(
            pcb_data=pcb_data,
            net_ids=net_ids,
            diff_pairs={},
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

    # #472 direct-first ordering (KICAD_DIRECT_FIRST=0 disables): nets with a
    # BARE BGA ball (>=2 pads, no attached copper -- the fanout-deferred
    # direct-route class) move to the FRONT of the order, keeping relative
    # order within each partition. Their short natural surface lanes get
    # claimed before the long nets and bus corridors contend for the same
    # space (the human's sequence: nearby connections first). Inert on
    # boards with no bare balls -- stubs/vias at every ball leave the order
    # untouched. Board-state-driven and engine-level (GUI parity).
    _direct_front_ids: set = set()  # captured for the #589 plan reorder
    if net_ids and env_knobs.DIRECT_FIRST:
        _bga_refs = set()
        from kicad_parser import find_components_by_type
        _sel_set = {nid for _nm, nid in net_ids}
        _pts_by = {}
        for _s in pcb_data.segments:
            if _s.net_id in _sel_set:
                _pts_by.setdefault(_s.net_id, []).append((_s.start_x, _s.start_y))
                _pts_by[_s.net_id].append((_s.end_x, _s.end_y))
        for _v in pcb_data.vias:
            if _v.net_id in _sel_set:
                _pts_by.setdefault(_v.net_id, []).append((_v.x, _v.y))
        _direct_ids = set()
        for _fp in find_components_by_type(pcb_data, 'BGA'):
            for _p in _fp.pads:
                if (_p.net_id in _sel_set and not _p.drill
                        and len(pcb_data.pads_by_net.get(_p.net_id, [])) >= 2
                        and not any(abs(_x - _p.global_x) < 0.05
                                    and abs(_y - _p.global_y) < 0.05
                                    for (_x, _y) in _pts_by.get(_p.net_id, ()))):
                    _direct_ids.add(_p.net_id)
        # BIG nets are NOT promoted. Direct-first exists to let short natural
        # surface lanes get claimed early, but a big power rail has the most BGA
        # balls, so the rule front-loads exactly the most expensive nets on the
        # board -- and everything routed afterwards then rips them. Measured on
        # muzy_zynq2: +1V8/+3V3/+1V0 were routed #1/#2/#3 of 136 and ripped
        # 26/47/12 times each afterwards, with +3V3 alone accounting for 17115 of
        # the board's 42447 torn segments (40% of ALL destroyed copper). +5V, the
        # one big rail that landed late (#131), was ripped only 7 times.
        # Threshold is a MULTIPLE OF THE BOARD'S MEDIAN net cost, not an absolute
        # value: span is in mm, so an absolute cut-off does not port between a
        # 20mm module and a 200mm backplane. KICAD_BIG_NET_FACTOR=0 disables.
        _big_ids = set()
        if _direct_ids:
            from net_cost import net_cost, big_net_threshold
            _th = big_net_threshold(pcb_data, [nid for _nm, nid in net_ids])
            if _th > 0:
                _big_ids = {nid for nid in _direct_ids
                            if net_cost(pcb_data, nid) > _th}
                _direct_ids = _direct_ids - _big_ids
        if _direct_ids:
            _front = [t for t in net_ids if t[1] in _direct_ids]
            _rest = [t for t in net_ids if t[1] not in _direct_ids]
            net_ids = _front + _rest
            _direct_front_ids = set(_direct_ids)
            print(f"Direct-first ordering (#472): {len(_front)} bare-ball "
                  f"net(s) moved to the front: "
                  f"{', '.join(nm for nm, _ in _front[:8])}"
                  f"{', ...' if len(_front) > 8 else ''}")
        if _big_ids:
            _names = [nm for nm, nid in net_ids if nid in _big_ids]
            print(f"  ({len(_big_ids)} BIG net(s) held OUT of direct-first to cut "
                  f"rip churn: {', '.join(_names[:6])}"
                  f"{', ...' if len(_names) > 6 else ''})")

    # All nets are single-ended in this tool
    single_ended_nets = net_ids
    total_routes = len(single_ended_nets)

    # Generate stub position labels for single-ended nets (when debug_lines enabled)
    if debug_lines and single_ended_nets:
        from target_swap import generate_single_ended_debug_labels
        stub_labels = generate_single_ended_debug_labels(
            pcb_data, single_ended_nets,
            lambda net_id: get_net_endpoints(pcb_data, net_id, config),
            use_mps_ordering=mps_unroll
        )
        if stub_labels:
            print(f"Generated {len(stub_labels)} stub position labels for single-ended nets")
            boundary_debug_labels.extend(stub_labels)

    results = []
    pad_swaps = []  # List of (pad1, pad2) tuples for nets that need swapping
    successful = 0
    failed = 0
    total_time = 0
    total_iterations = 0
    # Note: all_swap_vias is initialized at line 476 and populated during layer swaps

    # Skip routing if requested - just write output with swaps and debug info
    if skip_routing:
        print(f"\n--skip-routing: Skipping actual routing of {total_routes} items")
        print("Writing output file with swaps and debug labels only...")
        # Clear the lists so routing loops don't execute
        single_ended_nets = []
    else:
        print(f"\nRouting {total_routes} single-ended net(s)...")
        print("=" * 60)

    # Global planning pass (#589, KICAD_GLOBAL_PLAN=1 opt-in): rough-route
    # every net once against a throwaway base-style map (probes commit
    # nothing). Predicted paths become the plan's products: soft corridor
    # reservations (folded owner-exempt into every obstacle build via
    # global_plan.add_plan_source), a plan-informed net order (the #472
    # direct-first partition is preserved), clique-aware per-net layer
    # preferences (a soft step-cost discount applied in the SE loop), and
    # optional plan-driven stub layer swaps. The pass runs HERE -- before
    # the base obstacle map -- because the swaps MUTATE stub copper, which
    # must precede every map/cache build (the MPS-swap precedent); its
    # probe map is built and discarded locally either way. Engine-level,
    # so the GUI gets it for free. Probe iterations join total_iterations
    # so the pass is graded honestly against spending the same budget on
    # detailed search.
    # TOP-LEVEL RUNS ONLY: gated on final_reconcile, the structural
    # recursion guard every internal sub-run clears (the reconcile laps and
    # the repair_planes reconnects) -- planning exists to pre-route an open
    # board, and on a finished board the probes are slow and useless
    # (glasgow: the reconcile sub-run burned 11.6s / 92k iterations probing
    # 19 nets against full copper for 0 usable conflicts). Coupling note: a
    # deliberate top-level final_reconcile=False run also skips the plan.
    from global_plan import (plan_global_routes, apply_plan_order,
                             apply_plan_layer_swaps)
    _gp_layer_map = {name: i for i, name in enumerate(config.layers)}
    _gplan = (plan_global_routes(pcb_data, config, single_ended_nets,
                                 _gp_layer_map, verbose=verbose,
                                 net_clearances=net_clearances)
              if final_reconcile else None)
    if _gplan is not None:
        config._global_plan = _gplan
        _gp_nets_in = list(single_ended_nets)
        single_ended_nets = apply_plan_order(single_ended_nets, _gplan,
                                             front_ids=_direct_front_ids)
        # #589 scorer dump: plan state as JSON for offline order/layer
        # evaluation; _DUMP_EXIT stops here (before swaps mutate copper)
        # so a dump run costs probe-time only.
        if env_knobs.GLOBAL_PLAN.get('dump'):
            from global_plan import dump_plan
            dump_plan(env_knobs.GLOBAL_PLAN['dump'], _gplan, _gp_nets_in,
                      single_ended_nets, _direct_front_ids, config)
            if env_knobs.GLOBAL_PLAN.get('dump_exit'):
                import sys as _sys
                _sys.exit(0)
        total_iterations += _gplan.probe_iterations
        total_layer_swaps += apply_plan_layer_swaps(
            pcb_data, config, _gplan, single_ended_nets,
            all_segment_modifications, all_swap_vias,
            all_stubs_by_layer=all_stubs_by_layer,
            can_swap_to_top_layer=can_swap_to_top_layer, verbose=verbose)
        # #589 escape fanout: dogbone the plan-assigned ends the swap path
        # could not serve (bare pads / no pad-center via fit) -- still
        # BEFORE the base map build, so all downstream maps see the copper.
        from global_plan import apply_plan_escape_fanout
        apply_plan_escape_fanout(pcb_data, config, _gplan,
                                 single_ended_nets, all_swap_vias,
                                 all_swap_segments, verbose=verbose)

    # Build base obstacle map once (excludes all nets we're routing)
    all_net_ids_to_route = [nid for _, nid in net_ids]

    # Issue #103: pre-existing routed nets matching --rip-existing-nets become
    # ELIGIBLE for rip-up when blocking analysis names them. Their copper is
    # excluded from the static base map and registered like in-process routed
    # nets (per-net obstacles + routed_results), so the normal rip/re-route/
    # restore machinery applies. Without this, tracks committed by a previous
    # run are unrippable and every rechained retry dies with
    # 'no rippable blockers found'.
    existing_rippable: List[int] = []
    if rip_existing_nets:
        from net_queries import matches_net_filter
        to_route_set = set(all_net_ids_to_route)
        zone_net_ids = {z.net_id for z in pcb_data.zones}
        seg_net_ids = {s.net_id for s in pcb_data.segments}
        for nid in sorted(seg_net_ids):
            if nid == 0 or nid in to_route_set:
                continue
            if nid in zone_net_ids:
                continue  # plane nets belong to route_planes, not rip-up
            net = pcb_data.nets.get(nid)
            if not net or not net.name:
                continue
            if matches_net_filter(net.name, rip_existing_nets):
                existing_rippable.append(nid)
        # #521: protected nets (length-matched groups, routed diff pairs --
        # recorded in the sibling .kicad_pro by the step that made them) are
        # excluded from COLLATERAL rips. Naming a net exactly (no glob) in
        # --rip-existing-nets or --nets is the deliberate override. Nets with
        # KiCad-LOCKED copper are excluded unconditionally (no override).
        if existing_rippable:
            from protected_nets import (protection_map, filter_rippable_names,
                                        stash_rip_overrides)
            _prot = protection_map(pcb_data, input_file)
            # Override source: the RAW --nets patterns when main() passed
            # them -- expansion turns a glob into exact names, which made
            # any glob-selected protected net "exactly named" here (#521's
            # override is deliberately no-glob; found by the force-reroute
            # test). rip_existing_nets is never expanded, so it stays as-is.
            _rip_overrides = (list(rip_existing_nets)
                              + list(net_name_patterns
                                     if net_name_patterns is not None
                                     else (net_names or [])))
            # Run-6 z2 fix: stash for the in-run ladders too.
            stash_rip_overrides(pcb_data, _rip_overrides)
            if _prot:
                _keep = set(filter_rippable_names(
                    [pcb_data.nets[n].name for n in existing_rippable], _prot,
                    override_patterns=_rip_overrides,
                    context="--rip-existing-nets"))
                existing_rippable = [n for n in existing_rippable
                                     if pcb_data.nets[n].name in _keep]
        if existing_rippable:
            names = [pcb_data.nets[n].name for n in existing_rippable[:6]]
            print(f"{len(existing_rippable)} pre-existing net(s) eligible for rip-up: "
                  f"{', '.join(names)}{'...' if len(existing_rippable) > 6 else ''}")
            # A ripped-existing net's copper is this run's responsibility from
            # here on: include it in the cleanup/strip scope so the #220 stale
            # input-copper strip (and dead-end/cycle sweeps) cover it. Without
            # this a rerouted +3.3V shipped BOTH its original copper (crossing
            # nets routed through the vacated corridor) and its reroute (#300
            # follow-up, rp2350_dev GPIO4).
            sweep_scope_ids |= set(existing_rippable)
            # #513 item 5: a ripped-existing net re-routes at THIS invocation's
            # width resolution, silently dropping e.g. a 1.5mm power width to
            # the 0.25 default when the retry omitted --power-nets (nascom VCC;
            # check_connected doesn't check width, so nothing caught it).
            # Preserve each rippable net's routed width: inject its
            # dominant-by-length input width into the per-net width map when it
            # is wider than what this run would use. An explicit --power-nets
            # entry for the net still wins (priority 1 in get_net_track_width).
            from routing_common import dominant_net_widths
            _in_widths = dominant_net_widths(pcb_data.segments)
            for _rid in existing_rippable:
                _w_in = _in_widths.get(_rid, 0.0)
                if _w_in <= 0:
                    continue
                if _rid in (getattr(config, 'power_net_widths', None) or {}):
                    continue  # explicit override this run wins
                _w_now = config.get_net_track_width(_rid, config.layers[0])
                if _w_in > _w_now + 1e-6:
                    if config.net_track_widths is None:
                        config.net_track_widths = {}
                    config.net_track_widths[_rid] = _w_in
                    _rn = pcb_data.nets[_rid].name if _rid in pcb_data.nets else _rid
                    print(f"  Preserving routed width {_w_in}mm of rippable "
                          f"existing net {_rn} (this run would re-route it at "
                          f"{_w_now}mm; pass --power-nets/--power-nets-widths "
                          f"to override)")
    # #513 item 5, --force-reroute edition: a force-ripped net replans at THIS
    # invocation's width resolution; preserve its routed width the same way.
    # Its input segments are already stripped from pcb_data, so source the
    # dominant width from the force_ripped stash.
    if force_ripped:
        from routing_common import dominant_net_widths as _dnw_fr
        _fr_widths = _dnw_fr([s for _segs, _ in force_ripped.values()
                              for s in _segs])
        for _rid, _w_in in _fr_widths.items():
            if _w_in <= 0:
                continue
            if _rid in (getattr(config, 'power_net_widths', None) or {}):
                continue  # explicit override this run wins
            _w_now = config.get_net_track_width(_rid, config.layers[0])
            if _w_in > _w_now + 1e-6:
                if config.net_track_widths is None:
                    config.net_track_widths = {}
                config.net_track_widths[_rid] = _w_in
                _rn = pcb_data.nets[_rid].name if _rid in pcb_data.nets else _rid
                print(f"  Preserving routed width {_w_in}mm of force-rerouted "
                      f"net {_rn} (this run would re-route it at {_w_now}mm; "
                      f"pass --power-nets/--power-nets-widths to override)")
    if progress_callback:
        progress_callback(0, 0, "Building base obstacle map...")
    print("Building base obstacle map...")
    base_start = time.time()

    # Tap relocation (#424): zone-backed plane nets become base-map-excluded
    # and per-net cached so single-tap surgery is exact whole-net cache
    # recompute (ref-count-balanced by construction); deliberately NOT in
    # routed_results, so the whole-net rip ladder can never name them.
    relocatable_plane_ids = []
    from tap_relocation import tap_relocation_enabled as _tap_reloc_on
    if _tap_reloc_on():
        _zone_nids = {z.net_id for z in pcb_data.zones if z.net_id > 0}
        relocatable_plane_ids = sorted(
            n for n in _zone_nids
            if n not in all_net_ids_to_route and n not in existing_rippable
            and n in pcb_data.nets)
        if relocatable_plane_ids:
            print(f"Tap relocation armed: {len(relocatable_plane_ids)} plane "
                  f"net(s) cached for single-tap surgery")
    # Main-pass rip candidacy for PRE-EXISTING nets (0805, design intent):
    # a blocker routed by an EARLIER chain step used to be visible only as a
    # text hint ("the blocking copper belongs to N pre-existing net(s) ...")
    # while the run died with 'no rippable blockers found' -- even when the
    # blocker was one cell of an ordinary unprotected signal net. Plain
    # pre-existing nets are fair game: register them exactly like
    # --rip-existing-nets matches (base-map exclusion + per-net cache +
    # routed_results entry, #103), so every rip ladder (SE loop, reroute
    # loop, diff loop, Phase 3) can name and rip them, WITH the same custody
    # -- a ripped victim is queued for reroute this run and its original
    # copper is restored if the reroute fails. Guards:
    #   - never #521-protected or KiCad-locked (filter_rippable_names; the
    #     override stays "exact name in --nets/--rip-existing-nets"),
    #   - never '!'-negated in the caller's --nets patterns,
    #   - never plane/zone-backed nets (planes belong to route_planes),
    #   - small nets only (<= 30 segments and <= 6 vias): rip surgery,
    #     not bulldozing a bus,
    #   - never a net whose CLASS clearance would RAISE the cross-class
    #     routing floor (set_net_clearances computes the floor over the
    #     base-map exclusions; auto-candidates must not change routing on
    #     multi-class boards when nothing gets ripped).
    # Engine-side (batch_route internals), so CLI and GUI inherit it with no
    # new flag or control; KICAD_RIP_PREEXISTING=0 is the kill switch.
    # rip_preexisting=False comes from the RECONCILE SUB-RUN: candidacy is
    # only legal in a run whose victims a reconcile can catch, and the
    # sub-run has none (final_reconcile=False, the recursion guard) -- its
    # unrecoverable victims were term3_ecp5's +1V1//PF37+//PE26+ opens.
    # The sub-run falls back to hinted #103 authority only (explicitly
    # ripped nets join its routing scope as first-class targets).
    if (rip_preexisting
            if rip_preexisting is not None
            else os.environ.get('KICAD_RIP_PREEXISTING', '1') != '0'):
        from net_queries import split_net_patterns, net_pattern_matches
        _pe_to_route = set(all_net_ids_to_route)
        _pe_zone_nids = {z.net_id for z in pcb_data.zones}
        _pe_seen = set(existing_rippable)
        _pe_seg_count: Dict[int, int] = {}
        for _s in pcb_data.segments:
            _pe_seg_count[_s.net_id] = _pe_seg_count.get(_s.net_id, 0) + 1
        _pe_via_count: Dict[int, int] = {}
        for _v in pcb_data.vias:
            _pe_via_count[_v.net_id] = _pe_via_count.get(_v.net_id, 0) + 1
        _pe_caller_pats = list(net_name_patterns if net_name_patterns is not None
                               else (net_names or []))
        _pe_auto: List[int] = []
        for _nid in sorted(_pe_seg_count):
            if (_nid == 0 or _nid in _pe_to_route or _nid in _pe_seen
                    or _nid in _pe_zone_nids):
                continue
            _pe_net = pcb_data.nets.get(_nid)
            if not _pe_net or not _pe_net.name:
                continue
            if _pe_seg_count[_nid] > 30 or _pe_via_count.get(_nid, 0) > 6:
                continue
            # '!'-negated in the caller's --nets = an explicit hands-off
            # (split_net_patterns keeps active-low literals like '!RESET'
            # selectable by name, so those don't read as negations here).
            _, _pe_exc = split_net_patterns(_pe_caller_pats, {_pe_net.name})
            if any(net_pattern_matches(_pe_net.name, _p) for _p in _pe_exc):
                continue
            _pe_auto.append(_nid)
        if _pe_auto:
            from protected_nets import protection_map, filter_rippable_names
            _pe_prot = protection_map(pcb_data, input_file)
            if _pe_prot:
                _pe_keep = set(filter_rippable_names(
                    [pcb_data.nets[n].name for n in _pe_auto], _pe_prot,
                    override_patterns=list(rip_existing_nets or []) + _pe_caller_pats,
                    context="pre-existing rip candidacy"))
                _pe_auto = [n for n in _pe_auto
                            if pcb_data.nets[n].name in _pe_keep]
        if _pe_auto and net_clearances:
            _pe_floor0_ids = (all_net_ids_to_route + existing_rippable
                              + relocatable_plane_ids)
            _pe_floor0 = max([config.clearance]
                             + [net_clearances[n] for n in _pe_floor0_ids
                                if n in net_clearances])
            _pe_auto = [n for n in _pe_auto
                        if net_clearances.get(n, 0.0) <= _pe_floor0 + 1e-12]
        if _pe_auto:
            print(f"{len(_pe_auto)} unprotected pre-existing net(s) registered "
                  f"as main-pass rip candidates (<=30 seg/<=6 via, "
                  f"custody-backed; KICAD_RIP_PREEXISTING=0 disables)")
            # #513 item 5 parity with --rip-existing-nets: a ripped victim
            # re-routes at THIS invocation's width resolution -- preserve its
            # routed width so a wide power stub cannot reroute thin. (Silent
            # per-net: with board-wide candidacy the per-net print would be
            # noise; an explicit --power-nets entry still wins.)
            from routing_common import dominant_net_widths as _pe_dnw
            _pe_widths = _pe_dnw(pcb_data.segments)
            for _nid in _pe_auto:
                _w_in = _pe_widths.get(_nid, 0.0)
                if _w_in <= 0 or _nid in (getattr(config, 'power_net_widths',
                                                  None) or {}):
                    continue
                _w_now = config.get_net_track_width(_nid, config.layers[0])
                if _w_in > _w_now + 1e-6:
                    if config.net_track_widths is None:
                        config.net_track_widths = {}
                    config.net_track_widths[_nid] = _w_in
            existing_rippable.extend(_pe_auto)
    base_map_exclusions = all_net_ids_to_route + existing_rippable + relocatable_plane_ids
    # Cross-class clearance: install the per-net class map + routing-side floor on
    # config so BOTH the base map and every incremental in-run obstacle stamper
    # price foreign copper at KiCad's pairwise max(classA, classB). The floor is
    # computed over the ROUTED nets (== base map's nets_to_route) so the base map
    # and the incremental stampers agree. Inert when net_clearances is empty.
    config.set_net_clearances(net_clearances, base_map_exclusions)
    # #498: per-layer .kicad_dru clearance rules, installed engine-side so the
    # GUI inherits them with no wiring (see kicad_dru.install_layer_clearances).
    from kicad_dru import install_layer_clearances, install_track_clearances
    install_layer_clearances(config, layer_clearances, input_file, pcb_data)
    # Track-scoped .kicad_dru rules (#735), same engine-side pattern (raise-only
    # on seg-vs-seg stamps; effective map over THIS call's routed set).
    install_track_clearances(config, track_clearances, input_file, pcb_data,
                             routed_net_ids=base_map_exclusions)
    # #568: arming is run-scoped and the flag is module-global, so reset it
    # per call -- otherwise one unfrozen-base run would disarm rung-1 legality
    # for every later run in the same process (the GUI's whole session).
    try:
        from obstacle_cache import set_rung_unsafe as _rearm
        _rearm(bool(getattr(pcb_data, '_via_rung_unsafe', False)))
    except Exception:
        pass
    # Carry the RESOLVED map into the end-of-run reconciliation kwargs, exactly
    # like board_edge_clearance above: the reconciliation self-invocation reads
    # the OUTPUT file, whose sibling .kicad_dru does not exist yet (main()'s
    # fix_project_for_output copies it after batch_route returns), so its own
    # auto-read would find NOTHING and the sub-run would route blind to the
    # rules (caught by test_dru_layer_clearance_e2e: a reconciliation +3V3 via
    # 0.25mm inside the B.Cu rule against GND).
    _reconcile_kwargs['layer_clearances'] = dict(config.layer_clearances)
    # #581: same reasoning -- the reconciliation sub-run reads the OUTPUT file
    # whose .kicad_pro record does not exist yet mid-run; forward the resolved
    # value (None keeps the sub-run's own auto-read for the inactive case).
    if config.same_net_pad_clearance > 0:
        _reconcile_kwargs['same_net_pad_clearance'] = config.same_net_pad_clearance
    _reconcile_kwargs['track_clearances'] = dict(config.track_clearances)
    # #568: rung-1 via legality is only sound when base copper blocks EVERY
    # rung, i.e. when the base is frozen into the static keep-out bitmap. The
    # one remaining path that builds an UNFROZEN base is KICAD_NO_STATIC_BASE
    # (the vis branch that used to share this hazard is gone with #569), so
    # disarm the dual stamping there -- pcb_data for the cache path,
    # set_rung_unsafe for the RAW map mirrors, which see only
    # (obstacles, config) and cannot read the flag.
    if env_knobs.NO_STATIC_BASE:
        pcb_data._via_rung_unsafe = True
        from obstacle_cache import set_rung_unsafe as _set_rung_unsafe
        _set_rung_unsafe(True)
    base_obstacles = build_base_obstacle_map(
        pcb_data, config, base_map_exclusions,
        net_clearances=net_clearances,
        # #422: base holds only permanent copper/geometry (target + rippable
        # nets live in the per-net caches on a CLONE); stamp it straight into
        # the static keep-out bitmap so the working clone carries it as bits.
        static_base=not env_knobs.NO_STATIC_BASE,
        # #556: sub-phase progress so the GUI bar moves during the build
        progress_callback=progress_callback)

    base_elapsed = time.time() - base_start
    print(f"Base obstacle map built in {base_elapsed:.2f}s")
    if debug_memory:
        mem_after_base = get_process_memory_mb()
        print(format_memory_stats("After base obstacle map", mem_after_base, mem_after_base - mem_start))

    # Save original (pre-routing) segment signatures to preserve stubs during sync
    # We use object identity since segments are mutable and could be duplicated
    # Keep the objects alive alongside the id set: recycled ids of GC'd
    # originals otherwise alias NEW segments during sync (see route_diff, #195).
    _original_segments_keepalive = list(pcb_data.segments)
    original_segment_ids = set(id(s) for s in _original_segments_keepalive)

    # Get unrouted nets for stub proximity costs
    # Use sorted list for deterministic iteration order
    if minimal_obstacle_cache:
        # Only consider nets we're routing (faster for re-routing a few nets)
        all_unrouted_net_ids = sorted(all_net_ids_to_route)
        print(f"Using minimal obstacle cache for {len(all_unrouted_net_ids)} nets being routed")
    else:
        # All unrouted nets in the PCB (full analysis for stub proximity)
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

    # Pre-compute net obstacles for caching (speeds up per-route setup)
    if progress_callback:
        progress_callback(0, 0, "Pre-computing net obstacle cache...")
    print("Pre-computing net obstacle cache...")
    cache_start = time.time()
    net_obstacles_cache = precompute_all_net_obstacles(
        pcb_data, list(all_unrouted_net_ids), config,
        extra_clearance=0.0, diagonal_margin=defaults.DIAGONAL_MARGIN
    )
    # Rippable pre-existing nets need cache entries too: the working obstacle
    # map is base + cache, and their copper was excluded from base (issue #103).
    if existing_rippable or relocatable_plane_ids:
        from obstacle_cache import precompute_net_obstacles
        for nid in existing_rippable + relocatable_plane_ids:
            net_obstacles_cache[nid] = precompute_net_obstacles(
                pcb_data, nid, config, extra_clearance=0.0, diagonal_margin=defaults.DIAGONAL_MARGIN)

    cache_time = time.time() - cache_start
    print(f"Net obstacle cache built in {cache_time:.2f}s ({len(net_obstacles_cache)} nets)")
    if debug_memory:
        mem_after_cache = get_process_memory_mb()
        cache_size = estimate_net_obstacles_cache_mb(net_obstacles_cache)
        print(format_memory_stats("After net obstacles cache", mem_after_cache, mem_after_cache - mem_start))
        print(f"[MEMORY]   Cache estimated size: {cache_size:.1f} MB for {len(net_obstacles_cache)} nets")

    # Build working obstacle map (base + all nets) for incremental updates
    # Uses reference counting in Rust to correctly handle cells blocked by multiple nets
    if progress_callback:
        progress_callback(0, 0, "Building working obstacle map...")
    working_obstacles = build_working_obstacle_map(base_obstacles, net_obstacles_cache)
    # Shrink internal allocations to reduce memory footprint
    working_obstacles.shrink_to_fit()
    if debug_memory:
        mem_after_working = get_process_memory_mb()
        print(format_memory_stats("After working obstacle map", mem_after_working, mem_after_working - mem_start))

    # Route trace (#482, KICAD_ROUTE_TRACE=1): attach a recorder to pcb_data so
    # the two copper choke points (add_route_to_pcb_data /
    # remove_route_from_pcb_data) log every segment/via added, ripped, and
    # restored, in order, for animating the routing process. Default-off.
    # Gate on a real output path: an internal reconnect batch_route (e.g.
    # repair_planes) calls this with output_file="" and must not
    # start/dump a trace of its own (the plane front-end keeps a local one).
    if output_file:
        from route_trace import attach_trace as _attach_route_trace
        _attach_route_trace(pcb_data)

    # Create routing state object to hold all shared state
    state = create_routing_state(
        pcb_data=pcb_data,
        config=config,
        all_net_ids_to_route=all_net_ids_to_route,
        base_obstacles=base_obstacles,
        diff_pair_base_obstacles=None,  # Not used for single-ended
        diff_pair_extra_clearance=0.0,
        gnd_net_id=None,  # Not used for single-ended
        all_unrouted_net_ids=all_unrouted_net_ids,
        total_routes=total_routes,
        enable_layer_switch=enable_layer_switch,
        debug_lines=debug_lines,
        target_swaps={},  # Diff pair swaps not used
        target_swap_info=[],
        single_ended_target_swaps=single_ended_target_swaps,
        single_ended_target_swap_info=single_ended_target_swap_info,
        all_segment_modifications=all_segment_modifications,
        all_swap_vias=all_swap_vias,
        total_layer_swaps=total_layer_swaps,
        net_obstacles_cache=net_obstacles_cache,
        working_obstacles=working_obstacles,
        # Latent today (only diff_pair_loop reads state.cancel_check /
        # state.progress_callback, and route_diff passes its own), but the
        # single-ended state carried None while its caller had both in hand --
        # any future consumer would silently be uncancellable and silent.
        cancel_check=cancel_check,
        progress_callback=progress_callback,
    )

    # Create local aliases for frequently-used state fields
    routed_net_ids = state.routed_net_ids
    routed_net_paths = state.routed_net_paths
    routed_results = state.routed_results
    track_proximity_cache = state.track_proximity_cache
    layer_map = state.layer_map

    # #572: exact oracle links -> per-net-id forced edges for the SE loop
    # (and its rip/retry/reroute paths). Malformed entries are dropped, not
    # fatal -- the net then routes the normal (derivation) way.
    if oracle_links:
        _name_to_id572 = {n.name: i for i, n in pcb_data.nets.items()}
        # Two link shapes are not merely useless as forced edges, they are
        # ACTIVELY destructive, because route_oracle_links returns on the
        # first failure and a refused link discards the copper of every link
        # already routed for that net:
        #   * ZERO-LENGTH (A == B). Nothing to route, so the A* reports
        #     success with no copper -- which route_oracle_links must refuse
        #     as the #572 false-weld fingerprint. Measured on storm_tracker's
        #     final oracle payload: 113 of 400 links (62 of +3V3's 87), each
        #     able to abort its net's whole forced-edge list. They arise when
        #     two exact-fill clusters' nearest approach is one point.
        #   * EXACT DUPLICATES. The first copy lands copper; the second
        #     re-routes the now-joined pair, produces zero copper, and hits
        #     the same refusal. 51 of the 400 were duplicate copies.
        # Dropping them here -- the single point where oracle_links_by_net is
        # populated, so every consumer (SE loop, rip/retry, reroute) is
        # covered -- lands in the existing "malformed entries are dropped,
        # not fatal" contract: a net left with no usable link simply routes
        # the normal derivation way. Custody SCOPE is unaffected: the
        # finalize derives it from remaining_links' net names, not from this.
        _seen572 = set()
        _degen572 = _dup572 = 0
        for _ol in oracle_links:
            try:
                _onm, _oa, _ob = _ol[0], tuple(_ol[1]), tuple(_ol[2])
            except (TypeError, ValueError, IndexError):
                continue
            _onid = _name_to_id572.get(_onm)
            if _onid is not None and len(_oa) >= 4 and len(_ob) >= 4:
                if abs(_oa[0] - _ob[0]) < 1e-6 \
                        and abs(_oa[1] - _ob[1]) < 1e-6:
                    _degen572 += 1
                    continue
                _k572 = (_onid, round(_oa[0], 3), round(_oa[1], 3), _oa[2],
                         round(_ob[0], 3), round(_ob[1], 3), _ob[2])
                if _k572 in _seen572:
                    _dup572 += 1
                    continue
                _seen572.add(_k572)
                state.oracle_links_by_net.setdefault(_onid, []).append(
                    (_oa, _ob))
        if _degen572 or _dup572:
            print(f"  Oracle forced links (#572): dropped {_degen572} "
                  f"zero-length and {_dup572} duplicate link(s) -- neither "
                  f"can weld copper, and either aborts its net's remaining "
                  f"forced edges")
        if state.oracle_links_by_net:
            print(f"  Oracle forced links (#572): "
                  f"{sum(len(v) for v in state.oracle_links_by_net.values())}"
                  f" exact-fill link(s) on "
                  f"{', '.join(sorted(pcb_data.nets[i].name for i in state.oracle_links_by_net))}"
                  f" -- routing the oracle's own endpoints, not derived ones")

    # #540 item 2: price the outer pass's vacated corridors in every search
    # of this batch. The entries ride the existing ripped-route avoidance
    # dicts (filter_ripped_ghosts -> the composition passes), so per-net maps, rip-up retries and
    # reroute rounds all see them; the owners never route here, so the
    # routed_net_ids skip never drops them and no net repels itself.
    if external_ripped_ghosts and config.ripped_route_avoidance_cost > 0:
        from obstacle_costs import compute_ripped_route_costs as _crrc540
        _gcells = 0
        for _gnid, _gres in sorted(external_ripped_ghosts.items()):
            _glc, _gvp = _crrc540(_gres, config, layer_map)
            if len(_glc):
                state.ripped_route_layer_costs[_gnid] = _glc
                _gcells += len(_glc)
            if _gvp:
                state.ripped_route_via_positions[_gnid] = _gvp
        print(f"  (#540 ghosts: pricing {len(external_ripped_ghosts)} pending "
              f"casualty corridor(s), {_gcells} cost cells at "
              f"{config.ripped_route_avoidance_cost}mm)")

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

    # Plane-fragility soft costs (#424 planes-first): near-boundary pour
    # cells (of the EXACT fill when the board file is reachable) cost extra
    # so signals avoid BISECTING a plane at its necks. DEFAULT ON at 2.0
    # mm-equiv (KICAD_PLANE_FRAGILITY_COST=0 reverts); inert on boards
    # with no zones.
    from plane_fragility import register_plane_fragility
    register_plane_fragility(pcb_data, config, track_proximity_cache)

    # Congestion v2 (#424): demand/capacity bins + owner terminals; per-net
    # stamping happens at prepare (routing_context.stamp_congestion2).
    # #589 'c2': when the global plan ran, its predicted corridors seed the
    # demand map (rough paths ARE the demand map), replacing the
    # endpoints-only estimate. No-op unless KICAD_CONGESTION2_COST > 0.
    from congestion_field import build_congestion2
    _gp_c2 = (_gplan.demand_points(config)
              if (_gplan is not None and env_knobs.GLOBAL_PLAN['c2'])
              else None)
    config._congestion2 = build_congestion2(pcb_data, config,
                                            list(all_net_ids_to_route),
                                            extra_demand_points=_gp_c2)

    # History congestion (#590): fresh per-cell conflict field for this call
    # (rips + failed frontiers bump it; every prepare prices it). Env-gated
    # off by default.
    from history_congestion import reset_history
    reset_history(config)

    # Register rippable pre-existing nets as already-routed (issue #103):
    # blocking analysis iterates routed_net_paths (cells are recomputed from
    # pcb_data, so an empty path is fine), filter_rippable_blockers requires
    # routed_results membership, and rip_up_net removes exactly the
    # 'new_segments'/'new_vias' listed - the net's own file copper here.
    # One bucketing pass, not a per-net scan: main-pass pre-existing rip
    # candidacy can register hundreds of nets, and the old per-net list
    # comprehensions were O(nets x segments).
    _reg_segs: Dict[int, list] = {}
    _reg_vias: Dict[int, list] = {}
    if existing_rippable:
        _reg_set = set(existing_rippable)
        for _s in pcb_data.segments:
            if _s.net_id in _reg_set:
                _reg_segs.setdefault(_s.net_id, []).append(_s)
        for _v in pcb_data.vias:
            if _v.net_id in _reg_set:
                _reg_vias.setdefault(_v.net_id, []).append(_v)
    # #734: remember WHICH nets are mere rip candidates -- the audit gate
    # below must not depend on the is_existing_route flag surviving in
    # routed_results (later passes replace result dicts), so it keys on
    # this id set instead.
    state.preexisting_registered = set(existing_rippable)
    for nid in existing_rippable:
        state.routed_net_ids.append(nid)
        state.routed_net_paths[nid] = []
        state.routed_results[nid] = {
            'net_id': nid,
            'new_segments': _reg_segs.get(nid, []),
            'new_vias': _reg_vias.get(nid, []),
            'iterations': 0,
            'is_existing_route': True,
        }
        # #585 item 3a (env-gated): pre-existing copper emits track-proximity
        # ghosts like copper routed this run, so an already-routed board
        # exerts the same corridor-spreading pressure on new routes. Only
        # meaningful when track_proximity_cost > 0; rip-up removes the entry
        # exactly like a this-run net's.
        if env_knobs.TRACK_PROX_PREEXISTING and \
                config.track_proximity_cost > 0 and _reg_segs.get(nid):
            track_proximity_cache[nid] = compute_track_proximity_for_net(
                pcb_data, nid, config, layer_map)
    ripup_success_pairs = state.ripup_success_pairs
    rerouted_pairs = state.rerouted_pairs
    remaining_net_ids = state.remaining_net_ids
    results = state.results

    # Counters (kept as locals, not aliased from state)
    route_index = 0

    # Route single-ended nets
    se_successful, se_failed, se_time, se_iterations, route_index, user_quit = route_single_ended_nets(
        state, single_ended_nets,
        route_index_start=route_index,
        cancel_check=cancel_check, progress_callback=progress_callback
    )
    successful += se_successful
    failed += se_failed
    total_time += se_time
    total_iterations += se_iterations

    # Checkpoint abort (KICAD_STOP_AFTER / KICAD_STOP_FILE, set by the SE
    # loop): skip every later routing/repair pass and fall through to the
    # WRITE, so the partial board ships exactly as it stood at the stop.
    _ckpt_stop = getattr(state, 'checkpoint_stop', False)
    if _ckpt_stop:
        print("Checkpoint stop: skipping reroute/length-match/Phase 3/rescue/"
              "cleanup/reconciliation; writing the board as-is")

    # Run reroute loop for nets that were ripped during diff pair or single-ended routing
    rq_successful, rq_failed, rq_time, rq_iterations, route_index = (0, 0, 0.0, 0, route_index) if _ckpt_stop else run_reroute_loop(
        state, route_index_start=route_index,
        cancel_check=cancel_check, progress_callback=progress_callback,
        failed_so_far=failed
    )
    successful += rq_successful
    failed += rq_failed
    total_time += rq_time
    total_iterations += rq_iterations

    # Apply length matching if configured
    if length_match_groups and not _ckpt_stop:
        run_length_matching(routed_results, length_match_groups, config, pcb_data)

    # Sync pcb_data with length-matched segments before Phase 3
    # This ensures tap routes see meanders from other nets as obstacles
    if progress_callback:
        progress_callback(0, 0, "Syncing pcb_data...")
    sync_pcb_data_segments(pcb_data, routed_results, original_segment_ids, state, config)

    # Phase 3: Complete multi-point routing (tap connections)
    # This happens AFTER length matching so tap routes connect to meandered main routes
    num_multipoint_nets = len(state.pending_multipoint_nets) if state.pending_multipoint_nets else 0

    # Create phase 3 progress callback
    def phase3_progress_callback(current, total, net_name):
        if progress_callback:
            progress_callback(current, num_multipoint_nets, f"Multi-point: {net_name}")

    if not _ckpt_stop:
      run_phase3_tap_routing(
        state=state,
        pcb_data=pcb_data,
        config=config,
        base_obstacles=base_obstacles,
        gnd_net_id=None,  # Not used for single-ended
        all_unrouted_net_ids=all_unrouted_net_ids,
        routed_net_ids=routed_net_ids,
        remaining_net_ids=remaining_net_ids,
        routed_net_paths=routed_net_paths,
        routed_results=routed_results,
        diff_pair_by_net_id=state.diff_pair_by_net_id,  # Empty for single-ended
        results=results,
        track_proximity_cache=track_proximity_cache,
        layer_map=layer_map,
        progress_callback=phase3_progress_callback,
        cancel_check=cancel_check,
    )

    # Issue #134: nets whose stale copper would have shorted another net on
    # restore were left ripped instead of re-added (collision-safe restore).
    # Now that the board is stable, give them one clean reroute pass so the fix
    # does not cost completion. Only runs when a refusal actually happened, so
    # boards without the collision are unaffected.
    if state.collision_refused_net_ids:
        recover = []
        for nid in sorted(state.collision_refused_net_ids):
            if nid in routed_results or nid not in pcb_data.nets:
                continue
            if len(pcb_data.pads_by_net.get(nid, [])) < 2:
                continue
            if nid in state.diff_pair_by_net_id:
                pair_name, pair = state.diff_pair_by_net_id[nid]
                if (pair.p_net_id in state.queued_net_ids
                        or pair.n_net_id in state.queued_net_ids):
                    continue
                state.reroute_queue.append(('diff_pair', pair_name, pair))
                state.queued_net_ids.add(pair.p_net_id)
                state.queued_net_ids.add(pair.n_net_id)
                recover.append(pair_name)
            else:
                if nid in state.queued_net_ids:
                    continue
                state.reroute_queue.append(('single', pcb_data.nets[nid].name, nid))
                state.queued_net_ids.add(nid)
                recover.append(pcb_data.nets[nid].name)
        if recover and not _ckpt_stop:
            print(f"Issue #134 recovery: re-routing {len(recover)} net(s) left ripped "
                  f"to avoid a short: {', '.join(recover)}")
            run_reroute_loop(state, route_index_start=route_index,
                             cancel_check=cancel_check,
                             progress_callback=progress_callback)
        # Last resort (parity with the plane tools' piece-level settle,
        # 72ca5f9): a refused net whose clean reroute ALSO failed ships at
        # zero copper -- restore the saved route's non-colliding pieces
        # instead, leaving a small gap for a later pass rather than a
        # destroyed net. Piece-wise _saved_route_collides against the settled
        # board, so nothing restored can short copper routed meanwhile.
        _stash_134 = getattr(pcb_data, '_refused_saved_134', {}) or {}
        for nid in sorted(state.collision_refused_net_ids):
            if nid in routed_results or nid not in _stash_134:
                continue
            saved = _stash_134[nid]
            from rip_up_reroute import _saved_route_collides
            from routing_context import add_route_to_pcb_data
            keep_segs = [sg for sg in (saved.get('new_segments') or [])
                         if not _saved_route_collides(
                             {'new_segments': [sg], 'new_vias': []},
                             pcb_data, [nid], config.clearance)]
            keep_vias = [v for v in (saved.get('new_vias') or [])
                         if not _saved_route_collides(
                             {'new_segments': [], 'new_vias': [v]},
                             pcb_data, [nid], config.clearance)]
            from pcb_modification import drop_orphan_restore_pieces
            drop_orphan_restore_pieces(keep_segs, keep_vias, nid, pcb_data)
            if not keep_segs and not keep_vias:
                continue
            dropped = (len(saved.get('new_segments') or []) - len(keep_segs)
                       + len(saved.get('new_vias') or []) - len(keep_vias))
            pruned = dict(saved)
            pruned['new_segments'] = keep_segs
            pruned['new_vias'] = keep_vias
            pruned['partial_restore_134'] = True
            add_route_to_pcb_data(pcb_data, pruned, debug_lines=config.debug_lines)
            results.append(pruned)
            # #508 finding 8: register the restore as the net's AUTHORITATIVE
            # result, or the #87 superseded-result filter (`_authoritative`,
            # identity-keyed off routed_results) silently drops it from the
            # write-list on EVERY firing -- the copper landed in pcb_data,
            # the log said "restored", and the file shipped none of it (the
            # exact trap net_rescue.py's partial branch documents; evidence
            # runs_set14/rusefi_alphax4/step2b_retry.log). The entry
            # condition guarantees the net has no other result.
            routed_results[nid] = pruned
            nm = pcb_data.nets[nid].name if nid in pcb_data.nets else nid
            print(f"Issue #134 last resort: {nm} reroute failed; restored "
                  f"{len(keep_segs)} segment(s) + {len(keep_vias)} via(s) of its "
                  f"pre-rip route (dropped {dropped} colliding piece(s)); "
                  f"net remains PARTIAL for a later reconnect pass")

    # T5 zero-copper custody: casualties-only final reconciliation (route.py
    # front; parity with batch_route_diff_pairs' 43e6d10). A net RIPPED during
    # this run whose queued reroute never landed used to ship at ZERO copper
    # with no custody -- the #134 recovery above only covers nets whose RESTORE
    # was refused, not nets that were never restored at all (the ulx3s
    # GN8/GP2/GN22 class: pre-existing rippable nets ripped by the rip-up
    # ladder, reroute failed, stale-strip then removed their input copper from
    # the output). Restore-first semantics, collision-aware (never leaves
    # restored copper colliding with newly routed copper); honest 'unrecovered'
    # when nothing safe can be re-instated. Engine-side, so the GUI inherits.
    from diff_pair_custody import run_casualty_reconcile
    casualty_summary = None if _ckpt_stop else run_casualty_reconcile(
        state, progress_callback=progress_callback, cancel_check=cancel_check)

    # Issues #331/#371: last-chance scoped fine-parameter rescue for nets the
    # whole pipeline (main loop, rip-up ladder, reroute loop, Phase 3, #134
    # recovery) still left failed or partially connected. No rip-up and no
    # flags - scoped windows at finer grid/track/clearance only (net_rescue).
    # Runs BEFORE the summary counts so recovered nets grade as routed, and
    # before the cleanup pipeline so rescue copper is swept like all other
    # copper. KICAD_NET_RESCUE=0 disables it for A/B debugging.
    if progress_callback:
        progress_callback(0, 0, "Rescuing failed nets...")
    from net_rescue import rescue_failed_nets
    rescue_summary = None if _ckpt_stop else rescue_failed_nets(
        state, single_ended_nets, net_clearances=net_clearances,
        progress_callback=progress_callback, cancel_check=cancel_check)

    # Terminal geometry escalation ("better than shipping opens",
    # 2026-08-05): whole-net retry with track width + via size marching down
    # together toward the fab floor for every net the rescue still left
    # failed or open. Runs immediately after the rescue and BEFORE the
    # cleanup pipeline, deliberately: cleanup must operate on the FINAL
    # copper, so escalation copper is itself swept (stub gaps closed, dead
    # ends trimmed) like all other copper. Replaces the removed mid-retry
    # via rung (see single_ended_routing). KICAD_TERMINAL_ESCALATION=0
    # disables. Engine-side, so the GUI inherits it.
    from net_rescue import terminal_geometry_escalation
    # Candidacy scope: the run's own nets PLUS its pre-existing RIP VICTIMS.
    # A victim outside single_ended_nets whose reroute/restore landed
    # imperfectly ships open with nobody to catch it -- the escalation's
    # sweep re-derives openness authoritatively, so healthy victims cost
    # one connectivity check and are skipped. This also covers the final
    # reconciliation: it recurses into batch_route, so ITS victims meet
    # ITS escalation the same way (the "opens emerged after the escalation"
    # class from the term_ecp5 study -- victims were invisible, not late).
    _esc_scope = list(single_ended_nets)
    _esc_seen = {n for n, _i in _esc_scope}
    for _vn in (getattr(pcb_data, '_preexisting_rips', None) or {}):
        if _vn in _esc_seen:
            continue
        _vid = next((i for i, nn in pcb_data.nets.items() if nn.name == _vn),
                    None)
        if _vid is not None:
            _esc_scope.append((_vn, _vid))
    terminal_escalation_summary = None if _ckpt_stop else \
        terminal_geometry_escalation(
            state, _esc_scope, net_clearances=net_clearances,
            progress_callback=progress_callback, cancel_check=cancel_check)

    # Main-pass pre-existing rips (0805): pull every RIPPED pre-existing net
    # into the stale-strip scope -- a rerouted victim's ORIGINAL input copper
    # must not ship next to its reroute (#220/#300 class) -- and report each
    # rip's outcome on one line. Never-ripped auto-candidates deliberately
    # stay OUT of the sweep scope: cleanup must not touch nets this run
    # never disturbed.
    _pe_ripped_reg = getattr(pcb_data, '_preexisting_rips', None) or {}
    _pe_outcomes: Dict[str, str] = {}
    _victim_retry_names: List[str] = []
    if _pe_ripped_reg:
        sweep_scope_ids |= set(_pe_ripped_reg)
        # No silent casualties: a pre-existing victim whose reroute landed
        # only PARTIALLY (pads still disconnected) is a casualty the
        # zero-copper custody cannot see -- the kept partial result masks it
        # (sechzig /DRAM_VDD: VDDQ ripped it, VDD rerouted 1/16 pad pairs,
        # and the run would have shipped 15 open pads). Restore-first: when
        # the ORIGINAL copper can go back collision-free, rip the partial
        # reroute and restore the original; when the attacker took the
        # corridor, keep the partial and report honestly (the reconciliation
        # still gets a shot at it).
        from check_connected import check_net_connectivity as _pe_cnc

        def _pe_connected(_rid: int) -> bool:
            _r = _pe_cnc(
                _rid,
                [s for s in pcb_data.segments if s.net_id == _rid],
                [v for v in pcb_data.vias if v.net_id == _rid],
                pcb_data.pads_by_net.get(_rid, []),
                [z for z in pcb_data.zones if z.net_id == _rid])
            return bool(_r.get('connected'))
        if not _ckpt_stop:
            from rip_up_reroute import (rip_up_net as _pe_rip,
                                        restore_net as _pe_restore,
                                        _saved_route_collides as _pe_collides)
            for _rid in sorted(_pe_ripped_reg):
                _r_pe = routed_results.get(_rid)
                if _r_pe is None or _r_pe.get('is_existing_route'):
                    continue  # zero-copper custody / already-restored classes
                _orig_pe = (getattr(pcb_data, '_rip_saved', None)
                            or {}).get(_rid)
                if not _orig_pe or not (_orig_pe[0]
                                        or {}).get('is_existing_route'):
                    continue  # original payload not recoverable (re-rip chain)
                if _pe_connected(_rid):
                    continue  # reroute genuinely landed
                if _pe_collides(_orig_pe[0], pcb_data, [_rid],
                                config.clearance):
                    continue  # corridor taken; keep partial, report below
                _, _, _wir_par = _pe_rip(
                    _rid, pcb_data, routed_net_ids, routed_net_paths,
                    routed_results, state.diff_pair_by_net_id,
                    remaining_net_ids, results, config,
                    track_proximity_cache, state.working_obstacles,
                    state.net_obstacles_cache,
                    state.ripped_route_layer_costs,
                    state.ripped_route_via_positions, layer_map)
                if _wir_par:
                    successful -= 1
                _pe_restore(
                    _rid, _orig_pe[0], list(_orig_pe[1]), _orig_pe[2],
                    pcb_data, routed_net_ids, routed_net_paths,
                    routed_results, state.diff_pair_by_net_id,
                    remaining_net_ids, results, config,
                    track_proximity_cache, layer_map,
                    state.working_obstacles, state.net_obstacles_cache,
                    state.ripped_route_layer_costs,
                    state.ripped_route_via_positions,
                    refused_sink=state.collision_refused_net_ids)
                print(f"  Pre-existing victim '{_pe_ripped_reg[_rid]}': "
                      f"partial reroute left pads disconnected -- original "
                      f"copper restored (no silent casualties)")
        print(f"\nPre-existing rip outcome(s) ({len(_pe_ripped_reg)} net(s)):")
        for _rid in sorted(_pe_ripped_reg):
            _rname = _pe_ripped_reg[_rid]
            _blocked_for = None
            for _ev in reversed(state.net_history.get(_rid, []) or []):
                if _ev.get('event') == 'ripped_by':
                    _blocked_for = (_ev.get('details')
                                    or {}).get('ripping_net_name')
                    break
            _r_pe = routed_results.get(_rid)
            if _r_pe is None:
                _out_pe = ('NOT RECOVERED (still open; see '
                           'terminal_restores/reconciliation)')
            elif _r_pe.get('is_existing_route'):
                _out_pe = 'original copper restored'
            elif not _pe_connected(_rid):
                # Live connectivity, deliberately NOT failed_pads_info: the
                # end-of-run sweep has not attached that yet, and a phase-3
                # victim reroute's own tally is a main-edge proxy that can
                # read "routed" while pads stay open (sechzig /DRAM_VDD).
                _out_pe = ('rerouted PARTIAL (pads still disconnected; '
                           'original corridor taken -- reconciliation '
                           'retries it)')
            else:
                _out_pe = 'rerouted'
            _pe_outcomes[_rname] = _out_pe + (
                f" [was blocking '{_blocked_for}']" if _blocked_for else "")
            print(f"  '{_rname}'"
                  + (f" (was blocking '{_blocked_for}')" if _blocked_for else "")
                  + f": {_out_pe}")
            # Unrecovered victims become FIRST-CLASS reconcile targets.
            # Restore is collision-gated (a taken corridor makes it
            # illegal), so the only legal recovery is a REROUTE with full
            # router power -- the final reconciliation. Without this
            # wiring a victim with no result and no legal restore was
            # NOBODY'S responsibility and shipped open with an honest log
            # line (term2_ecp5: +1V1, /PF37+, /PE26+ all ended this way).
            if _r_pe is None or not _pe_connected(_rid):
                _victim_retry_names.append(_rname)

    # Final progress update
    if progress_callback:
        progress_callback(total_routes, total_routes, "Routing complete")

    # Build summary data
    import json
    # routed_single / failed_single are derived below, AFTER the authoritative
    # connectivity sweep -- a net with a result in routed_results may still have
    # left pads disconnected (a failed MST edge), and must not be reported routed
    # (issue #189). failed_single stays "no result at all"; the partially-routed
    # disconnected nets are surfaced via failed_multipoint (and mp_deficit).

    # Keep only each net's authoritative result in the write-list. routed_results
    # holds one result per net; rip-reroute paths (restore_net, layer-swap) can
    # leave a superseded duplicate of a net's result still in `results`, and its
    # copper is then written alongside the authoritative one -- stacking coincident
    # same-net vias (#87). Dropping by identity is safe now that the Phase-3 commit
    # reconciles each net's result against its actual copper, so the authoritative
    # result is the complete one (a dropped tap is flagged and re-routed, not lost).
    _authoritative = {id(r) for r in routed_results.values()}
    _stale = [r for r in results if id(r) not in _authoritative]
    if _stale:
        results[:] = [r for r in results if id(r) in _authoritative]
        print(f"Dropped {len(_stale)} superseded rip-reroute result(s) from the write-list")

    # --force-reroute custody: a force-ripped net whose replan landed NO new
    # copper gets its ORIGINAL copper back. Restore re-adds the SAVED objects
    # (identity-preserved), so the freeze snapshot below includes them and the
    # #220/#284 stale strip keeps them -- exactly the rip/restore contract
    # _compute_stale_input_copper documents. Without this, a failed replan
    # would strip a working route from the output in exchange for nothing.
    # Partial replans (some new copper) keep the new copper and report through
    # the normal failed-nets summary instead: restoring originals on top of a
    # partial reroute would stack stale copper the replan deliberately left.
    if force_ripped:
        _fr_new_copper = set()
        for _r in results:
            for _s in _r.get('new_segments') or []:
                _fr_new_copper.add(_s.net_id)
            for _v in _r.get('new_vias') or []:
                _fr_new_copper.add(_v.net_id)
        _fr_restored = []
        for _nid, (_segs, _vias) in force_ripped.items():
            if _nid in _fr_new_copper:
                continue
            pcb_data.segments = list(pcb_data.segments) + _segs
            pcb_data.vias = list(pcb_data.vias) + _vias
            _fr_restored.append(pcb_data.nets[_nid].name
                                if _nid in pcb_data.nets else str(_nid))
        if _fr_restored:
            print(f"--force-reroute: replan produced no copper for "
                  f"{len(_fr_restored)} net(s); ORIGINAL copper restored: "
                  f"{', '.join(_fr_restored[:6])}"
                  f"{', ...' if len(_fr_restored) > 6 else ''}")

    # ---- Issue #209 fix C: catch cleanup passes that disconnect a completed route ----
    # Snapshot each in-scope multi-pad net's connectivity on the WRITE-LIST copper
    # (original + routed new copper -- the same basis the output is written from)
    # BEFORE the post-routing cleanup passes (snap / phantom / cycle-prune /
    # dead-end sweep / neck). If a pass turns a connected net (or pad) disconnected,
    # that is a cleanup bug -- surface it loudly and in the JSON summary rather than
    # ship a silently-broken net (free_dap +3V3 IC2.13).
    from check_connected import check_net_connectivity as _cnc209

    # Gate basis: ON-BOARD originals at gate-setup time (routing is done, so
    # pcb_data already reflects rip-and-not-restored losses). Using the
    # pristine input snapshot here mis-attributed ROUTING losses to cleanup:
    # a ripped-and-lost net's original copper made the PRE check read
    # "connected", the stale strip then truthfully removed the ghost copper
    # from the file, and the POST check blamed the cleanup pass (smartknob
    # +5V: "cleanup DISCONNECTED ... dropped 36 segment(s)" on every wave,
    # while the disconnect was really a routing failure).
    _gate_seg_by_net: Dict[int, list] = {}
    for _s0 in pcb_data.segments:
        if id(_s0) in original_segment_ids:
            _gate_seg_by_net.setdefault(_s0.net_id, []).append(_s0)
    _gate_orig_via_ids = {id(v) for lst in _orig_via_by_net.values() for v in lst}
    _gate_via_by_net: Dict[int, list] = {}
    for _v0 in pcb_data.vias:
        if id(_v0) in _gate_orig_via_ids:
            _gate_via_by_net.setdefault(_v0.net_id, []).append(_v0)
    # Board membership for RESULTS copper at gate-setup time: a result can
    # still reference copper a later net's rip-up removed from the board (the
    # phantom drop reconciles that inside the pipeline, AFTER the pre-check).
    # Counting it inflated the PRE connectivity, so a net routing honestly
    # failed (smartknob +5V, 5 failed multipoint pads) was re-blamed on
    # cleanup when the phantom drop removed the ghost copper from the
    # write-list.
    _gate_board_seg_ids = {id(s) for s in pcb_data.segments}
    _gate_board_via_ids = {id(v) for v in pcb_data.vias}
    # ...but only SETUP-ERA results copper needs board membership: cleanup
    # passes legitimately ADD copper to results afterwards (snap connectors,
    # octolinear/microshift replacements, soft-joint bridges) which is not in
    # the setup snapshot -- filtering those out made every repaired net grade
    # "disconnected" in the post check (false +5V//STRAIN_S- reports).
    _gate_setup_result_seg_ids = {id(s) for r in results
                                  for s in (r.get('new_segments') or [])}
    _gate_setup_result_via_ids = {id(v) for r in results
                                  for v in (r.get('new_vias') or [])}

    def _writelist_copper_209(strip_seg_ids=frozenset(), strip_via_ids=frozenset()):
        # strip_*_ids: original input copper the writer will DELETE from its
        # verbatim copy (cleanup strip lists + the #220/#284 stale strips).
        # The post-cleanup check must exclude it, or a strip that disconnects a
        # net is invisible here -- the pre-snapshot passes empty sets (nothing
        # stripped yet), so pre and post grade the same write model.
        _s = {nid: [s for s in lst if id(s) not in strip_seg_ids]
              for nid, lst in _gate_seg_by_net.items()}
        _v = {nid: [v for v in lst if id(v) not in strip_via_ids]
              for nid, lst in _gate_via_by_net.items()}
        for _r in results:
            for _seg in _r.get('new_segments') or []:
                if (id(_seg) not in _gate_setup_result_seg_ids
                        or id(_seg) in _gate_board_seg_ids):
                    _s.setdefault(_seg.net_id, []).append(_seg)
            for _via in _r.get('new_vias') or []:
                if (id(_via) not in _gate_setup_result_via_ids
                        or id(_via) in _gate_board_via_ids):
                    _v.setdefault(_via.net_id, []).append(_via)
        # Stub layer-swap vias are written to the output but live only in
        # all_swap_vias (not in any result's new_vias, and the per-net snapshot
        # predates them). Without them every layer-swapped stub looks severed
        # from its pad and the net reports a phantom disconnection (issue #292:
        # nebula_watch's 14 layer swaps == its 14 "failed" multi-point nets).
        for _via in all_swap_vias:
            _v.setdefault(_via.net_id, []).append(_via)
        # Swap reuse-connector SEGMENTS ride all_swap_segments the same way
        # (#508 finding 13): written to the output, in no result's
        # new_segments -- omitting them severs the swapped stub from its pad
        # in this gate's model and manufactures phantom disconnects.
        for _seg in all_swap_segments:
            _s.setdefault(_seg.net_id, []).append(_seg)
        return _s, _v

    def _seg_sig_209(s):
        a, b = (round(s.start_x, 3), round(s.start_y, 3)), (round(s.end_x, 3), round(s.end_y, 3))
        return (min(a, b), max(a, b), s.layer)

    _zbn_209 = {}
    for _z in getattr(pcb_data, 'zones', []) or []:
        _zbn_209.setdefault(_z.net_id, []).append(_z)
    _pre_s_209, _pre_v_209 = _writelist_copper_209()
    _pre_conn_209 = {}
    for _nid in sweep_scope_ids:
        _pads209 = pcb_data.pads_by_net.get(_nid, [])
        if len(_pads209) < 2:
            continue
        _r209 = _cnc209(_nid, _pre_s_209.get(_nid, []), _pre_v_209.get(_nid, []),
                        _pads209, _zbn_209.get(_nid, []), tolerance=0.02)
        _pre_conn_209[_nid] = (
            bool(_r209.get('connected')),
            len(_r209.get('disconnected_pads') or []),
            {_seg_sig_209(s) for s in _pre_s_209.get(_nid, [])},
            {(round(v.x, 3), round(v.y, 3)) for v in _pre_v_209.get(_nid, [])},
        )

    # ---- Post-route cleanup: the ONE shared pipeline (#319 restructure) ----
    # All passes run inside run_post_route_cleanup in their canonical order
    # (see cleanup_pipeline.py), under the uniform contract that pcb_data is
    # mutated in lockstep with the write-list -- pcb_data IS the board that
    # will be written at every point after a pass.
    #
    # The freeze hook fires after the phantom drop, when the board holds
    # exactly what ROUTING committed (rips applied, write-list reconciled) and
    # no cleanup pass has trimmed it yet. The #220/#284 stale-input strip below
    # must reference THAT board -- the rip signal -- not one a cleanup pass has
    # since trimmed: a cleanup pass's own removals are already tracked in the
    # pipeline's strip list, and feeding them into the stale strip too made it
    # over-remove LOAD-BEARING input copper (the glasgow_revC B1 regression:
    # 187 vs 28 stale strips, 6 nets dropped). copy.copy each object, not just
    # the list: a list() alone is immune to list REBINDS but still shares
    # objects, so an in-place field move (nudge_grazing_vias shifts a this-run
    # via's x,y in place) would leak into the snapshot's signatures.
    _committed_segments: list = []
    _committed_vias: list = []
    _committed_seg_ids: set = set()
    _committed_via_ids: set = set()

    def _freeze_committed():
        _committed_segments.extend(copy.copy(s) for s in pcb_data.segments)
        _committed_vias.extend(copy.copy(v) for v in pcb_data.vias)
        # Identity of the LIVE objects at freeze time: the stale strip's
        # "original still on the committed board" test is by id(), not
        # signature -- a routed twin re-created on the byte-identical span
        # must not shield a ripped original from the strip (twin-shielding
        # leak, see _compute_stale_input_copper).
        _committed_seg_ids.update(id(s) for s in pcb_data.segments)
        _committed_via_ids.update(id(v) for v in pcb_data.vias)

    # Checkpoint stop skips the cleanup pipeline, but the FREEZE must still
    # happen: the #220/#284 stale-input strip below grades any input copper
    # absent from the committed snapshot as stale -- with an empty snapshot
    # it stripped EVERY routed net's input copper from the written file
    # (checkpoint boards shipped bus nets consisting only of their new tap
    # segments; 31-segment trunks vanished). The board at this point IS what
    # routing committed, which is exactly the freeze semantics.
    # KICAD_STOP_CLEANUP=1: run the cleanup pipeline ON the checkpoint
    # snapshot (dead-end sweep, grazes, etc.) before writing -- shows what
    # cleanup would retire (e.g. a stale escape stub the island-launch
    # route no longer uses). The freeze then fires inside the pipeline.
    _ckpt_cleanup = _ckpt_stop and env_knobs.STOP_CLEANUP
    if _ckpt_stop and not _ckpt_cleanup:
        _freeze_committed()
    # #473: nets still carrying unfinished pads keep ALL their copper
    # through the dead-end sweep -- their spurs are the landing sites the
    # next chain step / rescue needs, and sweeping them each step eroded
    # failed nets across chains.
    _protect_unfinished = {nid for nid, _r in routed_results.items()
                           if _r.get('failed_pads_info')}
    _protect_unfinished |= {nid for _nm, nid in single_ended_nets
                            if nid not in routed_results}
    _cleanup = None if (_ckpt_stop and not _ckpt_cleanup) else run_post_route_cleanup(
        results, pcb_data, sweep_scope_ids, config,
        protect_net_ids=_protect_unfinished,
        freeze_hook=_freeze_committed,
        # Lets the phantom step also remove ORPHAN routed copper from pcb_data
        # (rip/reroute slivers no surviving result references), so board ==
        # write model. Original input copper is identified by object identity
        # (the keepalive prevents id() recycling, see _original_segments_keepalive).
        # Stub layer-swap vias live in all_swap_vias (not in any result) but ARE
        # written to the output -- protect them like originals or the orphan
        # sweep would strip them from the board while the file keeps them.
        original_segment_ids=original_segment_ids,
        original_via_ids=({id(v) for lst in _orig_via_by_net.values() for v in lst}
                          | {id(v) for v in all_swap_vias}),
        keep_input_copper=keep_input_copper,
        progress_callback=progress_callback,
        # #536 octolinear smoothing: ON by default (restored).
        #
        # It was briefly defaulted OFF on the theory that smoothing mid-chain
        # starves a later step's rip-up/rescue of staircase slack -- two boards
        # (cubesat_backplane 10->2, spartan6_6layer 12->7) supported that. The
        # CORPUS refutes it: a direct A/B at one commit over 147 boards, with
        # only this knob differing, put smoothing ON at 129 incomplete nets and
        # OFF at 149 -- turning it off costs ~20, worsening 34 boards and helping
        # 10. Even spartan6, one of the two boards that motivated the change,
        # measures ON=6 OFF=9 there.
        #
        # So smoothing is not starving the later passes; on balance it helps
        # them, presumably by freeing corridor space. --no-smoothing disables it
        # per step; KICAD_SMOOTH_ROUTE=0/1 still overrides either way.
        smooth=smoothing)
    dead_end_input_segments = _cleanup.input_strip_segments if _cleanup is not None else []

    # Issue #220: the output writer copies the INPUT FILE verbatim, then adds the
    # write-list results and strips `segments_to_remove`. So an in-scope net's
    # original input copper survives to the output even after route.py ripped it
    # and did NOT restore/re-route it -- the net's authoritative final copper is
    # what's on the board (pcb_data). For a net routed-then-ripped (cparti +1V8),
    # that stale input copper not only lingers but can CROSS nets that routed into
    # its vacated corridor while it was ripped. Strip every in-scope net's original
    # input segment that is no longer on the final board. A segment route.py kept
    # is still on the frozen board (matched by OBJECT IDENTITY -- rip/restore
    # preserves it) and is NOT stripped, so legitimate copper is untouched.
    # Issue #284 (segment twin of the via strip below): also strip an original
    # segment the writer will RE-EMIT verbatim (same net, same span+layer, in
    # results.new_segments) -- otherwise a ripped/re-routed net that reproduces a
    # span exactly ships the verbatim original next to a byte-identical copy. This
    # is DRC-benign (same-net overlap is permitted) but keeps the output clean.
    # The strip references the FROZEN committed board (_committed_segments,
    # snapshotted by the pipeline's freeze hook), never live pcb_data: a
    # cleanup pass's removals must not masquerade as rip signals (B1).
    _emitted_segs = [_s for _r in results for _s in (_r.get('new_segments') or [])]
    _stale_input_segs = compute_stale_input_segments(
        _orig_seg_by_net, sweep_scope_ids, _committed_segments, _emitted_segs,
        final_ids=_committed_seg_ids)
    if _stale_input_segs:
        dead_end_input_segments = list(dead_end_input_segments) + _stale_input_segs
        print(f"Stripped {len(_stale_input_segs)} stale input segment(s) of "
              f"ripped/re-routed nets not on the final board (#220)")
    # Same for input VIAS (the #220 strip was segments-only): a ripped-existing
    # net's original vias otherwise ship next to its reroute's replacements as
    # same-net drill pairs.
    # Issue #284: strip an original input via of a ripped/re-routed net when it is
    # superseded on the final board (size/drill-aware) OR when the writer will
    # re-emit a via for the same net at the same position -- either way keeping
    # the verbatim original stacks two vias in one hole. The emitted-via set is
    # the writer's: each result's new_vias plus the stub layer-swap vias (same
    # basis as _writelist_copper_209 above).
    _emitted_vias = [_v for _r in results for _v in (_r.get('new_vias') or [])]
    _emitted_vias += list(all_swap_vias)
    stale_input_vias = compute_stale_input_vias(
        _orig_via_by_net, sweep_scope_ids, _committed_vias, _emitted_vias,
        final_ids=_committed_via_ids)
    # Orphan-island removals happen AFTER the freeze snapshot, so their vias
    # look 'committed' to the stale computation -- merge their strip list
    # explicitly (an island's barrel would otherwise ship floating).
    _oi_vias = getattr(_cleanup, 'input_strip_vias', None) or []
    if _oi_vias:
        _known = {id(v) for v in stale_input_vias}
        stale_input_vias = list(stale_input_vias) + \
            [v for v in _oi_vias if id(v) not in _known]
    if stale_input_vias:
        print(f"Stripping {len(stale_input_vias)} stale input via(s) of "
              f"ripped/re-routed nets not on the final board")

    # #508 finding 3: a COMMITTED tap relocation removed a PRE-EXISTING
    # plane-net via + stub from pcb_data. Plane nets are deliberately outside
    # sweep_scope_ids (never in routed_results), so neither strip computation
    # above can see the removal -- without this merge the writer re-emits the
    # removed via from the input text, under a net that has since routed
    # through the vacated pocket: a short in the file only. (The replacement
    # re-tap via ships via all_swap_vias.)
    if state.tap_relocation_removed_segments:
        _known_trs = {id(s) for s in dead_end_input_segments}
        dead_end_input_segments = list(dead_end_input_segments) + \
            [s for s in state.tap_relocation_removed_segments
             if id(s) not in _known_trs]
    if state.tap_relocation_removed_vias:
        _known_trv = {id(v) for v in stale_input_vias}
        stale_input_vias = list(stale_input_vias) + \
            [v for v in state.tap_relocation_removed_vias
             if id(v) not in _known_trv]
    if state.tap_relocation_removed_segments or state.tap_relocation_removed_vias:
        print(f"Stripping {len(state.tap_relocation_removed_segments)} "
              f"segment(s) + {len(state.tap_relocation_removed_vias)} via(s) "
              f"removed by committed tap relocations (#508)")

    # Uniform contract, stale-strip edition: the #284 re-emit clause can strip
    # an original that is STILL on the board (a routed twin reproduced its
    # span, so the file keeps only the emitted copy) -- mirror that removal
    # into pcb_data like every other subtractive step, or the board carries
    # copper the file won't have (ottercast Net-(C46-Pad1) sliver, found by
    # KICAD_BOARD_LEDGER). Identity-based; absent objects are a no-op.
    _stale_ids = {id(s) for s in _stale_input_segs}
    if _stale_ids:
        pcb_data.segments = [s for s in pcb_data.segments if id(s) not in _stale_ids]
    _stale_via_ids = {id(v) for v in stale_input_vias}
    if _stale_via_ids:
        pcb_data.vias = [v for v in pcb_data.vias if id(v) not in _stale_via_ids]

    # Board-vs-file ledger (KICAD_BOARD_LEDGER=1): audit the pipeline contract
    # now that every strip is known -- per in-scope net, pcb_data must equal
    # the write model (original input copper - strips + emitted results).
    verify_board_file_parity(
        pcb_data, sweep_scope_ids, _orig_seg_by_net, results,
        list(dead_end_input_segments) + list(_stale_input_segs),
        label=' route')

    # Issue #209 fix C: re-check the snapshotted nets against the post-cleanup
    # write-list and report any net a cleanup pass disconnected, listing the
    # dropped copper. A non-empty list here is a cleanup BUG (the routed net was
    # connected and a graph-preserving pass severed it), not a routing failure.
    # The post basis excludes everything the writer will STRIP (cleanup strip
    # lists + stale strips) -- otherwise a strip that disconnects a net is
    # invisible to this gate (the pre basis had no strips, so the comparison
    # stays like-for-like).
    cleanup_disconnected = []
    if _pre_conn_209:
        _post_s_209, _post_v_209 = _writelist_copper_209(
            strip_seg_ids={id(s) for s in dead_end_input_segments}
                          | {id(s) for s in _stale_input_segs},
            strip_via_ids={id(v) for v in stale_input_vias})
        for _nid, (_was_conn, _was_disc, _pre_segsig, _pre_viasig) in _pre_conn_209.items():
            _pads209 = pcb_data.pads_by_net.get(_nid, [])
            _r209 = _cnc209(_nid, _post_s_209.get(_nid, []), _post_v_209.get(_nid, []),
                            _pads209, _zbn_209.get(_nid, []), tolerance=0.02)
            _now_disc = len(_r209.get('disconnected_pads') or [])
            if not ((_was_conn and not _r209.get('connected')) or _now_disc > _was_disc):
                continue
            _net_name = pcb_data.nets[_nid].name if _nid in pcb_data.nets else f"Net {_nid}"
            _post_segsig = {_seg_sig_209(s) for s in _post_s_209.get(_nid, [])}
            _post_viasig = {(round(v.x, 3), round(v.y, 3)) for v in _post_v_209.get(_nid, [])}
            cleanup_disconnected.append({
                'net_name': _net_name,
                'net_id': _nid,
                'disconnected_pads': [
                    {'x': round(p[0], 3), 'y': round(p[1], 3),
                     'component_ref': p[3] if len(p) > 3 else '?'}
                    for p in (_r209.get('disconnected_pads') or [])],
                'dropped_segments': [
                    {'start': list(sa[0]), 'end': list(sa[1]), 'layer': sa[2]}
                    for sa in (_pre_segsig - _post_segsig)],
                'dropped_vias': [{'x': vx, 'y': vy} for (vx, vy) in (_pre_viasig - _post_viasig)],
            })
        if cleanup_disconnected:
            print(f"\n{RED}WARNING: post-routing cleanup DISCONNECTED "
                  f"{len(cleanup_disconnected)} completed route(s) "
                  f"(this is a cleanup bug, not a routing failure):{RESET}")
            for _cd in cleanup_disconnected:
                print(f"  {RED}{_cd['net_name']}: dropped "
                      f"{len(_cd['dropped_segments'])} segment(s) + "
                      f"{len(_cd['dropped_vias'])} via(s) -> "
                      f"{len(_cd['disconnected_pads'])} pad(s) now unconnected{RESET}")
                for _dv in _cd['dropped_vias']:
                    print(f"      dropped via @ ({_dv['x']:.3f}, {_dv['y']:.3f})")

    # ---- VIA DEDUP (run-7 E3) ----------------------------------------------
    # Failed/partial attempts can leave a result emitting a via at the exact
    # position of a surviving input via, or of another result's via for the
    # same net: identical coords, same span -- two barrels stacked in one
    # hole. KiCad tolerates same-net stacks, so no DRC flags them; they ship
    # silently and read as parser/net rot when the file is inspected later.
    # Drop the duplicates from the write list HERE, before total_vias and the
    # authoritative sweep, so the tally, the sweep, the writer and the GUI
    # results all see the same deduped copper. Same-net only (the key carries
    # net_id): cross-net coincidence is a real short the DRC sweep owns.
    # The key is EXACT, not a proximity bucket: this pass exists to drop
    # byte-identical re-emissions, and only those. Rounding to a coarse grid
    # (the original 0.01mm bucket) is not a tolerance -- it is a bucket, so it
    # both misses duplicates that straddle a bucket edge and can collapse two
    # DISTINCT vias that happen to share one. Quantize instead to the 0.1um
    # the writer itself emits (KiCad's 4-decimal mm text), which makes the key
    # exact for anything that would round-trip through the file identically.
    # drill/size are IN the key: two same-net barrels at one point with
    # different geometry are not interchangeable, so dropping either would be
    # a silent geometry change rather than a duplicate removal. Whatever this
    # pass declines to merge is still censused into summary['stacked_copper'],
    # which is the disclosure that actually matters (KiCad never flags a
    # same-net stack).
    _survive_via_strip = {id(v) for v in stale_input_vias}

    def _via_dedup_key(v):
        return (v.net_id, tuple(v.layers or ()),
                round(v.drill, 4), round(v.size, 4),
                round(v.x, 4), round(v.y, 4))

    _kept_via_keys = set()
    for _lst0 in _orig_via_by_net.values():
        for _v0 in _lst0:
            if id(_v0) not in _survive_via_strip:
                _kept_via_keys.add(_via_dedup_key(_v0))
    for _v0 in all_swap_vias:
        _kept_via_keys.add(_via_dedup_key(_v0))
    _via_dup_dropped = []
    for _r0 in results:
        _nv0 = _r0.get('new_vias') or []
        if not _nv0:
            continue
        _kept0 = []
        for _v0 in _nv0:
            _k0 = _via_dedup_key(_v0)
            if _k0 in _kept_via_keys:
                _via_dup_dropped.append(_v0)
                continue
            _kept_via_keys.add(_k0)
            _kept0.append(_v0)
        if len(_kept0) != len(_nv0):
            _r0['new_vias'] = _kept0
    if _via_dup_dropped:
        # Keep pcb_data == write model: remove the dropped OBJECTS from the
        # board too -- unless the same object also survives as a kept
        # occurrence (the same via listed by two results keeps its first).
        _kept_obj_ids = {id(v) for _r0 in results
                         for v in (_r0.get('new_vias') or [])}
        _drop_obj_ids = {id(v) for v in _via_dup_dropped} - _kept_obj_ids
        if _drop_obj_ids:
            pcb_data.vias = [v for v in pcb_data.vias
                             if id(v) not in _drop_obj_ids]
        print(f"Via dedup: dropped {len(_via_dup_dropped)} duplicate stacked "
              f"via(s) already present at the same position/span for the "
              f"same net")

    # Count total vias from results
    total_vias = sum(len(r.get('new_vias', [])) for r in results)

    # Issue #8: reconcile the reported success counts with the FINAL segment
    # graph -- i.e. the board that will actually be written. This runs AFTER the
    # stale-result drop, snap, phantom drop and dead-end sweep (and pcb_data was
    # synced to the write-list at the stale drop), so pcb_data now matches the
    # output. A net's pads are reconciled at Phase-3 commit, but its copper can
    # change afterwards (a later net's rip-up, the recovery reroute, a dropped
    # superseded result) and leave it split, or a multi-pad net may be routed
    # with a result that never tracked every pad -- either way it would ship as
    # phantom success (neo6502 /GPIO4, glasgow /IO_Banks/IO_Buffer_A/P1). Use the
    # AUTHORITATIVE union-find (check_net_connectivity -- the model
    # filter_already_routed and check_connected.py use); the stricter geometric
    # pad-group split wrongly splits genuinely-connected power/bus nets.
    from check_connected import (check_net_connectivity,
                                 net_break_within_outlines,
                                 net_pad_pairs_within_outlines)
    # Per-net copper as it will be WRITTEN: the input board's original copper
    # MINUS everything the writer strips (cleanup strip lists + the #220/#284
    # stale strips of ripped/re-routed nets) plus the write-list's new copper.
    # NOT pcb_data, which also holds orphan copper from rip/reroute that never
    # reaches the write-list (issue #8). Counting STRIPPED originals here (the
    # pre-fix behavior) graded ripped-and-stripped nets on ghost copper the
    # output file will not contain -- a rip victim whose reroute landed only
    # partially "passed" this sweep on its stripped pre-rip copper and shipped
    # broken with no failure record (T5 zero-copper custody, ulx3s GN12 class).
    _sweep_strip_seg_ids = ({id(s) for s in dead_end_input_segments}
                            | {id(s) for s in _stale_input_segs})
    _sweep_strip_via_ids = {id(v) for v in stale_input_vias}
    _segs_by_net: Dict[int, list] = {
        nid: [s for s in lst if id(s) not in _sweep_strip_seg_ids]
        for nid, lst in _orig_seg_by_net.items()}
    _vias_by_net: Dict[int, list] = {
        nid: [v for v in lst if id(v) not in _sweep_strip_via_ids]
        for nid, lst in _orig_via_by_net.items()}
    for _r0 in results:
        for _s in _r0.get('new_segments', []):
            _segs_by_net.setdefault(_s.net_id, []).append(_s)
        for _v in _r0.get('new_vias', []):
            _vias_by_net.setdefault(_v.net_id, []).append(_v)
    # Include the stub layer-swap vias the writer adds to the output (issue
    # #292): they are not in any result's new_vias, and the original-copper
    # snapshot predates them, so without this every layer-swapped net's pad
    # looks disconnected and ships as a phantom "failed multi-point" entry.
    for _v in all_swap_vias:
        _vias_by_net.setdefault(_v.net_id, []).append(_v)
    # Swap reuse-connector segments too (#508 finding 13): all_swap_segments
    # is written to the output but is in no result's new_segments, so
    # omitting it here severs layer-swapped stubs from their pads in this
    # sweep's model -- phantom failed_multipoint entries.
    for _s in all_swap_segments:
        _segs_by_net.setdefault(_s.net_id, []).append(_s)

    # ---- STACKED-COPPER SURFACING (run-7 E3) -------------------------------
    # The dedup above drops the attributable duplicates; census what is STILL
    # stacked on the write model (duplicate same-net segments, coincident
    # same-net vias) and surface it in the summary. check_drc stays silent on
    # same-net stacks by design (KiCad permits them), so without this the
    # only way to see one was to read the raw file.
    stacked_copper_findings = []
    try:
        from check_weird import stacked_copper_over_model
        stacked_copper_findings = stacked_copper_over_model(
            _segs_by_net, _vias_by_net,
            lambda _n: (pcb_data.nets[_n].name if _n in pcb_data.nets
                        else f"Net {_n}"))
    except Exception as _swe:
        print(f"  (stacked-copper check failed: {_swe})")
    if stacked_copper_findings:
        print(f"\n{RED}WARNING: {len(stacked_copper_findings)} stacked-copper "
              f"finding(s) on the write model (same-net duplicates KiCad "
              f"will not flag):{RESET}")
        for _f in stacked_copper_findings[:8]:
            print(f"  {RED}{_f['net']} {_f['layer']}: {_f['detail']}{RESET}")
        if len(stacked_copper_findings) > 8:
            print(f"  {RED}... and {len(stacked_copper_findings) - 8} more{RESET}")

    _zones_by_net: Dict[int, list] = {}
    for _z in getattr(pcb_data, 'zones', []) or []:
        _zones_by_net.setdefault(_z.net_id, []).append(_z)
    # (pairs_total, pairs_connected) per graded net, outline-aware -- raw
    # material for the pad-pair tallies emitted with the summary (#409
    # follow-up). Filled by the sweep below and by the straggler grading at
    # summary time.
    _pad_pair_stats: Dict[int, Tuple[int, int]] = {}
    # #734: nets this run actually DISTURBED -- the only pre-existing
    # candidates whose brokenness is this run's to report and to hand the
    # final reconciliation. Everything else with an is_existing_route
    # custody entry was merely rip-ELIGIBLE.
    _pe_reg734 = getattr(state, 'preexisting_registered', None) or set()
    _scope734 = ({nid for _nm, nid in single_ended_nets}
                 | set(all_net_ids_to_route or ()))
    _pe_disturbed_audit = (set(_pe_ripped_reg)
                           | set(getattr(state, 'casualty_custody', {}) or {})
                           | set(state.collision_refused_net_ids or set())
                           | {s.net_id for s in _stale_input_segs}
                           | {v.net_id for v in stale_input_vias}
                           | set(force_ripped or {})
                           | {nid for nid, v in
                              (getattr(state, 'terminal_restores', None)
                               or {}).items() if v != 'full'})
    if os.environ.get('KICAD_SCOPE_DEBUG'):
        _b734 = {
            'ripped_reg': set(_pe_ripped_reg),
            'casualty': set(getattr(state, 'casualty_custody', {}) or {}),
            'collision': set(state.collision_refused_net_ids or set()),
            'stale_segs': {s.net_id for s in _stale_input_segs},
            'stale_vias': {v.net_id for v in stale_input_vias},
            'force': set(force_ripped or {}),
            'trestore': {nid for nid, v in
                         (getattr(state, 'terminal_restores', None)
                          or {}).items() if v != 'full'},
        }
        for _bk, _bs in _b734.items():
            if _bs:
                _nmz = sorted((pcb_data.nets[n].name.split('/')[-1]
                               if n in pcb_data.nets else str(n))
                              for n in _bs)[:12]
                print(f"  [scope-dbg] disturbed[{_bk}] = {len(_bs)}: {_nmz}")
    for _nid, _res in routed_results.items():
        if _nid in state.diff_pair_by_net_id:
            continue  # diff pairs report via their own path
        _pads = pcb_data.pads_by_net.get(_nid, [])
        if len(_pads) < 2:
            continue
        # #734 reconcile scoping: every pre-existing rip CANDIDATE gets a
        # routed_results entry (custody registration, is_existing_route),
        # so on a narrow --nets run this audit minted failed_pads for the
        # WHOLE board's open stub nets and the final reconciliation then
        # routed all of them (measured: a 2-net run on a mid-chain board
        # routed 48 extras, with the reconcile pricing layers on its own
        # AUTO ladder -- different economics than the requested nets).
        # Candidacy makes a net rip-ELIGIBLE; responsibility transfers at
        # an ACTUAL rip. An is_existing_route entry whose net this run
        # never disturbed is neither this run's failure nor the
        # reconcile's to route: a net the run ripped is in the
        # _preexisting_rips registry (or its entry was replaced by the
        # rip/restore machinery, dropping the flag) and still audits.
        if os.environ.get('KICAD_SCOPE_DEBUG'):
            _nm734 = (pcb_data.nets[_nid].name.split('/')[-1]
                      if _nid in pcb_data.nets else str(_nid))
            print(f"  [scope-dbg] audit {_nm734}: registered="
                  f"{_nid in _pe_reg734} "
                  f"requested={_nid in _scope734} "
                  f"disturbed={_nid in _pe_disturbed_audit}")
        if (_nid in _pe_reg734 and _nid not in _scope734
                and _nid not in _pe_disturbed_audit):
            # Clear, don't just skip: an earlier stage may have stamped
            # failed_pads_info on this untouched candidate's entry, and
            # the failed_multipoint export below reads the flag verbatim.
            if _res.get('failed_pads_info'):
                _res['failed_pads_info'] = []
            continue
        _r = check_net_connectivity(
            _nid, _segs_by_net.get(_nid, []), _vias_by_net.get(_nid, []),
            _pads, _zones_by_net.get(_nid, []), tolerance=0.02,
            pcb_data=pcb_data)
        # #479 multi-board: pads split only ACROSS board outlines are not
        # failures (no copper can join them); keep within-outline breaks.
        _broken, _dp = net_break_within_outlines(pcb_data, _r)
        if _dp:
            _res['failed_pads_info'] = [
                {'x': _p[0], 'y': _p[1],
                 'component_ref': _p[3] if len(_p) > 3 else '?',
                 'pad_number': '?'}
                for _p in _dp]
        elif _res.get('failed_pads_info'):
            # Authoritatively connected now: drop a stale (stricter-model or
            # pre-rip) failure flag so the net is not reported as a phantom fail.
            _res['failed_pads_info'] = []
        # Issue #184: re-derive the multi-point pad counts from this same
        # authoritative union-find (which credits pads reached via planes/zones,
        # fanout stubs, and rip-up/retry reroutes), not the per-net MST edge tally
        # gathered during routing -- otherwise the headline under-reports
        # connectivity (e.g. upsy_desky 73/101) on boards check_connected.py
        # confirms fully connected, and the tap-retry loop chases already-connected
        # pads. Match check_connected.py semantics: count over all of the net's pads.
        if _res.get('is_multipoint'):
            _res['tap_pads_total'] = len(_pads)
            _res['tap_pads_connected'] = len(_pads) - len(_dp)
        # #409 follow-up: keep this net's outline-aware pair credit for the
        # pad-pair tallies (num_components is otherwise discarded by this
        # sweep). Cross-outline gaps are neither routable nor open (#479).
        _pad_pair_stats[_nid] = net_pad_pairs_within_outlines(
            pcb_data, _r, _pads)

    # Collect multi-point tap routing stats and failed pad details
    tap_pads_connected = 0
    tap_pads_total = 0
    tap_edges_routed = 0
    tap_edges_failed = 0
    multipoint_nets = 0
    failed_multipoint = []  # List of {net_name, net_id, failed_pads: [...]}
    for net_id, result in routed_results.items():
        if result.get('is_multipoint'):
            multipoint_nets += 1
            tap_pads_connected += result.get('tap_pads_connected', 0)
            tap_pads_total += result.get('tap_pads_total', 0)
            tap_edges_routed += result.get('tap_edges_routed', 0)
            tap_edges_failed += result.get('tap_edges_failed', 0)
        # Collect failed pad details for any net with unreached pads. Issue #8:
        # non-multipoint multi-pad nets can also end disconnected (glasgow P1),
        # so this is no longer gated on is_multipoint.
        failed_pads_info = result.get('failed_pads_info', [])
        if failed_pads_info:
            net_name = pcb_data.nets[net_id].name if net_id in pcb_data.nets else f"Net {net_id}"
            failed_multipoint.append({
                'net_name': net_name,
                'net_id': net_id,
                'failed_pads': failed_pads_info
            })

    # ---- COVERAGE GATE (T5 zero-copper custody) ----------------------------
    # Invariant: a net this run DISTURBED must not ship broken and unreported.
    # The sweep above only covers routed_results; a net OUTSIDE it -- skipped as
    # "already fully connected", or a pre-existing rippable net -- can still be
    # broken by this run (ripped by the rip-up ladder and never restored, its
    # restore refused, or its input copper stale-stripped). Those nets are in no
    # failure list, so the final reconciliation never retried them and they
    # shipped at ZERO copper with no owner (ulx3s GN8/GP2/GN22). Check every
    # disturbed, unclassified, multi-pad net against the WRITTEN copper and
    # report the broken ones as failed_multipoint entries -- which both surfaces
    # them honestly in the summary/JSON and feeds them to the end-of-run
    # reconciliation retry below.
    coverage_gate_nets = []
    _cov_disturbed = (set(getattr(state, 'casualty_custody', {}) or {})
                      | set(state.collision_refused_net_ids or set())
                      | {s.net_id for s in _stale_input_segs}
                      | {v.net_id for v in stale_input_vias}
                      # #468 follow-up: a broken terminal restore ('full_open'
                      # / 'stub') puts the original copper BACK, unwinding the
                      # stale-strip signal above -- without this the net leaves
                      # the disturbed set and ships open, unreported.
                      | {nid for nid, v in
                         (getattr(state, 'terminal_restores', None) or {}).items()
                         if v != 'full'})
    _cov_classified = {nid for _, nid in single_ended_nets} | set(routed_results)
    for _nid in sorted((_cov_disturbed & sweep_scope_ids) - _cov_classified):
        _pads = pcb_data.pads_by_net.get(_nid, [])
        if len(_pads) < 2:
            continue
        _r = check_net_connectivity(
            _nid, _segs_by_net.get(_nid, []), _vias_by_net.get(_nid, []),
            _pads, _zones_by_net.get(_nid, []), tolerance=0.02,
            pcb_data=pcb_data)
        # #479 multi-board: only within-outline breaks gate coverage.
        _broken, _dp = net_break_within_outlines(pcb_data, _r)
        if not _broken or not _dp:
            continue
        _net_name = pcb_data.nets[_nid].name if _nid in pcb_data.nets else f"Net {_nid}"
        print(f"{RED}COVERAGE GATE: {_net_name} was disturbed by this run "
              f"(ripped/stripped) and ships with {len(_dp)} disconnected "
              f"pad(s) -- reporting as failed and queuing for the final "
              f"reconciliation{RESET}")
        coverage_gate_nets.append(_net_name)
        failed_multipoint.append({
            'net_name': _net_name,
            'net_id': _nid,
            'failed_pads': [
                {'x': _p[0], 'y': _p[1],
                 'component_ref': _p[3] if len(_p) > 3 else '?',
                 'pad_number': '?'}
                for _p in _dp],
        })

    # ---- FRAGMENT SWEEP -----------------------------------------------------
    # The grading verdict above is pad-oriented: a zone-less net can grade
    # "connected" while its copper sits in several KiCad islands (run 6's
    # VCC3V3: 25/27 reported, 7 islands real). Census every such net with the
    # strict-fragment model and report splits honestly. DISCLOSURE ONLY since
    # the #578 fragment gate was removed for main parity and ported back: these
    # entries name the splits, they no longer re-queue the net for routing.
    fragmented_nets = []
    _frag_already = {m['net_id'] for m in failed_multipoint}
    for _nid, _res in sorted(routed_results.items()):
        if _nid in _frag_already or _zones_by_net.get(_nid):
            continue
        # #734: an untouched pre-existing candidate's fragmented input
        # copper (e.g. a fanout stub at each end of an unrouted net) is
        # the CHAIN's business, not this run's -- without this gate the
        # sweep re-enrolled every net the audit gate had just scoped out.
        if (_nid in _pe_reg734 and _nid not in _scope734
                and _nid not in _pe_disturbed_audit):
            continue
        _segs = _segs_by_net.get(_nid, [])
        if not _segs:
            continue
        try:
            from check_connected import net_copper_fragments
            from routing_common import _max_fragments_within_one_outline
            _frag = net_copper_fragments(
                _nid, _segs, _vias_by_net.get(_nid, []),
                pcb_data.pads_by_net.get(_nid, []), [], pcb_data=pcb_data)
            _nf = _max_fragments_within_one_outline(
                pcb_data, _frag['fragment_anchors'])
        except Exception:
            continue
        if _nf <= 1:
            continue
        _net_name = (pcb_data.nets[_nid].name if _nid in pcb_data.nets
                     else f"Net {_nid}")
        # Same #659 rule as the fragment gate in filter_already_routed, and
        # the same trap: STRICT pad-less-ness is not "dead". Ask the
        # authoritative graph which copper is dead, drop it, and re-count --
        # only if the remainder is whole does the reconciliation have nothing
        # to route (it would otherwise weld to copper the late sweep deletes).
        try:
            if os.environ.get('KICAD_659_DIVERT', '1') == '0':
                raise RuntimeError('KICAD_659_DIVERT=0')
            from check_connected import net_dead_copper as _ndc659
            _dead_s659, _dead_v659 = _ndc659(
                pcb_data, _nid, _segs, _vias_by_net.get(_nid, []),
                pcb_data.pads_by_net.get(_nid, []), [])
        except Exception:
            _dead_s659, _dead_v659 = [], []
        if _dead_s659 or _dead_v659:
            _ds659 = {id(s) for s in _dead_s659}
            _dv659 = {id(v) for v in _dead_v659}
            try:
                _frag2_659 = net_copper_fragments(
                    _nid, [s for s in _segs if id(s) not in _ds659],
                    [v for v in _vias_by_net.get(_nid, [])
                     if id(v) not in _dv659],
                    pcb_data.pads_by_net.get(_nid, []), [], pcb_data=pcb_data)
                _nf2_659 = _max_fragments_within_one_outline(
                    pcb_data, _frag2_659['fragment_anchors'])
            except Exception:
                _nf2_659 = _nf
            if _nf2_659 <= 1:
                print(f"  Fragment sweep: {_net_name} grades pad-connected "
                      f"and every extra fragment is DEAD copper "
                      f"({len(_dead_s659)} segment(s), {len(_dead_v659)} "
                      f"via(s) tied to no pad or pour) -- not a route (#659); "
                      f"left to the late orphan sweep instead of the "
                      f"reconciliation")
                continue
        print(f"{RED}FRAGMENT SWEEP: {_net_name} grades pad-connected but its "
              f"copper is {_nf} separate fragment(s) "
              f"({_frag['padless_fragments']} pad-less) -- reporting as "
              f"failed and queuing for the final reconciliation{RESET}")
        fragmented_nets.append(_net_name)
        failed_multipoint.append({
            'net_name': _net_name,
            'net_id': _nid,
            'failed_pads': [
                {'x': _x, 'y': _y, 'component_ref': '?', 'pad_number': '?'}
                for (_x, _y) in _frag['fragment_anchors'][1:]],
        })

    # ---- ORACLE SUMMARY CHECK (KICAD_ORACLE_SUMMARY) ------------------------
    # KiCad's own connectivity over the AS-WRITTEN board, once per run.
    # Strictly ADDITIVE: it can only add failure disclosure (kicad-cli's
    # threaded connectivity is nondeterministic, so it never reclassifies a
    # failure as success). Entries land in failed_multipoint BEFORE the
    # reconcile gate reads it, so oracle-open nets are retried automatically.
    # OPT-IN: KICAD_ORACLE_SUMMARY=1 ENABLES this; it is OFF by default and
    # this comment used to claim the opposite. It was reverted from default-on
    # under #675 for a good reason -- it appends to failed_multipoint, which
    # the reconcile gate reads, so it CHANGES COPPER, which made routing
    # depend on whether kicad-cli happens to be installed (lora_v3: 7 DRC
    # without KiCad, 0 with) and silently broke A/B comparability. Disclosure
    # may be environment-dependent; copper may not. Do not flip this default
    # without answering that. The GUI path (return_results) skips regardless
    # (in-memory board, file fidelity unavailable) with disclosure.
    oracle_open: Dict[str, int] = {}
    # #659: the GENUINE opens among the flagged links -- both endpoints on
    # live copper -- kept in kicad_unconnected's own shape so the final
    # reconciliation can route the exact endpoints (#572 forced links)
    # instead of re-deriving them and skipping the net as already connected.
    _oracle_open_links: List = []
    oracle_check = 'skipped'
    if not (final_reconcile and not skip_routing and output_file
            and not return_results):
        oracle_check = ('skipped (in-memory board)' if return_results
                        else 'skipped')
    elif not env_knobs.ORACLE_SUMMARY:
        oracle_check = 'disabled'
    else:
        try:
            from kicad_oracle import find_kicad_cli, kicad_unconnected
            _ocli = find_kicad_cli()
        except Exception:
            _ocli = None
        if not _ocli:
            oracle_check = 'unavailable'
            print("  (oracle summary check: kicad-cli not found -- "
                  "in-process grading only)")
        else:
            import tempfile as _otf
            _staged = None
            try:
                _tf = _otf.NamedTemporaryFile(suffix='.kicad_pcb',
                                              delete=False)
                _tf.close()
                _staged = _tf.name
                _ow = write_routed_output(
                    input_file=input_file, output_file=_staged,
                    results=results,
                    all_segment_modifications=all_segment_modifications,
                    all_swap_vias=all_swap_vias,
                    all_swap_segments=all_swap_segments,
                    target_swap_info=[],
                    single_ended_target_swap_info=single_ended_target_swap_info,
                    pad_swaps=pad_swaps, pcb_data=pcb_data,
                    debug_lines=[], exclusion_zone_lines=[],
                    boundary_debug_labels=[], skip_routing=skip_routing,
                    add_teardrops=False,
                    segments_to_remove=dead_end_input_segments,
                    vias_to_remove=stale_input_vias)
                # #650: the staged board's sibling project is the INPUT's
                # (seed_project_for_output), whose declared floors are LOOSER
                # than the ones this run routed to -- the zone filler then
                # pulls the pours back further than the shipped board's will
                # and this check over-reports opens. Its entries land in
                # failed_multipoint before the reconcile gate reads them, so
                # a phantom open costs a real retry. Grade at what ships.
                if env_knobs.INRUN_FLOOR_SYNC:
                    try:
                        from fix_kicad_drc_settings import apply_routed_floors
                        apply_routed_floors(_staged,
                                            clearance=config.clearance,
                                            clamp_nondefault_netclasses=False)
                    except Exception as _e650:
                        print(f"  (in-run DRC floor sync skipped: {_e650})")
                _links = (kicad_unconnected(_staged, _ocli) if _ow else None)
                if _links is None:
                    oracle_check = 'failed'
                else:
                    oracle_check = 'ok'
                    _scope_names = {pcb_data.nets[_n].name
                                    for _n in (set(routed_results)
                                               | sweep_scope_ids)
                                    if _n in pcb_data.nets}
                    _zone_names = {pcb_data.nets[_z].name
                                   for _z in _zones_by_net
                                   if _z in pcb_data.nets}
                    _flagged = {m['net_name'] for m in failed_multipoint}
                    _by_net: Dict[str, list] = {}
                    for _net, _a, _b in _links:
                        if (_net in _scope_names
                                and _net not in _zone_names):
                            _by_net.setdefault(_net, []).append((_a, _b))
                    # #659: KiCad saying "open" does NOT say which repair is
                    # called for, and the three answers are different
                    # operations. Classify each link by what its endpoints
                    # actually sit on:
                    #   live <-> live -> a GENUINE open; weld it (and hand the
                    #       reconciliation the EXACT endpoints, or its own
                    #       model skips the net as already connected).
                    #   pad-less end -> rip/reroute DEBRIS; it conducts
                    #       nothing, so welding adds dead metal. The late
                    #       orphan sweep deletes it at the end of the run.
                    #   graphic end -> net-tagged copper ART (#337/#513):
                    #       immutable, so neither operation applies.
                    # Measured over the recorded corpus the zone-less signal
                    # links split 9 pad-less / 36 graphic / 6 live, so
                    # enrolling all of them sends the router after 45 links it
                    # cannot help -- which is exactly what cost spartan6 448s
                    # and bred three fresh debris nets.
                    from check_connected import classify_unconnected_link
                    _routable_layers659 = set(config.layers or [])
                    for _net, _pairs in sorted(_by_net.items()):
                        oracle_open[_net] = len(_pairs)
                        if _net in _flagged:
                            continue
                        _onid = next((i for i, n in pcb_data.nets.items()
                                      if n.name == _net), None)
                        _genuine659, _debris659, _art659 = [], 0, 0
                        for (_a, _b) in _pairs:
                            try:
                                _ca, _cb = classify_unconnected_link(
                                    pcb_data, _onid, _a, _b,
                                    _a[3] if len(_a) > 3 else None,
                                    _b[3] if len(_b) > 3 else None)
                            except Exception:
                                _ca = _cb = 'unknown'
                            if 'graphic' in (_ca, _cb):
                                _art659 += 1
                            elif 'padless' in (_ca, _cb):
                                _debris659 += 1
                            elif _ca == 'live' and _cb == 'live':
                                _genuine659.append((_a, _b))
                        if not _genuine659:
                            _why659 = []
                            if _debris659:
                                _why659.append(
                                    f"{_debris659} to pad-less debris (the "
                                    f"late orphan sweep removes it)")
                            if _art659:
                                _why659.append(
                                    f"{_art659} to net-tagged copper GRAPHICS "
                                    f"(#513: convert them to pads/tracks)")
                            print(f"  ORACLE CHECK: {_net}: KiCad reports "
                                  f"{len(_pairs)} open link(s), none weldable"
                                  + (" -- " + "; ".join(_why659)
                                     if _why659 else "")
                                  + " -- not queued for the reconciliation "
                                    "(#659)")
                            continue
                        print(f"{RED}ORACLE CHECK: {_net}: KiCad reports "
                              f"{len(_genuine659)} GENUINE open link(s) the "
                              f"in-process grading passed -- reporting as "
                              f"failed and queuing for the final "
                              f"reconciliation{RESET}")
                        failed_multipoint.append({
                            'net_name': _net,
                            'net_id': _onid if _onid is not None else -1,
                            'oracle_only': True,
                            'failed_pads': [
                                {'x': _a[0], 'y': _a[1],
                                 'component_ref': '?', 'pad_number': '?'}
                                for (_a, _b) in _genuine659],
                        })
                        # Forward only links the weld router can actually
                        # take: route_oracle_links returns on the FIRST
                        # failure and discards the copper of every link
                        # already routed for that net, so one endpoint on a
                        # layer outside this run's stack would abort the rest.
                        for (_a, _b) in _genuine659:
                            if (len(_a) > 2 and len(_b) > 2
                                    and _a[2] in _routable_layers659
                                    and _b[2] in _routable_layers659):
                                _oracle_open_links.append((_net, _a, _b))
            except Exception as _oe:
                oracle_check = 'failed'
                print(f"  (oracle summary check failed: {_oe})")
            finally:
                for _p in ((_staged,) if _staged else ()):
                    for _s in (_p, os.path.splitext(_p)[0] + '.kicad_pro'):
                        try:
                            os.unlink(_s)
                        except OSError:
                            pass

    # Derive final counts set-based from this run's scope rather than the loop
    # counters (issue #87): a net with unconnected pads is not fully routed, and
    # a net ripped during Phase 3 whose re-route failed never reaches the failure
    # counter.
    scope_ids = {nid for _, nid in single_ended_nets}
    failed_multipoint_ids = {m['net_id'] for m in failed_multipoint}
    fully_routed_ids = {nid for nid in scope_ids
                        if nid in routed_results and nid not in failed_multipoint_ids}
    successful = len(fully_routed_ids)
    failed = len(scope_ids) - successful

    # Now classify each net for the summary. A net only counts as routed_single
    # if it is fully connected (issue #189): one that routed a result but left
    # pads disconnected (a failed MST edge) is in failed_multipoint_ids and is
    # NOT routed. failed_single stays "no result at all" so the place/route loop
    # does not double-count it (its deficit is already in mp_deficit). The third
    # bucket, open_single, names the nets BETWEEN those two: a kept result whose
    # copper still leaves pads disconnected. Multipoint nets are excluded from
    # it -- their shortfall is already priced pad-by-pad in the multipoint
    # deficit, and counting the net again would double-charge it; a
    # non-multipoint open net previously contributed to NO count at all, so a
    # verdict of failed_single + pad-deficit read 0 while the board shipped
    # open (run-7 west-fan class).
    routed_single = []
    failed_single = []
    failed_single_ids = []  # Track IDs for history output
    open_single = []
    open_single_ids = []
    for net_name, net_id in single_ended_nets:
        if net_id in fully_routed_ids:
            routed_single.append(net_name)
        elif net_id not in routed_results \
                or routed_results[net_id].get('rescue_terrain'):
            # rescue_terrain = tap-only escape copper with zero connectivity
            # progress (net_rescue): the net is still entirely unrouted, so
            # it stays a clean failure -- the terrain copper is disclosed by
            # the rescue summary, not by reclassifying the net as open.
            failed_single.append(net_name)
            failed_single_ids.append(net_id)
        elif not routed_results[net_id].get('is_multipoint'):
            open_single.append(net_name)
            open_single_ids.append(net_id)

    # Print human-readable summary
    print("\n" + "=" * 60)
    print("Routing complete")
    print("=" * 60)
    if single_ended_nets:
        if failed_single or open_single:
            _bad_bits = []
            if failed_single:
                _bad_bits.append(f"{len(failed_single)} FAILED")
            if open_single:
                _bad_bits.append(f"{len(open_single)} OPEN")
            print(f"  {RED}Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed ({', '.join(_bad_bits)}){RESET}")
        else:
            print(f"  Single-ended:  {len(routed_single)}/{len(single_ended_nets)} routed")
    if multipoint_nets > 0:
        tap_pads_failed = tap_pads_total - tap_pads_connected
        if tap_pads_failed > 0:
            print(f"  {RED}Multi-point:   {tap_pads_connected}/{tap_pads_total} pads connected ({tap_pads_failed} FAILED){RESET}")
        else:
            print(f"  Multi-point:   {tap_pads_connected}/{tap_pads_total} pads connected ({multipoint_nets} nets)")
    if ripup_success_pairs:
        print(f"  Rip-up success: {len(ripup_success_pairs)} (routes that ripped blockers)")
    if rerouted_pairs:
        print(f"  Rerouted:      {len(rerouted_pairs)} (ripped nets re-routed)")
    if single_ended_target_swaps:
        from target_swap import summarize_target_swaps
        swap_pairs = summarize_target_swaps(single_ended_target_swaps)
        print(f"  Target swaps:  {len(swap_pairs)}")
    print(f"  Total vias:    {total_vias}")
    print(f"  Total time:    {total_time:.2f}s")
    print(f"  Iterations:    {total_iterations:,}")
    _frag = getattr(config, '_fragility_field', None)
    if _frag is not None and _frag.refreshes:
        print(f"  Fragility refresh (#466): {_frag.refreshes} windows, "
              f"{_frag.refresh_s:.2f}s total")
    from history_congestion import print_history_summary
    print_history_summary(config)

    # Print detailed failure summary
    if failed_single:
        print(f"\n{RED}Failed single-ended nets:{RESET}")
        for net_name in failed_single:
            print(f"  {RED}{net_name}{RESET}")
    if open_single:
        # A kept result is NOT a connection: these nets ship copper with pads
        # still disconnected (pad detail follows in the multi-point list).
        print(f"\n{RED}Routed-but-OPEN single-ended nets (result kept, pads disconnected):{RESET}")
        for net_name in open_single:
            print(f"  {RED}{net_name}{RESET}")
    if failed_multipoint:
        print(f"\n{RED}Failed multi-point connections:{RESET}")
        for item in failed_multipoint:
            net_name = item['net_name']
            for pad in item['failed_pads']:
                print(f"  {RED}{net_name}: {pad['component_ref']} pad {pad['pad_number']} at ({pad['x']:.2f}, {pad['y']:.2f}) not connected{RESET}")

    # Print history for all failed nets (helps debug why they failed) --
    # open nets included: their history shows which attempt left the stub.
    if failed_single_ids or open_single_ids:
        print_failed_net_histories(state, failed_single_ids + open_single_ids,
                                   pcb_data)

    from target_swap import summarize_target_swaps  # cycle-safe swap list (#380)
    summary = {
        'routed_single': routed_single,
        'failed_single': failed_single,
        # Nets with a KEPT result whose pads are still disconnected (their pad
        # detail is in failed_multipoint). Excludes multipoint nets, whose
        # shortfall is already the multipoint pad deficit -- so a verdict may
        # safely count len(failed_single) + len(open_single) + pad-deficit.
        'open_single': open_single,
        'failed_multipoint': [
            {
                'net_name': item['net_name'],
                'failed_pads': [
                    {'component_ref': p['component_ref'], 'pad_number': p['pad_number'], 'x': p['x'], 'y': p['y']}
                    for p in item['failed_pads']
                ]
            }
            for item in failed_multipoint
        ],
        'multipoint_nets': multipoint_nets,
        'multipoint_pads_connected': tap_pads_connected,
        'multipoint_pads_total': tap_pads_total,
        'multipoint_edges_routed': tap_edges_routed,
        'multipoint_edges_failed': tap_edges_failed,
        'ripup_success_pairs': sorted(ripup_success_pairs),
        'rerouted_pairs': sorted(rerouted_pairs),
        'single_ended_target_swaps': [{'net1': k, 'net2': v} for k, v in summarize_target_swaps(single_ended_target_swaps)],
        'layer_swaps': total_layer_swaps,
        'successful': successful,
        'failed': failed,
        'total_time': round(total_time, 2),
        'total_iterations': total_iterations,
        'total_vias': total_vias,
        # Issue #209 fix C: nets a post-routing cleanup pass disconnected (a
        # cleanup bug). Empty in the normal case; non-empty flags dropped copper.
        'cleanup_disconnected': cleanup_disconnected,
        # Smallest copper clearance any step actually routed at (e.g. fine-pitch
        # taps below the nominal). Grade/check_drc the board at this floor.
        'min_clearance_used': __import__('clearance_ledger').effective(clearance),
    }
    if impedance_width_clamped:
        # #610: layers whose impedance-solved width was clamped UP to the
        # width floor, {layer: [solved_mm, floor_mm]}. Those layers route at
        # floor_mm and will NOT meet the impedance request -- the clamp was
        # loud on a terminal and invisible to pipelines. Key absent when no
        # clamp fired.
        summary['impedance_width_clamped'] = impedance_width_clamped
    if rescue_summary:
        # #331/#371 rescue pass outcome (key absent when nothing was rescued,
        # so pre-rescue JSON_SUMMARY consumers/diffs are unaffected).
        summary['rescue'] = rescue_summary
    if terminal_escalation_summary:
        # Terminal geometry escalation outcome per net: "recovered at WxS/D"
        # or "unrecovered" (key absent when nothing was attempted).
        summary['terminal_escalations'] = terminal_escalation_summary['nets']
    if _pe_outcomes:
        # Main-pass pre-existing rips (0805): {victim net: outcome}. Key
        # absent when no pre-existing net was ripped.
        summary['preexisting_rips'] = _pe_outcomes
    if casualty_summary and casualty_summary.get('attempted'):
        # T5 custody: casualties-only reconcile tally (additive; key absent
        # when no rip casualty occurred -- the common case).
        summary['casualty_reconcile'] = casualty_summary
    if coverage_gate_nets:
        # T5 coverage gate: disturbed out-of-scope nets shipping broken
        # (additive; also present as failed_multipoint entries above).
        summary['coverage_gate_nets'] = coverage_gate_nets
    if getattr(state, 'terminal_restores', None):
        # #468 follow-up (run-7 E2): terminal-restore outcomes per net --
        # 'full' is a true restoration; 'full_open' restored copper that never
        # covered every pad; 'stub' kept only the escape stubs. Additive; the
        # broken outcomes are also graded by the sweeps/coverage gate above.
        summary['terminal_restores'] = {
            (pcb_data.nets[_n].name if _n in pcb_data.nets else str(_n)): _v
            for _n, _v in sorted(state.terminal_restores.items())}
    if fragmented_nets:
        # Fragment sweep: pad-connected nets whose copper is several KiCad islands
        # (additive; also present as failed_multipoint entries above).
        summary['fragmented_nets'] = fragmented_nets
    if stacked_copper_findings:
        # run-7 E3: same-net duplicate copper still on the write model after
        # the via dedup (additive; KiCad's DRC never flags same-net stacks,
        # so this key is the only disclosure).
        summary['stacked_copper'] = stacked_copper_findings
    summary['oracle_check'] = oracle_check
    if oracle_open:
        # Oracle summary: KiCad's own open-link counts for this run's scope
        # (additive; nets not already flagged also gained failed_multipoint
        # entries marked oracle_only).
        summary['oracle_open'] = oracle_open
    _ripped_open = sorted({m['net_name'] for m in failed_multipoint
                           if m['net_id'] not in scope_ids})
    if _ripped_open:
        # 1.3 rip-return integrity: nets OUTSIDE this run's --nets scope
        # (ripped pre-existing / disturbed) shipping with opens. They cannot
        # move successful/failed (scope-based), so they get their own key --
        # a caller that only reads the scoped tallies must still see them.
        summary['ripped_open'] = _ripped_open
        # ...and say whether the VERDICT already counts them. The verdict is
        # `failed_single + open_single + pad-deficit`; `ripped_open` is in none
        # of the first two by construction (it is scope-EXCLUDED), and the
        # deficit ships as two bare scalars with no per-net attribution, so
        # membership in the third was untestable from the summary. One run's
        # true open count was therefore a RANGE -- 58 to 61 -- that no
        # published field could narrow. This names the uncounted set instead of
        # leaving the reader to guess it.
        _counted = set(summary.get('failed_single') or []) \
            | set(summary.get('open_single') or [])
        summary['ripped_open_uncounted'] = sorted(
            n for n in _ripped_open if n not in _counted)
        summary['ripped_open_note'] = (
            'ripped_open nets are OUTSIDE this run\'s --nets scope, so they '
            'move neither failed_single nor open_single. Those listed in '
            'ripped_open_uncounted are additionally not in either scoped term: '
            'if they are also outside the multipoint pad deficit, the run\'s '
            'open-net verdict is understated by that many.')
    try:
        from protected_nets import PROTECTED_SKIPPED
        if PROTECTED_SKIPPED:
            # {context: {net: reason}}. reason 'locked' has NO override --
            # a caller retrying with the net named exactly will be refused
            # again, so it must stop rather than loop.
            summary['protected_skipped'] = {
                _c: dict(_m) for _c, _m in PROTECTED_SKIPPED.items()}
    except Exception:
        pass
    # #409: report-only frontier-blocking attribution per net still failed at
    # END of run (additive; key omitted when no failed net has a recorded
    # analysis). Last-wins per net -- 'stage' names the loop that recorded it;
    # a net that failed early but was later rescued/rerouted is filtered out
    # here because the final failed sets are authoritative.
    blockers_report = []
    boxed_in_report = []
    try:
        _final_failed_ids = list(dict.fromkeys(
            failed_single_ids + [m['net_id'] for m in failed_multipoint]))
        for _nid in _final_failed_ids:
            _name = (pcb_data.nets[_nid].name if _nid in pcb_data.nets
                     else f"Net {_nid}")
            _fb = state.frontier_blocking.get(_nid)
            if _fb and _fb.get('blocked_by'):
                _rec = {'net': _name, 'stage': _fb['stage'],
                        'blocked_by': _fb['blocked_by']}
                if _fb.get('more'):
                    _rec['more'] = _fb['more']
                blockers_report.append(_rec)
            # Boxed-in failures ('no rippable blockers found') name their
            # PRE-EXISTING blockers only in prose (#301/#103 hint), so retry
            # tooling had to regex the log. Serialize the recorded hint
            # events in the same schema: last event wins (matches the
            # frontier_blocking last-wins convention); entries are minimal
            # (the hint records names, not cell counts) and carry
            # "preexisting": true as the marker. Report-only.
            _pre103 = None
            for _ev in (state.net_history.get(_nid) or []):
                if _ev.get('event') == 'preexisting_blockers':
                    _pre103 = ((_ev.get('details') or {}).get('blockers')
                               or _pre103)
            if _pre103:
                blockers_report.append({
                    'net': _name, 'stage': 'preexisting',
                    'blocked_by': [{'net': _bn, 'preexisting': True}
                                   for _bn in _pre103]})
            # The static-vs-congestion verdict (the "boxed in by static
            # obstacles" hint, #95) as its OWN key, beside `blockers` rather
            # than inside it: `blockers` names WHICH routed copper is in the
            # way, `boxed_in` says whether anything rippable is in the way at
            # all, and a consumer reads both.
            _bx = None
            for _ev in (state.net_history.get(_nid) or []):
                if _ev.get('event') == 'boxed_in_static':
                    _bx = _ev.get('details') or _bx
            if _bx:
                boxed_in_report.append(dict(_bx, net=_name))
        if blockers_report:
            summary['blockers'] = blockers_report
        if boxed_in_report:
            summary['boxed_in'] = boxed_in_report
    except Exception:
        blockers_report = []
    # #409 follow-up: pad-pair routability tallies (PRR ingredients: connected
    # = |pads| - pad components from the authoritative union-find above, total
    # = |pads| - 1, both counted per outer outline -- a cross-outline gap is
    # neither a routable pair nor an open one, matching #479's
    # net_break_within_outlines semantics) plus a per-open-net outcome.
    # Population: every multi-pad net graded against the WRITTEN board -- the
    # sweep over routed_results (which includes pre-existing rippable nets)
    # plus scope nets with no result and coverage-gate nets, graded here
    # identically. NOT reconcilable with multipoint_edges_* (component-MST:
    # pre-existing copper joins terminals, so there are fewer edges than pad
    # pairs). route.py runs no DRC, so every route-time deficit is an 'open';
    # shorts are check_drc.py's domain ('outcome' exists so a DRC-integrated
    # emitter can add 'short' without a schema break). Computed before the
    # final reconciliation, like 'blockers'.
    pad_pairs_open_report = []
    try:
        _straggler_ids = list(failed_single_ids) + \
            [m['net_id'] for m in failed_multipoint]
        for _nid in _straggler_ids:
            if _nid in _pad_pair_stats or _nid in state.diff_pair_by_net_id:
                continue
            _pads = pcb_data.pads_by_net.get(_nid, [])
            if len(_pads) < 2:
                continue
            _r = check_net_connectivity(
                _nid, _segs_by_net.get(_nid, []), _vias_by_net.get(_nid, []),
                _pads, _zones_by_net.get(_nid, []), tolerance=0.02,
                pcb_data=pcb_data)
            _pad_pair_stats[_nid] = net_pad_pairs_within_outlines(
                pcb_data, _r, _pads)
        _pp_conn = 0
        _pp_total = 0
        _refused = set(state.collision_refused_net_ids or set())
        for _nid, (_pt, _pc) in _pad_pair_stats.items():
            # The k == 0 (padless-copper answer) and invisible-pad clamps live
            # in net_pad_pairs_within_outlines, applied per outline.
            _pp_total += _pt
            _pp_conn += _pc
            if _pc >= _pt:
                continue
            _name = (pcb_data.nets[_nid].name if _nid in pcb_data.nets
                     else f"Net {_nid}")
            if _nid in _refused:
                _sub = 'collision_refused'
            elif _nid in failed_multipoint_ids and _nid not in routed_results:
                _sub = 'coverage_gate'
            elif _nid not in routed_results \
                    or routed_results[_nid].get('rescue_terrain'):
                _sub = 'unrouted'
            else:
                _sub = 'partial'
            pad_pairs_open_report.append({
                'net': _name, 'pairs_connected': _pc, 'pairs_total': _pt,
                'outcome': 'open', 'open_subtype': _sub})
        pad_pairs_open_report.sort(key=lambda e: e['net'])
        if _pp_total:
            summary['pad_pairs_connected'] = _pp_conn
            summary['pad_pairs_total'] = _pp_total
        if pad_pairs_open_report:
            summary['pad_pairs_open'] = pad_pairs_open_report
    except Exception:
        pad_pairs_open_report = []

    # #487: trace-side ampacity for the POWER nets this call sized (the IPC
    # model was plane-only; a routed power trace's capacity went unreported).
    # Bottleneck segment at the layer's stackup copper weight; report-only,
    # printed AND carried in the summary so chains/graders can gate on it.
    try:
        from plane_resistance import (analyze_power_trace_net,
                                      print_power_trace_ampacity)
        _amp = {}
        for _pnid in sorted(config.power_net_widths or {}):
            _r = analyze_power_trace_net(pcb_data, _pnid)
            if _r:
                _pname = (pcb_data.nets[_pnid].name
                          if _pnid in pcb_data.nets else f"net_{_pnid}")
                _amp[_pname] = _r
        if _amp:
            print_power_trace_ampacity(_amp)
            summary['power_trace_ampacity'] = [
                {'net': _n,
                 'bottleneck_width_mm': round(_r['bottleneck_width'], 4),
                 'bottleneck_layer': _r['bottleneck_layer'],
                 'copper_oz': _r['copper_oz'],
                 'max_current_a': round(_r['max_current'], 2),
                 'max_current_ipc2152_a': round(_r['max_current_ipc2152'], 2),
                 'temp_rise_c': _r['temp_rise_c'],
                 'trace_length_mm': round(_r['trace_length'], 1)}
                for _n, _r in sorted(_amp.items())]
    except Exception:
        pass
    # WHICH SUMMARY IS THIS? A run that fires the reconciliation sub-pass emits
    # a SECOND JSON_SUMMARY, scoped to that subset, and the only thing saying so
    # was a prose "Note:" printed after it. Anything that scrapes the last
    # JSON_SUMMARY -- a tail, a grep, a person -- then reads the subset's tally
    # as the run's. Measured on run 14: one A/B arm emitted two summaries and
    # the other one, so the arms were compared at `routed_single: 1` against
    # `121` and the wrong arm looked catastrophic.
    #
    # Derived from the sink rather than a new kwarg, because the sink's own
    # contract already IS this distinction: "the first pass and, when it fires,
    # the reconciliation sub-run's".
    summary['scope'] = 'run' if not _SUMMARY_SINK else 'reconciliation-subset'
    if summary['scope'] != 'run':
        # These are recomputed over the WHOLE board even in the subset pass, so
        # a reader merging tallies must not add them twice.
        summary['board_scoped_keys'] = [k for k in ('stacked_copper',
                                                    'power_trace_ampacity',
                                                    'min_clearance_used')
                                        if k in summary]
    try:                       # #653: env knobs into the machine-readable
        import env_knobs as _ek653   # summary, so a harness can detect a
        summary['env_knobs'] = _ek653.active_env_knobs()   # dirty baseline
    except Exception:          # without re-reading logs
        pass
    print(f"JSON_SUMMARY: {json.dumps(summary)}")
    _SUMMARY_SINK.append(summary)

    # Write output file or return results for direct application
    if return_results:
        # Return results data for direct application (e.g., KiCad plugin).
        # Swap/modification info must be applied to the live board just like
        # write_routed_output applies it to the output file.
        results_data = {
            'results': results,
            'all_swap_vias': all_swap_vias,
            'all_swap_segments': all_swap_segments,
            'pad_swaps': pad_swaps,
            'single_ended_target_swap_info': single_ended_target_swap_info,
            'all_segment_modifications': all_segment_modifications,
            'exclusion_zone_lines': exclusion_zone_lines if debug_lines else [],
            'boundary_debug_labels': boundary_debug_labels if debug_lines else [],
            # Original-file dead-end copper the caller (GUI) should delete from the
            # live board, mirroring the writer's strip (issue #84).
            'segments_to_remove': dead_end_input_segments,
            'vias_to_remove': stale_input_vias,
            # #409: same data as JSON_SUMMARY['blockers'] (may be empty).
            'blockers': blockers_report,
            # ...and the static-vs-congestion verdict beside it, same data as
            # JSON_SUMMARY['boxed_in']. The GUI reads results_data, not the
            # printed summary, so a key that only reaches stdout does not
            # reach the plugin (CLAUDE.md's Class-1 CLI/GUI gap).
            'boxed_in': boxed_in_report,
            # #409 follow-up: same data as JSON_SUMMARY['pad_pairs_open']
            # (may be empty).
            'pad_pairs_open': pad_pairs_open_report,
        }
    else:
        # Write output file using extracted output_writer module
        wrote = write_routed_output(
            input_file=input_file,
            output_file=output_file,
            results=results,
            all_segment_modifications=all_segment_modifications,
            all_swap_vias=all_swap_vias,
            all_swap_segments=all_swap_segments,
            target_swap_info=[],
            single_ended_target_swap_info=single_ended_target_swap_info,
            pad_swaps=pad_swaps,
            pcb_data=pcb_data,
            debug_lines=debug_lines,
            exclusion_zone_lines=exclusion_zone_lines,
            boundary_debug_labels=boundary_debug_labels,
            skip_routing=skip_routing,
            add_teardrops=add_teardrops,
            segments_to_remove=dead_end_input_segments,
            vias_to_remove=stale_input_vias
        )
        # When nothing could be routed (every net failed) there is no copper to
        # write, so write_routed_output produces no file. Pass the board through
        # unchanged so the pipeline never loses its board and a later step (a
        # finer-grid retry, planes, repair) can still run on it (issues #90, #167)
        # -- otherwise the whole chain FileNotFoundErrors on the missing output.
        if not wrote and output_file:
            _write_passthrough_output(input_file, output_file)
        elif wrote and output_file:
            # Deep ledger (KICAD_BOARD_LEDGER=1): the written FILE must match
            # pcb_data per in-scope net -- audits the writer's text transforms
            # (layer mods, polarity/target swap relabels, strips) on top of the
            # in-memory contract checked before the write.
            verify_written_file_parity(output_file, pcb_data, sweep_scope_ids,
                                       label=' route')
        if output_file and os.path.isfile(output_file):
            # #650: sync the output's sibling .kicad_pro to the floors this run
            # ROUTED to, BEFORE anything grades the board in-run (the plane
            # finalize's oracle audit, its #589 re-audit). Until now that
            # sibling was the input's, seeded by seed_project_for_output, and
            # the audit graded the pours at the input's declared floors while
            # the shipped board is graded at the clamped ones -- measured on
            # orangecrab, 57 reported opens vs the 46 that ship (GND 19 vs
            # 11), all of it rules.min_hole_clearance 0.25 -> 0.0889.
            # Only-loosen and idempotent, so main()'s writeback stays
            # authoritative.
            #
            # SIZE OF THE GAP IS CHAIN-POSITION DEPENDENT, so do not read that
            # 57/46 as a per-step cost: the declared floors survive only until
            # the FIRST writeback (down the recorded orangecrab chain
            # min_hole_clearance goes 0.25 -> 0.09 at step 1 and stays there,
            # leaving later steps stale by ~1 um -- inert). This earns its
            # keep on the first step over a board still carrying its declared
            # floors, which includes the ordinary case of a board that arrives
            # with pours already drawn, and on the aspirational stock netclass
            # (0.2 declared, routed 0.1) that CLAUDE.md describes.
            #
            # clamp_nondefault_netclasses is deliberately FALSE: whether the
            # non-Default classes get clamped is the writeback's call (#439 --
            # it depends on the caller having passed a --clearance ceiling,
            # which is a main() fact, not an engine one). Lowering them here
            # could ship a tightened class on a run that meant to honor them.
            if env_knobs.INRUN_FLOOR_SYNC:
                try:
                    from fix_kicad_drc_settings import apply_routed_floors
                    apply_routed_floors(output_file,
                                        clearance=config.clearance,
                                        clamp_nondefault_netclasses=False,
                                        verbose=True)
                except Exception as _e650b:
                    print(f"  (in-run DRC floor sync skipped: {_e650b})")
        if wrote and output_file:
            # #666 always-on write-divergence repair: a rip/restore cycle can
            # leave a net's copper in the MODEL while the write lists dropped
            # it -- the file ships the net bare while every ledger says
            # 'routed' (measured: IO_9 in routed_single with 0 segs written;
            # RAM_D9's restored pre-existing copper stripped by the next
            # pass). Measurement over bookkeeping: re-emit any in-scope
            # net's model copper that the WRITTEN FILE lost entirely.
            try:
                from kicad_parser import parse_kicad_pcb as _pk666
                from kicad_writer import add_tracks_and_vias_to_pcb as _aw666
                _out666 = _pk666(output_file)
                _fs666 = {s.net_id for s in _out666.segments}
                _fv666 = {v.net_id for v in _out666.vias}
                _scope666 = {nid for _n, nid in net_ids}
                _lost666 = []
                for _nid in _scope666:
                    _ms = [s for s in pcb_data.segments if s.net_id == _nid]
                    _mv = [v for v in pcb_data.vias if v.net_id == _nid]
                    if (_ms or _mv) and _nid not in _fs666 \
                            and _nid not in _fv666:
                        _lost666.append((_nid, _ms, _mv))
                if _lost666:
                    _names666 = [pcb_data.nets[n].name
                                 for n, _, _ in _lost666]
                    print(f"  WARNING (#666): the written file LOST all "
                          f"copper of {len(_lost666)} routed net(s) the "
                          f"model still holds -- re-emitting: "
                          f"{', '.join(_names666)}")
                    _tr666 = [
                        {'start': (s.start_x, s.start_y),
                         'end': (s.end_x, s.end_y), 'width': s.width,
                         'layer': s.layer, 'net_id': s.net_id}
                        for _, _ms, _ in _lost666 for s in _ms]
                    _vi666 = [
                        {'x': v.x, 'y': v.y, 'size': v.size,
                         'drill': v.drill, 'layers': v.layers,
                         'net_id': v.net_id,
                         # #749 A: these are PRE-EXISTING vias -- copper the
                         # model still holds and the write dropped -- so they
                         # carry their own protection back, and one that had
                         # none keeps INHERITING the board's `(setup ...)`
                         # rather than gaining the prevailing spec on the way
                         # in. Without both keys a re-emitted via-in-pad ships
                         # tented instead of filled+capped+plated.
                         'tenting_attrs': dict(
                             getattr(v, 'tenting_attrs', None) or {}),
                         'inherit_when_unspecified': True}
                        for _, _, _mv in _lost666 for v in _mv]
                    import tempfile as _tf666, shutil as _sh666
                    _fd666, _tmp666 = _tf666.mkstemp(suffix='.kicad_pcb')
                    import os as _os666
                    _os666.close(_fd666)
                    if _aw666(output_file, _tmp666, _tr666, _vi666):
                        _sh666.move(_tmp666, output_file)
            except Exception as _e666:
                print(f"  (write-divergence repair unavailable: {_e666})")
            # #666/IO_9 scoped cap move (CLI file mode, like the reaudit):
            # the bare-ball rescue shipped an escape via that conflicts
            # with a MOVABLE cap, with a relocation verified at rescue
            # time. Apply the move to the written file, mirror it into
            # pcb_data, and re-weld the moved cap's nets with the oracle
            # (union source: KiCad decides what needs welding). The full
            # clearance step must NOT rerun post-route (strands every
            # moved cap's joints, measured 6->13); this moves ONLY the
            # conflicting cap(s).
            _capmv = getattr(pcb_data, '_pending_cap_moves', None)
            if _capmv:
                try:
                    from placement.writer import write_placed_output
                    print(f"  #666 scoped cap move: relocating "
                          f"{len(_capmv)} cap(s) off rescue via(s): "
                          + ', '.join(m['reference'] for m in _capmv))
                    if write_placed_output(output_file, output_file,
                                           _capmv):
                        _mvnets = set()
                        for _mv in _capmv:
                            _fpmv = pcb_data.footprints.get(
                                _mv['reference'])
                            if _fpmv is not None:
                                _dxm = _mv['new_x'] - _fpmv.x
                                _dym = _mv['new_y'] - _fpmv.y
                                _fpmv.x = _mv['new_x']
                                _fpmv.y = _mv['new_y']
                                for _pmv in _fpmv.pads:
                                    _pmv.global_x += _dxm
                                    _pmv.global_y += _dym
                            _mvnets.update(_mv.get('net_ids') or [])
                        _mvnames = sorted(
                            pcb_data.nets[n].name for n in _mvnets
                            if n in pcb_data.nets)
                        if _mvnames:
                            from kicad_oracle import oracle_reconnect
                            _cap_cfg = GridRouteConfig(
                                clearance=config.clearance,
                                track_width=config.track_width,
                                via_size=config.via_size,
                                via_drill=config.via_drill,
                                grid_step=config.grid_step,
                                layers=list(config.layers),
                                layer_costs=(list(config.layer_costs)
                                             if getattr(config,
                                                        'layer_costs',
                                                        None) else []),
                                power_net_widths=dict(
                                    getattr(config, 'power_net_widths',
                                            None) or {}))
                            _orc_cap = oracle_reconnect(
                                output_file, _mvnames, _cap_cfg,
                                track_via_clearance=config.clearance,
                                hole_to_hole_clearance=(
                                    config.hole_to_hole_clearance),
                                project_from=input_file)
                            # The FOURTH oracle_reconnect consumer, and the one
                            # #713 item 3's first pass missed. Without this it
                            # printed "0 link(s) welded, -1 remaining" for an
                            # oracle that could not run, indistinguishable
                            # from one that ran and found nothing to do.
                            if not _orc_cap.get('available'):
                                _capwhy = _orc_cap.get(
                                    'why', 'no reason recorded')
                                print(f"  #666 cap-move re-weld: DID NOT RUN "
                                      f"-- {_capwhy}; the re-weld is "
                                      f"unchecked")
                            else:
                                print(f"  #666 cap-move re-weld: "
                                      f"{_orc_cap.get('links_routed', 0)} "
                                      f"link(s) welded, "
                                      f"{_orc_cap.get('remaining', -1)} "
                                      f"remaining")
                except Exception as _ecap:
                    print(f"  (scoped cap move failed: {_ecap})")

    # Update schematics with swap info if directory specified
    if schematic_dir and single_ended_target_swap_info:
        # Convert swap info to format for schematic updater
        schematic_swaps = []
        for info in single_ended_target_swap_info:
            if info.get('n1_pad') and info.get('n2_pad'):
                pad1 = info['n1_pad']
                pad2 = info['n2_pad']
                schematic_swaps.append({
                    'component_ref': pad1.component_ref,
                    'pad1': pad1.pad_number,
                    'pad2': pad2.pad_number
                })
        if schematic_swaps:
            apply_swaps_to_schematics(schematic_dir, schematic_swaps, verbose=verbose)

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

    # Obstacle-map ref-count integrity audit (issue: ref-count leak/desync hunt).
    # Invariant: working_obstacles == base_obstacles + sum(net_obstacles_cache).
    # Clone the working map, remove every net's CURRENT cache, and compare to the
    # base. Any residual means a per-net contribution the cache no longer accounts
    # for -- a leak (add not mirrored by remove) or an over-decrement. Env-gated so
    # normal runs pay nothing; fully defensive (never breaks a real route).
    if env_knobs.OBSTACLE_AUDIT:
        from obstacle_cache import run_obstacle_audit
        run_obstacle_audit(base_obstacles, state.working_obstacles,
                           state.net_obstacles_cache)

    # #348 (glasgow /SCL): END-OF-RUN RECONCILIATION. Mid-run rip churn can
    # leave a victim net partially connected whose gap is trivially routable
    # in the FINAL board state -- /SCL shipped with 6 pads open after a rip
    # victim's re-route failed against an interim board, yet route.py
    # connects it in 0.25s when asked on the finished board (Andy proved the
    # corridor by hand-bridging it with one via first). The authoritative
    # end-of-run check above knows exactly which nets are incomplete; give
    # them ONE more standard pass against the board as written. Self-invokes
    # batch_route on the output file (one level -- final_reconcile=False),
    # mirroring repair_planes' rip-casualty self-reconnect. The
    # reconcile pass prints its own summary/JSON_SUMMARY scoped to the
    # retried nets; the failure lists in that LAST summary are the honest
    # still-open set. Runs on BOTH fronts: CLI re-invokes on the written
    # file; GUI (return_results) re-invokes on the in-memory board and
    # merges the sub-run's results (claude-tab/stress parity gap closure).
    _custody_nets9 = []
    _custody_links9 = []
    _zone_complete9 = set()
    # PLANE FINALIZE (#562): the pours-first chain's repair step, absorbed.
    # Measured (3-board arch chains): the standalone repair step's entire
    # remaining value is plane-net completion its TAP machinery + the
    # kicad-oracle exact-fill reconnect earn (ux +3V3: 21 pad taps + gate
    # multipoint + rescue; oc P3.3V: one pad tap) -- the casualties-only and
    # failed-nets reconciles above cannot reach it (multipoint plane deficits
    # are neither). Run repair's exact trio IN-RUN on the written output:
    # route_planes engine (taps + region joins) -> clean_plane_copper ->
    # oracle_reconnect (KiCad exact-fill links, the completion earner).
    # ORDER SWAP: runs BEFORE the final reconciliation -- live reuse is then
    # sound by construction, and the custody nets merge into the reconcile's
    # single sub-run (one parse/base-build serves both).
    # In-process bonus: clearance_ledger is shared, so tap-escalated floors
    # flow into this run's own DRC writeback.
    # BOTH FRONTS (#562 graduation): CLI file-mode runs repair's file trio on
    # the written output; GUI (return_results) runs the SAME engine against
    # the in-memory board -- repair_planes(pcb_data=..., return_results=True)
    # already runs the shared cleanup pipeline in-memory, and the finalize
    # merges the resulting BOARD DELTA (object diff of pcb_data before/after)
    # into results_data, so the reconcile's identity-based strip de-dup keeps
    # working. The ORACLE leg needs a real file for kicad-cli's exact fill,
    # so under return_results it is deferred to the applier: the finalize
    # posts results_data['plane_finalize_oracle'] and swig_gui runs the
    # staged-save oracle (planes-tab pattern) after the copper lands on the
    # live board.
    # Gate: KICAD_PLANE_FINALIZE=1 (default ON; env var is a kill switch).
    #
    # Debug re-entry (Andy, 0805): most finalize debugging pays a full route
    # step per cycle. Two env knobs split the run at THIS boundary:
    #   KICAD_CKPT_PREFINALIZE=1  -> snapshot the just-written board (+
    #     .kicad_pro sibling) to <output-stem>.prefinalize.kicad_pcb before
    #     the finalize touches it (CLI file mode only).
    #   KICAD_FINALIZE_ONLY=1     -> with --skip-routing, run ONLY the
    #     finalize + final reconciliation on the input board's existing
    #     copper:  KICAD_FINALIZE_ONLY=1 python3 py_router/route.py \
    #       x.prefinalize.kicad_pcb --output dbg.kicad_pcb --skip-routing \
    #       --nets <same> <same params>
    _fin_only = os.environ.get('KICAD_FINALIZE_ONLY', '0') == '1'
    _reaudit9 = None  # #589: post-reconcile oracle re-audit context
    if (os.environ.get('KICAD_CKPT_PREFINALIZE', '0') == '1'
            and output_file and not return_results and not skip_routing):
        try:
            import shutil as _sh9
            _stem9 = (output_file[:-len('.kicad_pcb')]
                      if output_file.endswith('.kicad_pcb') else output_file)
            _ck9 = _stem9 + '.prefinalize.kicad_pcb'
            _sh9.copy2(output_file, _ck9)
            _pro9 = _stem9 + '.kicad_pro'
            if os.path.exists(_pro9):
                _sh9.copy2(_pro9, _stem9 + '.prefinalize.kicad_pro')
            print(f"  CHECKPOINT: pre-finalize board -> {_ck9} "
                  f"(KICAD_CKPT_PREFINALIZE; re-enter with "
                  f"KICAD_FINALIZE_ONLY=1 + --skip-routing)")
        except Exception as _e9c:
            print(f"  (pre-finalize checkpoint failed: {_e9c})")
    if (final_reconcile and (not skip_routing or _fin_only)
            and not _ckpt_stop
            and (output_file or return_results)
            and not _plane_finalize_active()
            and os.environ.get('KICAD_PLANE_FINALIZE', '1') == '1'):
        try:
            _finalize_depth(+1)
            from repair_planes import (
                repair_planes as _rdp_engine, auto_detect_zones as _adz)
            _gui9 = bool(return_results)
            if _gui9:
                # No written file to detect zones from: derive the (net,
                # layer) pairs from the in-memory board, in zone order,
                # deduped -- the same pairs auto_detect_zones reads from the
                # file (one entry per copper layer of each pour).
                _zpairs = []
                _seen9 = set()
                for _z9 in (getattr(pcb_data, 'zones', None) or []):
                    _nm9 = (pcb_data.nets[_z9.net_id].name
                            if _z9.net_id in pcb_data.nets
                            else getattr(_z9, 'net_name', None))
                    _ly9 = getattr(_z9, 'layer', None)
                    if not _nm9 or not _ly9 or not _ly9.endswith('.Cu'):
                        continue
                    if (_nm9, _ly9) not in _seen9:
                        _seen9.add((_nm9, _ly9))
                        _zpairs.append((_nm9, _ly9))
            else:
                _zpairs = _adz(output_file)
            # Respect the caller's net filter: a net excluded by pattern is
            # excluded BY PLAN (same rule as the reconciliation above).
            # But say so LOUDLY. Since #562 a pour ALONE connects nothing --
            # a plane net outside this route's scope gets no weld, no
            # finalize, no oracle, and if no LATER route step covers it,
            # every one of its pads ships disconnected while this step
            # reports success for its own scope (review finding F1: a
            # scoped retry chain `route.py --nets 'SDRAM_*'` after a pour
            # is a realistic shape that hits this).
            if net_names:
                from net_queries import matches_net_filter as _mnf9
                _excluded9 = sorted({n for n, _l in _zpairs
                                     if not _mnf9(n, net_names)})
                _zpairs = [(n, l) for n, l in _zpairs if _mnf9(n, net_names)]
                if _excluded9:
                    print(f"{RED}  Plane finalize: zone net(s) "
                          f"{', '.join(_excluded9)} are OUTSIDE this route's "
                          f"--nets scope -- excluded from the finalize BY "
                          f"PLAN. Since #562 a pour alone connects nothing: "
                          f"unless a later route step covers these nets, "
                          f"their pads ship disconnected.{RESET}")
                    # ...and in the SUMMARY, not only in the log, so a later
                    # grade can tell a step that declined to weld these pads
                    # BY PLAN from one that failed to. The printed
                    # `JSON_SUMMARY:` line predates the finalize, so it does
                    # NOT carry this key; `summary` sits in `_SUMMARY_SINK` by
                    # reference, so `--json-out`, the returned dict and the
                    # JSON_SUMMARY_MIN line (printed last) do.
                    summary['finalize_excluded_nets'] = _excluded9
            # PRE-GATE for the MODEL-BASED legs (engine taps/joins +
            # cleanup): run them only for zone nets the fill-aware checker
            # says are incomplete on THIS run's board (pcb_data == file
            # here -- the finalize precedes the reconcile). The ORACLE leg deliberately stays
            # UNGATED on the full zone-net set: the raster model
            # OVER-credits (castor-class gaps only KiCad's exact fill
            # sees), and the oracle is the model-independent verifier --
            # on a healthy board it costs one refill and exits at round 0.
            _zpairs_all = list(_zpairs)
            if _zpairs:
                from check_connected import check_net_connectivity as _cnc9
                _zbn9 = {}
                for _z9 in (getattr(pcb_data, 'zones', None) or []):
                    _zbn9.setdefault(_z9.net_id, []).append(_z9)
                _broken9 = []
                for _nid9, _net9 in pcb_data.nets.items():
                    if _net9.name not in {n for n, _l in _zpairs}:
                        continue
                    try:
                        _r9 = _cnc9(
                            _nid9,
                            [s for s in pcb_data.segments
                             if s.net_id == _nid9],
                            [v for v in pcb_data.vias if v.net_id == _nid9],
                            pcb_data.pads_by_net.get(_nid9, []),
                            _zbn9.get(_nid9, []), pcb_data=pcb_data)
                        if not _r9.get('connected'):
                            _broken9.append(_net9.name)
                    except Exception:
                        _broken9.append(_net9.name)  # unknown -> repair it
                if not _broken9:
                    print("\nPlane finalize (#562): all zone nets complete "
                          "(fill-aware) -- skipping engine/cleanup legs, "
                          "oracle verifies")
                    _zpairs = []
                else:
                    _zpairs = [(n, _l) for n, _l in _zpairs
                               if n in set(_broken9)]
            _finalize_rip9 = os.environ.get('KICAD_FINALIZE_RIP', '1') == '1'
            if _zpairs:
                _zn = [n for n, _l in _zpairs]
                _zl = [_l for _n, _l in _zpairs]
                print(f"\nPlane finalize (#562): repairing "
                      f"{len(set(_zn))} zone net(s) in-run"
                      f"{' (rip authority over blockers)' if _finalize_rip9 else ''}: "
                      f"{', '.join(sorted(set(_zn)))}")
                import time as _time9
                _t9 = _time9.time()
                # Live-board reuse (KICAD_PLANE_FINALIZE_LIVE=1): hand the
                # engine THIS run's pcb_data instead of re-parsing the file it
                # just wrote -- kills the parse and, bigger, reuses the
                # ZoneFillModel caches the pour-launch ladder already built
                # for every plane net. SNAP FIRST (the reconcile lesson: the
                # written file is nm-grid-exact, the in-memory board is not,
                # and repairing against a different board than the file forks
                # the outcome). #508-class risk (write-list vs pcb_data
                # divergence) is why this has its own gate; A/B with
                # KICAD_BOARD_LEDGER=1 before defaulting it on.
                # Live reuse is sound BY CONSTRUCTION here: the finalize now
                # runs BEFORE the final reconciliation (order swap), so the
                # only writes since pcb_data was current are this run's own
                # (FILE_LEDGER-verified at the main write). The old guard
                # existed because the reconcile's CLI sub-run wrote file-only
                # copper ahead of the finalize -- that ordering is gone.
                _pcb9 = None
                if progress_callback:
                    _pcb9 = (lambda c, t, m, _o=progress_callback:
                             _o(c, t, f"Plane finalize: {m}"))
                _live9 = None
                if _gui9 or os.environ.get('KICAD_PLANE_FINALIZE_LIVE',
                                           '1') == '1':
                    from kicad_parser import snap_pcb_data_to_iu_grid \
                        as _snap9
                    _snapped9 = _snap9(pcb_data)
                    if _snapped9:
                        print(f"  Plane finalize: snapped {_snapped9} "
                              f"in-memory coordinate(s) onto the nm grid")
                    _live9 = pcb_data
                if _gui9:
                    # In-memory engine leg (GUI): repair appends its kept
                    # copper to pcb_data as REAL objects and its return
                    # branch runs the shared cleanup pipeline (the same
                    # run_post_route_cleanup call clean_plane_copper wraps),
                    # so the separate cleanup leg below is CLI-file-only.
                    # Merge the BOARD DELTA -- not the dict emissions --
                    # into results_data: identity survives into the
                    # reconcile's strip de-dup, and the delta also captures
                    # any strip-and-replace outcome exactly.
                    _b4sid9 = {id(s) for s in pcb_data.segments}
                    _b4vid9 = {id(v) for v in pcb_data.vias}
                    _b4segs9 = list(pcb_data.segments)
                    _b4vias9 = list(pcb_data.vias)
                    _rdp_engine(
                        input_file or "", "", _zn, _zl,
                        track_width=config.track_width,
                        clearance=config.clearance,
                        grid_step=config.grid_step,
                        via_size=config.via_size,
                        via_drill=config.via_drill,
                        hole_to_hole_clearance=config.hole_to_hole_clearance,
                        routing_layers=config.layers,
                        net_clearances=net_clearances,
                        layer_clearances=dict(config.layer_clearances or {}),
                        # #338 (review DRC-1): forward THIS run's RESOLVED
                        # copper-to-edge floor. The engine's own re-resolve
                        # cannot work here: its PLANE_EDGE_CLEARANCE default
                        # (0.5) reads as an explicit override and masks the
                        # project rule, so a board declaring 0.6 got finalize
                        # copper at 0.5 -- real edge DRC.
                        board_edge_clearance=config.board_edge_clearance,
                        # #581: forward the resolved same-net pad via
                        # clearance -- the output's .kicad_pro sibling
                        # does not exist yet mid-run, so the engine's
                        # own auto-read would find nothing.
                        same_net_pad_clearance=(
                            config.same_net_pad_clearance
                            if config.same_net_pad_clearance > 0 else None),
                        # Rip authority (0804-wave finding): the absorbed
                        # repair step ran with --rip-blocker-nets in 143/150
                        # recorded chains, and dropping it left "unroutable
                        # without rip authority" oracle links shipping open
                        # (30 boards / 54 deferrals in the wave). The engine
                        # already skips #521-protected nets as blockers and
                        # custody-restores casualties; KICAD_FINALIZE_RIP=0
                        # reverts for A/B.
                        rip_blocker_nets=_finalize_rip9,
                        power_nets=power_nets,
                        power_nets_widths=power_nets_widths,
                        # #658: the chain's layer economics reach the
                        # finalize legs (previously uniform 1.0 -- welds
                        # traveled priced-up layers for free).
                        layer_costs=list(config.layer_costs or []) or None,
                        pcb_data=pcb_data, return_results=True,
                        progress_callback=_pcb9)
                    _cursid9 = {id(s) for s in pcb_data.segments}
                    _curvid9 = {id(v) for v in pcb_data.vias}
                    _new_s9 = [s for s in pcb_data.segments
                               if id(s) not in _b4sid9]
                    _new_v9 = [v for v in pcb_data.vias
                               if id(v) not in _b4vid9]
                    _rm_s9 = [s for s in _b4segs9 if id(s) not in _cursid9]
                    _rm_v9 = [v for v in _b4vias9 if id(v) not in _curvid9]
                    # A removal naming copper THIS run emitted must drop it
                    # from our write-lists instead of riding the remove
                    # channels (reconcile-merge pattern: the applier's
                    # key-based remove would no-op on a not-yet-added
                    # object and the withdrawn copper would ship).
                    _ours_s9 = {id(s) for _r in results_data.get('results', [])
                                for s in (_r.get('new_segments') or [])}
                    _ours_v9 = {id(v) for _r in results_data.get('results', [])
                                for v in (_r.get('new_vias') or [])}
                    _drop_s9 = {id(s) for s in _rm_s9 if id(s) in _ours_s9}
                    _drop_v9 = {id(v) for v in _rm_v9 if id(v) in _ours_v9}
                    if _drop_s9 or _drop_v9:
                        for _r in results_data.get('results', []):
                            if _drop_s9:
                                _r['new_segments'] = [
                                    s for s in (_r.get('new_segments') or [])
                                    if id(s) not in _drop_s9]
                            if _drop_v9:
                                _r['new_vias'] = [
                                    v for v in (_r.get('new_vias') or [])
                                    if id(v) not in _drop_v9]
                    if _new_s9 or _new_v9:
                        results_data.setdefault('results', []).append({
                            'net_name': '(plane finalize)',
                            'success': True,
                            'new_segments': _new_s9,
                            'new_vias': _new_v9})
                    results_data.setdefault('segments_to_remove', []).extend(
                        s for s in _rm_s9 if id(s) not in _drop_s9)
                    results_data.setdefault('vias_to_remove', []).extend(
                        v for v in _rm_v9 if id(v) not in _drop_v9)
                    print(f"  Plane finalize (GUI): +{len(_new_s9)} seg(s) "
                          f"+{len(_new_v9)} via(s), -{len(_rm_s9)} seg(s) "
                          f"-{len(_rm_v9)} via(s) merged into results")
                else:
                    _rdp_engine(
                        output_file, output_file, _zn, _zl,
                        track_width=config.track_width,
                        clearance=config.clearance,
                        grid_step=config.grid_step,
                        via_size=config.via_size,
                        via_drill=config.via_drill,
                        hole_to_hole_clearance=config.hole_to_hole_clearance,
                        routing_layers=config.layers,
                        net_clearances=net_clearances,
                        # #498: forward THIS run's resolved per-layer map --
                        # the output file's .kicad_dru sibling does not exist
                        # yet, so the engine's own auto-read would find none
                        # and tap/join copper would route blind to the rules.
                        layer_clearances=dict(config.layer_clearances or {}),
                        # #338 (review DRC-1): same reason for the edge floor
                        # -- output_file has no sibling .kicad_pro yet, and
                        # the engine default 0.5 masks the project read.
                        board_edge_clearance=config.board_edge_clearance,
                        # #581: forward the resolved same-net pad via
                        # clearance -- the output's .kicad_pro sibling
                        # does not exist yet mid-run, so the engine's
                        # own auto-read would find nothing.
                        same_net_pad_clearance=(
                            config.same_net_pad_clearance
                            if config.same_net_pad_clearance > 0 else None),
                        # Same rip-authority restoration as the GUI leg above.
                        rip_blocker_nets=_finalize_rip9,
                        power_nets=power_nets,
                        power_nets_widths=power_nets_widths,
                        # #658: same layer economics as the GUI leg above.
                        # 34d2e448 forwarded layer_costs to the _gui9 branch
                        # ONLY, so on the CLI -- every replay, stress and
                        # corpus run -- the finalize's welds/taps/reconnects
                        # still travelled priced-up layers for free, which is
                        # the exact defect that commit set out to close.
                        layer_costs=list(config.layer_costs or []) or None,
                        pcb_data=_live9,
                        progress_callback=_pcb9)
                print(f"  [finalize timing] engine leg: "
                      f"{_time9.time() - _t9:.1f}s")
                if not _gui9:
                    _t9 = _time9.time()
                    from pcb_modification import clean_plane_copper
                    _cs, _cr = clean_plane_copper(
                        output_file, sorted(set(_zn)),
                        config.clearance, config.grid_step)
                    print(f"  [finalize timing] cleanup leg: "
                          f"{_time9.time() - _t9:.1f}s")
                    if _cs or _cr:
                        print(f"  Plane finalize cleanup: closed {_cs} stub "
                              f"gap(s), trimmed {_cr} dead-end segment(s)")
            # ORACLE leg: kicad-cli computes the exact fill, so this leg
            # needs a REAL file on disk.
            #   CLI -- the written output_file.
            #   GUI -- a STAGED file: the caller's live board (stage_board_fn)
            #          with this run's pending copper written onto it, which
            #          is exactly what output_file holds at this point in file
            #          mode. Staging here (rather than deferring to the
            #          applier) is what makes the two fronts agree: the oracle
            #          copper lands BEFORE the final reconciliation and its
            #          unroutable links feed custody, same as the CLI.
            # The base MUST come from stage_board_fn, never from input_file:
            # the GUI's input_file is the ORIGINAL board on disk and is
            # missing whatever earlier chain steps applied in-session
            # (staging from it measured GUI DRC 431 vs CLI 0).
            _orc_file9 = None
            _orc_tmp9 = []
            if _zpairs_all and not _gui9:
                _orc_file9 = output_file
            elif _zpairs_all and _gui9 and stage_board_fn is not None:
                try:
                    import tempfile as _tf9
                    _base9 = stage_board_fn()
                    if _base9:
                        _orc_tmp9.append(_base9)
                        _fd9, _stg9 = _tf9.mkstemp(suffix='.kicad_pcb')
                        os.close(_fd9)
                        _orc_tmp9.append(_stg9)
                        if write_routed_output(
                                input_file=_base9, output_file=_stg9,
                                results=results,
                                all_segment_modifications=(
                                    all_segment_modifications),
                                all_swap_vias=all_swap_vias,
                                all_swap_segments=all_swap_segments,
                                target_swap_info=[],
                                single_ended_target_swap_info=(
                                    single_ended_target_swap_info),
                                pad_swaps=pad_swaps, pcb_data=pcb_data,
                                skip_routing=skip_routing,
                                add_teardrops=add_teardrops,
                                segments_to_remove=results_data.get(
                                    'segments_to_remove'),
                                vias_to_remove=results_data.get(
                                    'vias_to_remove')):
                            _orc_file9 = _stg9
                except Exception as _e9:
                    print(f"  Plane finalize (GUI): could not stage a board "
                          f"for the oracle leg ({_e9}); deferring to apply")
                    _orc_file9 = None
            if _zpairs_all and _gui9 and _orc_file9 is None:
                # FALLBACK only (no stage_board_fn, or staging failed): post
                # the net list + engine-resolved params and let the applier
                # run the staged-save oracle after the copper lands (the
                # planes-tab pattern). Custody cannot merge into the
                # reconcile on this path -- there is no verdict yet.
                _zna = sorted({n for n, _l in _zpairs_all})
                results_data['plane_finalize_oracle'] = {
                    'nets': _zna,
                    'clearance': config.clearance,
                    'track_width': config.track_width,
                    'via_size': config.via_size,
                    'via_drill': config.via_drill,
                    'grid_step': config.grid_step,
                    'hole_to_hole_clearance': config.hole_to_hole_clearance,
                    # #498: the applier's temp save has no .kicad_dru sibling
                    'layer_clearances': dict(config.layer_clearances or {}),
                    # #658 (audit finding): the CLI finalize prices layers for
                    # its oracle leg; this posted param set omitted them, so
                    # the GUI's counterpart built a BARE config and its weld
                    # router ran at UNIFORM layer economics -- welds crossing
                    # a priced-up plane layer for free, and the #658
                    # forbidden-layer guards inert. Class-1 parity gap: a new
                    # engine parameter has to reach BOTH fronts.
                    'layers': list(config.layers or []),
                    'layer_costs': (list(config.layer_costs)
                                    if getattr(config, 'layer_costs', None)
                                    else []),
                    'power_net_widths': dict(
                        getattr(config, 'power_net_widths', None) or {}),
                }
                # Hands-off for the reconcile comes from the FILL-AWARE
                # checker instead of the oracle verdict: zone nets the model
                # says are complete after the engine leg must not be
                # re-touched by the STALE failure buckets (the pf7
                # regression class). Model over-credit only defers a fix to
                # the post-apply oracle -- never breaks a healthy net.
                try:
                    from check_connected import (
                        check_net_connectivity as _cnc9b)
                    _zbn9b = {}
                    for _z9b in (getattr(pcb_data, 'zones', None) or []):
                        _zbn9b.setdefault(_z9b.net_id, []).append(_z9b)
                    for _nid9b, _net9b in pcb_data.nets.items():
                        if _net9b.name not in set(_zna):
                            continue
                        try:
                            _r9b = _cnc9b(
                                _nid9b,
                                [s for s in pcb_data.segments
                                 if s.net_id == _nid9b],
                                [v for v in pcb_data.vias
                                 if v.net_id == _nid9b],
                                pcb_data.pads_by_net.get(_nid9b, []),
                                _zbn9b.get(_nid9b, []), pcb_data=pcb_data)
                            if _r9b.get('connected'):
                                _zone_complete9.add(_net9b.name)
                        except Exception:
                            pass
                except Exception:
                    pass
                if _zone_complete9:
                    print(f"  Plane finalize (GUI): {len(_zone_complete9)} "
                          f"zone net(s) verified complete (fill-aware) -- "
                          f"hands-off for the reconciliation")
            if _orc_file9:
                # Oracle + custody scope: ALL zone nets (see the pre-gate
                # note -- the oracle is the model-independent verifier).
                import time as _time9
                _zna = sorted({n for n, _l in _zpairs_all})
                _t9 = _time9.time()
                from kicad_oracle import oracle_reconnect
                try:
                    from fix_kicad_drc_settings import \
                        effective_board_edge_clearance
                    _oedge = effective_board_edge_clearance(input_file, 0.0)
                except Exception:
                    _oedge = 0.0
                # #658 fifth power path: this config used to be BARE (no
                # layers/layer_costs), so the oracle's weld router ran at
                # UNIFORM layer economics -- on the #589 champion 86% of the
                # In1 power residue (62mm at the 0.2/0.4/0.8 width-upgrade
                # rungs) was weld copper laid straight across the GND plane
                # layer while --layer-costs priced it 6x for everyone else.
                # Costs are soft, so a weld that must touch a plane layer
                # (a GND link onto In1) still can.
                _ocfg = GridRouteConfig(
                    clearance=config.clearance,
                    track_width=config.track_width,
                    via_size=config.via_size, via_drill=config.via_drill,
                    grid_step=config.grid_step,
                    layers=list(config.layers),
                    layer_costs=(list(config.layer_costs)
                                 if getattr(config, 'layer_costs', None)
                                 else []),
                    # #658: power-net membership rides along so the weld
                    # leg's per-net KICAD_POWER_LAYER_COSTS multipliers
                    # (power_layer_config in oracle_reconnect) can fire.
                    power_net_widths=dict(
                        getattr(config, 'power_net_widths', None) or {}),
                    board_edge_clearance=_oedge)
                from kicad_dru import install_layer_clearances
                install_layer_clearances(_ocfg, None, input_file, None)
                # #527 follow-up: the oracle's own per-round / per-link
                # callbacks were already there, but THIS call site never
                # passed one -- so the whole leg ran behind the cleanup
                # pipeline's last message ("Cleanup: soft-joint bridging..."),
                # which reads as a hang. It is the longest silent stretch in
                # the run: on a board where the model and KiCad's exact fill
                # disagree the oracle can spend minutes routing the missing
                # links (act_probe_2ghz: 33 links, ~10k seeded on-pad start
                # cells per blocked endpoint).
                #
                # cancel_check IS threaded, and is custody-safe here by
                # construction (#527's rule: cancel must never abandon a
                # restore half-done). The oracle leg is purely ADDITIVE --
                # it welds links KiCad reports missing -- and its own
                # cancel_check breaks only BETWEEN links/rounds, then returns
                # the normal result dict. So a cancel lands every weld
                # completed so far and drops only the unstarted ones; it can
                # never strand a net at zero copper the way an interrupted
                # rip/restore could.
                _opc9 = None
                if progress_callback:
                    _opc9 = (lambda c, t, m, _o=progress_callback:
                             _o(c, t, f"Plane finalize: {m}"))
                _orc = oracle_reconnect(
                    _orc_file9, _zna, _ocfg,
                    track_via_clearance=defaults.PLANE_TRACK_VIA_CLEARANCE,
                    hole_to_hole_clearance=config.hole_to_hole_clearance,
                    progress_callback=_opc9,
                    cancel_check=cancel_check,
                    project_from=input_file)
                print(f"  [finalize timing] oracle leg: "
                      f"{_time9.time() - _t9:.1f}s")
                # #713 item 3: this leg had NO summary key at all, so an
                # `available: False` -- the authoritative zone-aware check
                # never running -- was invisible to every JSON_SUMMARY
                # consumer, and the run read as fully checked. `oracle_check`
                # describes a DIFFERENT, opt-in end-of-run check and says
                # nothing about this one.
                summary['oracle_reconnect'] = {
                    k: _orc.get(k) for k in
                    ('available', 'reason', 'why', 'rounds', 'links_routed',
                     'links_failed', 'remaining')}
                if not _gui9:
                    # #589: keep the oracle's net list + config for the
                    # post-reconciliation re-audit (CLI file mode only).
                    _reaudit9 = (list(_zna), _ocfg)
                if _gui9:
                    # The staged file is a throwaway: hand the oracle's copper
                    # back through the SAME channels the engine leg uses.
                    # Removals need GEOMETRY-key matching, not id(): the
                    # oracle PARSED the staged temp file, so its
                    # removed_segments/removed_vias are parse products whose
                    # id() can never equal a write-list object's -- an
                    # identity de-dup here is dead code by construction
                    # (found by two independent reviewers; the withdrawn
                    # copper then rode the remove channel, no-op'd against
                    # the not-yet-populated live board because the applier
                    # REMOVES FIRST and adds after, and shipped anyway).
                    # Three channels are handled, all by geometry key:
                    #   1. removal names copper THIS run emitted (results'
                    #      new_segments/new_vias AND the swap channels) ->
                    #      drop it from the write-list;
                    #   2. removal names pre-existing board copper -> ride
                    #      segments_to_remove/vias_to_remove for the applier;
                    #   3. EITHER WAY, mirror the removal into pcb_data so
                    #      the final reconciliation routes against the same
                    #      board the applier will produce (the CLI reconcile
                    #      re-parses the oracle-edited file and gets this
                    #      for free).
                    _os9 = list(_orc.get('new_segments') or [])
                    _ov9 = list(_orc.get('new_vias') or [])
                    _ors9 = list(_orc.get('removed_segments') or [])
                    _orv9 = list(_orc.get('removed_vias') or [])

                    def _skey9(s):
                        a = (round(s.start_x, 4), round(s.start_y, 4))
                        b = (round(s.end_x, 4), round(s.end_y, 4))
                        return (s.net_id, s.layer, min(a, b), max(a, b))

                    def _vkey9(v):
                        return (v.net_id, round(v.x, 4), round(v.y, 4))

                    _rm_skeys9 = {_skey9(s) for s in _ors9}
                    _rm_vkeys9 = {_vkey9(v) for v in _orv9}
                    _matched_skeys9, _matched_vkeys9 = set(), set()
                    if _rm_skeys9 or _rm_vkeys9:
                        _lists9 = [(_r, 'new_segments', 'new_vias')
                                   for _r in results_data.get('results', [])]
                        for _r, _sk, _vk in _lists9:
                            if _rm_skeys9 and _r.get(_sk):
                                _keep = [s for s in _r[_sk]
                                         if _skey9(s) not in _rm_skeys9]
                                if len(_keep) != len(_r[_sk]):
                                    _matched_skeys9.update(
                                        _skey9(s) for s in _r[_sk]
                                        if _skey9(s) in _rm_skeys9)
                                    _r[_sk] = _keep
                            if _rm_vkeys9 and _r.get(_vk):
                                _keep = [v for v in _r[_vk]
                                         if _vkey9(v) not in _rm_vkeys9]
                                if len(_keep) != len(_r[_vk]):
                                    _matched_vkeys9.update(
                                        _vkey9(v) for v in _r[_vk]
                                        if _vkey9(v) in _rm_vkeys9)
                                    _r[_vk] = _keep
                        # Swap channels are this-run copper too (the #284
                        # withdrawn-copper class): a removal naming a swap
                        # via/segment must not ship it either.
                        if _rm_skeys9 and all_swap_segments:
                            _matched_skeys9.update(
                                _skey9(s) for s in all_swap_segments
                                if _skey9(s) in _rm_skeys9)
                            all_swap_segments[:] = [
                                s for s in all_swap_segments
                                if _skey9(s) not in _rm_skeys9]
                        if _rm_vkeys9 and all_swap_vias:
                            _matched_vkeys9.update(
                                _vkey9(v) for v in all_swap_vias
                                if _vkey9(v) in _rm_vkeys9)
                            all_swap_vias[:] = [
                                v for v in all_swap_vias
                                if _vkey9(v) not in _rm_vkeys9]
                        # Channel 3: mirror EVERY removal into pcb_data.
                        pcb_data.segments[:] = [
                            s for s in pcb_data.segments
                            if _skey9(s) not in _rm_skeys9]
                        pcb_data.vias[:] = [
                            v for v in pcb_data.vias
                            if _vkey9(v) not in _rm_vkeys9]
                    # Channel 2: only removals NOT matched to this-run copper
                    # ride the remove channels (the applier removes them from
                    # the live board before adding).
                    _ride_s9 = [s for s in _ors9
                                if _skey9(s) not in _matched_skeys9]
                    _ride_v9 = [v for v in _orv9
                                if _vkey9(v) not in _matched_vkeys9]
                    if _ride_s9 or _ride_v9:
                        results_data.setdefault(
                            'segments_to_remove', []).extend(_ride_s9)
                        results_data.setdefault(
                            'vias_to_remove', []).extend(_ride_v9)
                    if _os9 or _ov9:
                        results_data.setdefault('results', []).append({
                            'net_name': '(plane finalize oracle)',
                            'success': True,
                            'new_segments': _os9, 'new_vias': _ov9})
                        # Also into pcb_data, so the final reconciliation
                        # below prices this copper as a real obstacle and
                        # sees the nets it just completed.
                        pcb_data.segments.extend(_os9)
                        pcb_data.vias.extend(_ov9)
                    print(f"  Plane finalize oracle (GUI): +{len(_os9)} "
                          f"seg(s) +{len(_ov9)} via(s), -{len(_ors9)} seg(s) "
                          f"-{len(_orv9)} via(s) merged into results "
                          f"({len(_ors9) - len(_ride_s9)} seg / "
                          f"{len(_orv9) - len(_ride_v9)} via removal(s) "
                          f"matched this run's own copper)")
                try:
                    import json as _json9
                    print('JSON_ORACLE: ' + _json9.dumps(
                        {k: v for k, v in _orc.items()
                         if k not in ('new_segments', 'new_vias')}))
                except Exception:
                    pass
                # Custody (the reason this runs IN-RUN): links the oracle's
                # own router leaves flagged after 3 rounds failed WITHOUT
                # rip authority -- its link-router cannot rip a signal net
                # sitting on the only corridor. Their nets MERGE into the
                # final reconciliation below (order swap: finalize first,
                # ONE sub-run for failed signal nets + stubborn plane nets
                # -- saves a whole parse + base-build self-invocation, and
                # the reconcile then retries signals against the WELDED
                # planes, pours-first-consistent).
                _rl9 = _orc.get('remaining_links')
                # `remaining` counts links that no copper can ever join
                # (cross-board) as well, so gate custody on the FILTERED
                # detail when we have it -- otherwise a multi-outline board
                # pays a doomed reconcile (with rip authority) every run.
                # remaining_links is None ONLY when the link detail was never
                # obtained (source failure / cancel), where `remaining` may
                # be a stale count: fall back to all zone nets there, but
                # never on an empty list, which honestly means "none left".
                _has_custody = (_rl9 is None and _orc.get('remaining', 0) > 0) \
                    or bool(_rl9)
                if _orc.get('available') and _has_custody:
                    # Custody scope = exactly the nets with remaining links
                    # (ux pf7: the 6 stubborn links were ALL GND; putting
                    # every zone net in custody re-touched gate-connected
                    # +3V3 and broke it).
                    _custody_nets9 = (sorted({l[0] for l in _rl9})
                                      if _rl9 else list(_zna))
                    # #572: keep the EXACT links -- the reconcile routes
                    # them as forced edges; net-level retry alone is
                    # structurally blind to them (the fill-model zone
                    # credit both skips the net as "Already fully
                    # connected" and, past the skip, derives no missing
                    # edge -- a 0-copper vacuous success, measured on
                    # ghoul GND).
                    # CLASSIFY before welding (#659), on the DEFAULT path.
                    # The same rule the opt-in summary check applies: a
                    # remaining link whose endpoint sits on pad-less copper
                    # is DEBRIS, and handing it to the reconcile as a forced
                    # edge routes copper to dead metal. The oracle's own
                    # debris pass is meant to delete such copper, but its
                    # locator cannot see a via-only cluster, so those links
                    # arrive here intact. Deletion is handled by the late
                    # orphan sweep; what matters here is not WELDING them.
                    _custody_links9 = list(_rl9 or [])
                    try:
                        from check_connected import (
                            classify_unconnected_link as _cul9)
                        _name2id9 = {n.name: i
                                     for i, n in pcb_data.nets.items()}
                        _live9, _dead9 = [], 0
                        for _lk9 in _custody_links9:
                            _lid9 = _name2id9.get(_lk9[0])
                            if _lid9 is None:
                                _live9.append(_lk9)
                                continue
                            _ca9, _cb9 = _cul9(
                                pcb_data, _lid9, _lk9[1], _lk9[2],
                                _lk9[1][3] if len(_lk9[1]) > 3 else None,
                                _lk9[2][3] if len(_lk9[2]) > 3 else None)
                            if 'padless' in (_ca9, _cb9):
                                _dead9 += 1
                            else:
                                _live9.append(_lk9)
                        if _dead9:
                            print(f"  Plane finalize: {_dead9} of "
                                  f"{len(_custody_links9)} remaining link(s) "
                                  f"point at PAD-LESS copper (#659) -- not "
                                  f"weld targets; the late orphan sweep "
                                  f"removes them")
                            _custody_links9 = _live9
                            # A net left with no live link has nothing for
                            # the reconcile to route. Drop it from custody --
                            # but do NOT add it to _zone_complete9 below,
                            # which asserts KiCad verified it complete. It
                            # did not; its debris is simply not routable.
                            _custody_nets9 = sorted({l[0] for l in _live9}) \
                                if _live9 else []
                    except Exception as _ce9:
                        print(f"  (finalize link classification skipped: "
                              f"{_ce9})")
                    # Zone nets KiCad verified COMPLETE are hands-off for
                    # the reconcile: the main run's failure buckets are
                    # STALE after the finalize fixes nets.
                    _zone_complete9 = set(_zna) - set(_custody_nets9) \
                        - {l[0] for l in (_rl9 or [])}
                    print(f"  Plane finalize: {_orc['remaining']} oracle "
                          f"link(s) unroutable without rip authority on "
                          f"{', '.join(_custody_nets9)} -- joining the "
                          f"final reconciliation")
                else:
                    # Oracle verdict: every zone net complete -- none of
                    # them may be re-touched by the stale failure buckets.
                    if _orc.get('available'):
                        _zone_complete9 = set(_zna)
        except Exception as _e:
            # The failure must name its blast radius: JSON_SUMMARY was
            # already printed BEFORE the finalize, so a grader reading it
            # sees the pre-finalize tally -- nets the raster model
            # over-credits (castor class) ship open with no failure record
            # unless the operator sees THIS line. (Review finding F3.)
            _unverified9 = sorted({n for n, _l in (locals().get('_zpairs_all')
                                                   or [])})
            print(f"{RED}  plane finalize pass FAILED: {_e}\n"
                  f"  Zone net(s) left UNVERIFIED by the kicad-oracle: "
                  f"{', '.join(_unverified9) if _unverified9 else '(none in scope)'} "
                  f"-- the printed JSON_SUMMARY predates the finalize and "
                  f"may over-credit these nets; re-verify with "
                  f"check_connected/check_drc before trusting this board."
                  f"{RESET}")
        finally:
            _finalize_depth(-1)
            # Drop the GUI oracle's staging files (locals() guard: the names
            # only exist once the finalize body got that far).
            for _p9 in (locals().get('_orc_tmp9') or []):
                # ...and its SIBLINGS (audit finding): write_routed_output
                # seeds a .kicad_pro next to the staged board, which this
                # loop never removed -- one leaked temp per GUI route with
                # planes. gui_utils already does this via copy_board's
                # shared SIBLING_EXTS; use the same list rather than a
                # second hand-rolled one.
                try:
                    from copy_board import SIBLING_EXTS as _sib9
                except Exception:
                    _sib9 = ('.kicad_pro', '.kicad_prl', '.kicad_dru',
                             '.design-brief.json')
                for _q9 in (_p9,) + tuple(os.path.splitext(_p9)[0] + _e
                                          for _e in _sib9):
                    try:
                        os.unlink(_q9)
                    except OSError:
                        pass

    # #655: the zero-copper scan must run BEFORE the reconcile gate --
    # when the only casualties are phase-3 victims outside every bucket,
    # the buckets alone would skip the reconcile entirely.
    _zero_pre9 = []
    if final_reconcile and not skip_routing and not _ckpt_stop:
        _copper_pre9 = ({s.net_id for s in pcb_data.segments}
                        | {v.net_id for v in pcb_data.vias})
        _zone_pre9 = {z.net_id for z in (pcb_data.zones or [])}
        _scope_pre9 = {nid for _n, nid in net_ids}
        _zero_pre9 = sorted(
            net.name for nid, net in pcb_data.nets.items()
            if nid in _scope_pre9 and nid not in _copper_pre9
            and nid not in _zone_pre9 and len(net.pads) >= 2)
    if (final_reconcile and not skip_routing and not _ckpt_stop
            and (output_file or return_results)
            and (failed_single or failed_multipoint or _custody_nets9
                 or _victim_retry_names or open_single or _zero_pre9)):
        # #562 order swap: stubborn-oracle-link plane nets (custody) merge
        # into THIS sub-run instead of a second self-invocation -- one
        # parse/base-build serves both, and signal retries now run against
        # the welded planes.
        _rec_names = list(dict.fromkeys(
            n for n in (failed_single
                        + [m['net_name'] for m in failed_multipoint]
                        + _custody_nets9
                        + _victim_retry_names
                        # open_single: a KEPT result whose pads are still
                        # disconnected (non-multipoint). The bucket CLAUDE.md
                        # warns contributes to NEITHER failure term -- and it
                        # never fed the reconcile either, so such a net got
                        # no retry at all (found in the 2026-08-05 victim
                        # audit; same nobody's-responsibility shape).
                        + open_single)
            if n not in _zone_complete9))
        # #655 zero-copper backstop: a phase-3 tap-rip victim whose FINAL
        # reroute never landed ships with NO copper while every ledger
        # still says 'routed' (terminal_restores empty, failed_single
        # empty, story status stale) -- the nobody's-responsibility
        # bucket. Catch it by MEASUREMENT, not bookkeeping: any in-scope
        # multi-pad net with zero segments AND zero vias at reconcile
        # time gets enrolled. Zone-owning nets are exempt (a pour-served
        # plane net is legitimately trackless).
        _known9 = set(_rec_names)
        _zero9 = [n for n in _zero_pre9
                  if n not in _known9 and n not in _zone_complete9]
        if _zero9:
            print(f"  Zero-copper backstop (#655): {len(_zero9)} net(s) "
                  f"with NO copper yet absent from every failure bucket "
                  f"-- enrolling: {', '.join(_zero9)}")
            _rec_names.extend(_zero9)
        if os.environ.get('KICAD_SCOPE_DEBUG'):
            print(f"  [scope-dbg] reconcile buckets: "
                  f"failed_single={len(failed_single)} "
                  f"failed_multipoint={len(failed_multipoint)} "
                  f"custody={len(_custody_nets9)} "
                  f"victims={len(_victim_retry_names)} "
                  f"open_single={len(open_single)} "
                  f"zero={len(_zero9)}")
            for _m in failed_multipoint[:6]:
                print(f"  [scope-dbg] fm entry: {_m['net_name']} "
                      f"oracle={_m.get('oracle_only', False)} "
                      f"pads={len(_m.get('failed_pads', []))}")
        print(f"\nFinal reconciliation: retrying {len(_rec_names)} "
              f"incomplete/custody net(s) against the finished board: "
              f"{', '.join(_rec_names)}")
        try:
            _rk = dict(_reconcile_kwargs)
            # force_reroute must NOT forward: the strip already happened in
            # THIS run; a forwarded flag would re-strip the retried nets'
            # partial copper (and thrash on a second failure).
            _rk.update(final_reconcile=False, skip_routing=False,
                       force_reroute=False, rip_preexisting=False)
            # #572 (fix direction 2): hand the oracle's EXACT unroutable
            # links to the sub-run as forced edges. Without them the
            # sub-run's model-credited connectivity both skips the custody
            # net ("Already fully connected") and, past the skip, derives no
            # missing edge (0-copper vacuous success) -- the link ships open
            # while every retry claims success. With them the sub-run routes
            # the exact endpoints, and a blocked link feeds the standard
            # frontier analysis -> rip ladder -> retries.
            # #659: the oracle summary check's GENUINE opens ride the same
            # channel as the plane finalize's custody links. Without them the
            # sub-run's own model both skips the net ("Already fully
            # connected" -- the very verdict KiCad just contradicted) and,
            # past the skip, derives no missing edge: the net was enrolled,
            # retried, and reported success while shipping the same open.
            _forced659 = list(_custody_links9)
            if _oracle_open_links:
                _rec_set659 = set(_rec_names)
                _fresh659 = [l for l in _oracle_open_links
                             if l[0] in _rec_set659]
                if _fresh659:
                    print(f"  Oracle-open forced links (#659): "
                          f"{len(_fresh659)} exact endpoint pair(s) on "
                          f"{', '.join(sorted({l[0] for l in _fresh659}))}")
                    _forced659.extend(_fresh659)
            if _forced659:
                _rk['oracle_links'] = _forced659
            # #527 follow-up: the inner run forwards the SAME progress
            # callback, so its routing/rescue/cleanup messages were pixel-
            # identical to the first pass's and the GUI looked like it ran
            # the whole process twice. Prefix them.
            _outer_pcb = _rk.get('progress_callback')
            if _outer_pcb:
                _rk['progress_callback'] = (
                    lambda c, t, m, _o=_outer_pcb: _o(
                        c, t, f"Final reconcile: {m}"))
            # Rip-authority escalation (#103 self-applied): nets that died with
            # 'no rippable blockers found' were boxed by PRE-EXISTING copper
            # this run may not touch, and the router itself printed the
            # --rip-existing-nets retry it wanted. Take that advice in-run:
            # grant the reconciliation rip authority over exactly the
            # frontier-attributed blocker names recorded in the failed nets'
            # histories (capped; rip authority is permission, not compulsion
            # -- the rip-up ladder only fires where a route is actually
            # blocked, and ripped pre-existing nets go through the standard
            # reroute-or-restore custody). Existing patterns are kept.
            _rec_ids = {nid for nid, net in pcb_data.nets.items()
                        if net.name in set(_rec_names)}
            # #651 kill switch: on a nearly-complete board the escalation's
            # hinted blockers are THIS RUN's successes (the RAM-bus
            # massacre: 3 failures in, 13 zero-copper ships out).
            # KICAD_RECONCILE_RIP_ESCALATION=0 disables the self-grant.
            _esc_on = os.environ.get('KICAD_RECONCILE_RIP_ESCALATION',
                                     '1') != '0'
            _hinted = []
            for _nid in _rec_ids:
                for _ev in (state.net_history.get(_nid) or []):
                    if _ev.get('event') == 'preexisting_blockers':
                        for _bn in (_ev.get('details') or {}).get('blockers') or []:
                            if _bn not in _hinted:
                                _hinted.append(_bn)
            _RIP_ESCALATION_CAP = 12
            # Auto-granted rip authority must respect the caller's own net
            # filter: a net excluded by pattern ('!GND' while planes route
            # in a later step) is excluded BY PLAN, and ripping its stubs
            # here reroutes the whole net as track copper in a step that was
            # told not to touch it (ottercast: 52 dogbone stubs became a
            # 757-segment GND web). Explicit --rip-existing-nets from the
            # operator is honored as given; only the escalation filters.
            def _filter_rip_hints103(_names):
                # GUARD-based, not scope-based (2026-08-06): the old filter
                # dropped every blocker outside --nets, which killed legit
                # authority over ordinary SIGNAL blockers (ecp5 /PF37-: a
                # fresh run on the final board closes it via candidacy, but
                # the sub-run -- candidacy off, hints scope-filtered --
                # could not). The ottercast hazard is specifically nets the
                # caller NEGATED ('!GND' stubs -> 757-seg web) and
                # plane/zone-backed nets; keep excluding those, plus a size
                # cap so the sub-run's first-class reroute of the ripped
                # blocker is near-certain. Protected/locked names are
                # dropped later by the sub-run's own filter_rippable_names.
                # (Shared by the initial #103 escalation and the #572
                # lap-authority harvest below.)
                if not (_names and net_names):
                    return list(_names or [])
                from net_queries import (matches_net_filter as _mnf,
                                         split_net_patterns as _snp)
                # PROTECTED names are dropped here UNCONDITIONALLY: the
                # #521 override door ("an exact name overrides") exists for
                # DELIBERATE operator naming, and #103 hints are machine-
                # generated exact names -- without this check a hinted
                # protected blocker (ecp5: /HR_CK_N, a protected pair leg
                # AND a historical blocker in the same window) would ride
                # the override door. Locked copper already has no override
                # anywhere.
                from protected_nets import protection_map as _pm103
                _prot103 = _pm103(pcb_data, input_file)
                _all_names103 = {n.name for n in pcb_data.nets.values()}
                _, _neg103 = _snp(net_names, _all_names103)
                from net_queries import net_pattern_matches as _npm103
                _zone_ids103 = {z.net_id
                                for z in (getattr(pcb_data, 'zones', None)
                                          or [])}
                _keep103 = []
                for _bn in _names:
                    if _bn in _prot103:
                        continue                       # protected: NEVER
                    if _mnf(_bn, net_names):
                        _keep103.append(_bn)          # in scope: as before
                        continue
                    if any(_npm103(_bn, _p) for _p in _neg103):
                        continue                       # negated BY PLAN
                    _bid = next((i for i, nn in pcb_data.nets.items()
                                 if nn.name == _bn), None)
                    if _bid is None or _bid in _zone_ids103:
                        continue                       # plane/zone-backed
                    _bsegs = sum(1 for s in pcb_data.segments
                                 if s.net_id == _bid)
                    _bvias = sum(1 for v in pcb_data.vias
                                 if v.net_id == _bid)
                    if _bsegs > 30 or _bvias > 6:
                        continue                       # reroute not certain
                    _keep103.append(_bn)
                return _keep103

            _hinted = _filter_rip_hints103(_hinted)

            def _harvest_lap_authority572():
                """#572 lap authority: forced-link failures INSIDE the
                sub-run hint pre-existing blockers the outer history can
                never name (custody plane nets never enter the outer SE
                loop, so the #103 escalation above has no hint source for
                them -- measured on eis/ddr5: the exact link fails against
                a pre-existing signal wall with the authority hint printed,
                and dies unheard). The sub-run exports those hints via the
                in-process channel; guard-filter them identically and grant
                --rip-existing-nets for ONE more lap. Returns True when new
                authority was granted (the lap loop re-laps on it).

                Also prunes links of LANDED nets from the next lap's
                oracle_links: a welded net must neither re-bypass the
                model-credit skip nor re-route (duplicate) its strap."""
                _landed = set(getattr(batch_route, '_forced_link_landed',
                                      None) or [])
                if _landed and _rk.get('oracle_links'):
                    _kept572 = [l for l in _rk['oracle_links']
                                if l and l[0] not in _landed]
                    if len(_kept572) != len(_rk['oracle_links']):
                        _rk['oracle_links'] = _kept572
                _flh = dict(getattr(batch_route, '_forced_link_hints',
                                    None) or {})
                # #651's kill switch has to cover THIS self-grant too (audit
                # finding): KICAD_RECONCILE_RIP_ESCALATION=0 was honoured for
                # the #103 hint list but not here, so a run with the
                # escalation "disabled" could still grant itself rip
                # authority for one more lap. Given what motivated the
                # switch -- the RAM-bus massacre, 3 failures in and 13
                # zero-copper ships out -- a half-open kill switch is worse
                # than none, because it reads as off.
                if not _esc_on:
                    return False
                if not _flh or '*' in (_rk.get('rip_existing_nets') or []):
                    return False
                _have = set(_rk.get('rip_existing_nets') or [])
                _fresh = [n for names in _flh.values() for n in names
                          if n not in _have]
                _fresh = _filter_rip_hints103(
                    list(dict.fromkeys(_fresh)))[:_RIP_ESCALATION_CAP]
                if not _fresh:
                    return False
                _rk['rip_existing_nets'] = list(dict.fromkeys(
                    list(_have) + _fresh))
                print(f"  Reconciliation lap authority (#572): rip "
                      f"authority over forced-link blockers "
                      f"{', '.join(_fresh)} -- one more lap")
                return True

            if not _esc_on and _hinted:
                print(f"  Reconciliation rip-escalation DISABLED "
                      f"(KICAD_RECONCILE_RIP_ESCALATION=0): dropping "
                      f"{len(_hinted)} hinted blocker(s)")
                _hinted = []
            if _hinted and '*' not in (rip_existing_nets or []):
                _hinted = _hinted[:_RIP_ESCALATION_CAP]
                _rk['rip_existing_nets'] = list(dict.fromkeys(
                    (rip_existing_nets or []) + _hinted))
                print(f"  Reconciliation rip authority (#103): "
                      f"--rip-existing-nets over hinted blockers "
                      f"{', '.join(_hinted)}")
            # #513 item 5: the reconciliation re-routes each incomplete net at
            # the forwarded kwargs' width resolution -- which silently dropped
            # a ripped power net's 1.5mm width to the 0.25 default when this
            # invocation carried no --power-nets (nascom VCC). Preserve each
            # reconciled net's ORIGINAL dominant width (from the pre-routing
            # input snapshot) via the power-width channel; an explicit
            # --power-nets entry for the net still wins.
            from routing_common import dominant_net_widths as _dnw5
            _orig_w5 = _dnw5(_s for _lst in _orig_seg_by_net.values()
                             for _s in _lst)
            _pn5 = list(_rk.get('power_nets') or [])
            _pw5 = list(_rk.get('power_nets_widths') or [])
            if len(_pn5) == len(_pw5):
                from net_queries import matches_net_filter as _mnf5
                _tw5 = _rk.get('track_width') or config.track_width
                for _nid5 in _rec_ids:
                    _w5 = _orig_w5.get(_nid5, 0.0)
                    _nm5 = (pcb_data.nets[_nid5].name
                            if _nid5 in pcb_data.nets else None)
                    if not _nm5 or _w5 <= (_tw5 or 0.0) + 1e-6:
                        continue
                    if _pn5 and _mnf5(_nm5, _pn5):
                        continue  # explicit power width wins
                    _pn5.append(_nm5)
                    _pw5.append(_w5)
                    print(f"  Preserving routed width {_w5}mm for reconciled "
                          f"net {_nm5} (original copper was wider than this "
                          f"run's {_tw5}mm default)")
                if _pn5:
                    _rk['power_nets'] = _pn5
                    _rk['power_nets_widths'] = _pw5
            if return_results:
                # Bounded PROGRESS-LOOP -- GUI leg (2026-08-06): SAME laps
                # as the CLI branch below, entirely in-memory (no temp
                # board files): each lap re-snaps pcb_data to the nm grid
                # (the eth_tap parity fix) so a lap sees the bit-identical
                # board the CLI lap re-parses, then merges its own _rdata
                # through the standard channels. Identical loop condition
                # and cap => identical copper on both fronts.
                _rok = 0
                _rfail = 0
                _rt = 0.0
                for _lap10 in range(3):
                    # GUI-parity reconciliation (gap-closure): re-invoke against
                    # the SAME in-memory board (the copper this run just
                    # committed lives in pcb_data, not in any file) and merge
                    # the sub-run's results into ours. An inner strip that
                    # targets copper THIS run emitted must instead drop it from
                    # our write-lists (the GUI applier REMOVES FIRST and adds
                    # after -- swig_gui runs the remove channels before the add
                    # loop -- so a strip of a not-yet-added segment would no-op
                    # against the live board and the deleted copper would ship).
                    #
                    # SNAP FIRST. The CLI branch below reconciles against the
                    # WRITTEN file, whose coordinates are 100% on KiCad's integer-nm
                    # grid because the writer quantises on the way out. The router
                    # works in float mm, so this in-memory board is NOT: ~20% of its
                    # coordinates sit up to 0.49 nm off-grid (eth_tap step 11:
                    # 2813/13692). Reconciling against an un-snapped board means the
                    # two fronts retry against DIFFERENT boards, so `return_results`
                    # -- which should only decide whether results are RETURNED --
                    # changed what got ROUTED: 3423 (CLI) vs 3428 (GUI) segments on
                    # identical input and identical kwargs. Snapping here makes this
                    # board bit-identical to the file the CLI would have re-parsed.
                    # Geometrically a no-op (max deviation 0.4878 nm < the 0.5 nm
                    # rounding threshold, so every value lands on the same integer
                    # nm the writer emits).
                    from kicad_parser import snap_pcb_data_to_iu_grid
                    _snapped = snap_pcb_data_to_iu_grid(pcb_data)
                    if _snapped:
                        print(f"  Reconciliation: snapped {_snapped} in-memory "
                              f"coordinate(s) onto the nm grid (CLI-file parity).")
                    _rk.update(return_results=True, pcb_data=pcb_data)
                    _lrok, _lrfail, _lrt, _rdata = batch_route(
                        input_file, "", _rec_names, **_rk)
                    _rok += _lrok
                    _rfail = _lrfail
                    _rt += _lrt
                    _our_new_segs = set()
                    for _r in results_data.get('results', []):
                        for _s in (_r.get('new_segments') or []):
                            _our_new_segs.add(id(_s))
                    _inner_strips = _rdata.get('segments_to_remove') or []
                    _strip_ours = {id(_s) for _s in _inner_strips
                                   if id(_s) in _our_new_segs}
                    if _strip_ours:
                        for _r in results_data.get('results', []):
                            _r['new_segments'] = [
                                _s for _s in (_r.get('new_segments') or [])
                                if id(_s) not in _strip_ours]
                    results_data.setdefault('results', []).extend(
                        _rdata.get('results', []))
                    results_data.setdefault('segments_to_remove', []).extend(
                        _s for _s in _inner_strips if id(_s) not in _strip_ours)
                    # #484 H2: mirror the segment "ours" de-dup for vias -- an
                    # inner strip naming a via THIS run emitted must drop it from
                    # our write-lists, not ride vias_to_remove (the GUI applier
                    # removes before adding, so a positional collision could
                    # leave the emitted via shipped un-removed).
                    _our_new_vias = set()
                    for _r in results_data.get('results', []):
                        for _v in (_r.get('new_vias') or []):
                            _our_new_vias.add(id(_v))
                    _inner_vstrips = _rdata.get('vias_to_remove') or []
                    _vstrip_ours = {id(_v) for _v in _inner_vstrips
                                    if id(_v) in _our_new_vias}
                    if _vstrip_ours:
                        for _r in results_data.get('results', []):
                            _r['new_vias'] = [
                                _v for _v in (_r.get('new_vias') or [])
                                if id(_v) not in _vstrip_ours]
                    if _inner_vstrips:
                        results_data.setdefault('vias_to_remove', []).extend(
                            _v for _v in _inner_vstrips
                            if id(_v) not in _vstrip_ours)
                    # #484 H2: pad_swaps / single_ended_target_swap_info were
                    # silently dropped -- a target/polarity swap performed by the
                    # reconciliation sub-run then never reached the GUI applier,
                    # and the reconciled net's stub pointed at the OLD net.
                    for _key in ('all_swap_vias', 'all_swap_segments',
                                 'all_segment_modifications', 'pad_swaps',
                                 'single_ended_target_swap_info',
                                 'exclusion_zone_lines', 'boundary_debug_labels'):
                        if _rdata.get(_key):
                            results_data.setdefault(_key, []).extend(_rdata[_key])
                    # SNAP AGAIN. The sub-run above routed in float mm too, so the
                    # copper it just merged is back off-grid (8 coordinates on
                    # eth_tap step 11) even though we snapped before invoking it.
                    # The CLI's equivalent copper goes through the writer and lands
                    # grid-exact, so without this the fork reopens on exactly the
                    # nets the reconcile pass touched. Snap pcb_data AND the
                    # results the GUI applier will read -- they are usually the
                    # same Segment objects, but not contractually.
                    snap_pcb_data_to_iu_grid(pcb_data)
                    for _r in results_data.get('results', []):
                        _rsnap = type('_P', (), {})()
                        _rsnap.segments = _r.get('new_segments') or []
                        _rsnap.vias = _r.get('new_vias') or []
                        snap_pcb_data_to_iu_grid(_rsnap)
                    # #572: same lap-authority rule as the CLI leg below.
                    _granted572 = bool(_lrfail) and _harvest_lap_authority572()
                    if not (_lrfail and (_lrok or _granted572)):
                        break
                    print(f"  Reconciliation progress-loop: lap "
                          f"{_lap10 + 1} recovered {_lrok} with "
                          f"{_lrfail} still failed -- one more lap on "
                          f"the updated board")
            else:
                _rk.update(return_results=False)
                # Bounded PROGRESS-LOOP (2026-08-06, Andy's step7-vs-step8
                # observation): each retry reshuffles the board enough that
                # the NEXT attempt can win -- the recorded chains' manual
                # retry steps were lap 2 by hand. Re-lap the SAME sub-run
                # while the previous lap recovered something and targets
                # remain failed (cap 3 laps). Cheap by construction: the
                # sub-run re-parses the updated board and already-closed
                # targets short-circuit as 'Already fully connected'; no
                # grading, no snapshots, no temp board files.
                _rok = 0
                _rfail, _rt = 0, 0.0
                for _lap10 in range(3):
                    _lrok, _lrfail, _lrt = batch_route(
                        output_file, output_file, _rec_names, **_rk)
                    _rok += _lrok
                    _rfail = _lrfail
                    _rt += _lrt
                    # #572: fresh forced-link blocker hints justify a lap
                    # even without recoveries -- lap N failed against a
                    # named pre-existing wall this authority now opens.
                    _granted572 = bool(_lrfail) and _harvest_lap_authority572()
                    if not (_lrfail and (_lrok or _granted572)):
                        break
                    print(f"  Reconciliation progress-loop: lap "
                          f"{_lap10 + 1} recovered {_lrok} with {_lrfail} "
                          f"still failed -- one more lap on the updated "
                          f"board")
            print("Note: the JSON_SUMMARY above covers only the "
                  "reconciliation subset (it carries scope="
                  "\"reconciliation-subset\"); the run's full tally is the "
                  "earlier JSON_SUMMARY, the one with scope=\"run\", plus "
                  "these recoveries. Never scrape the LAST JSON_SUMMARY of a "
                  "route log -- count them, or read the scope.")
            if _rok:
                successful += _rok
                failed = max(0, failed - _rok)
        except Exception as _e:
            _RECONCILE_RAISED[0] = True
            print(f"{RED}  final reconciliation pass failed: {_e}{RESET}")



    # #589 post-reconciliation oracle RE-AUDIT (KICAD_FINALIZE_REAUDIT=1,
    # experiment): the finalize's oracle leg audits BEFORE the final
    # reconciliation lays its forced-link/rescue copper WITH RIP AUTHORITY,
    # so rip-restore damage to plane nets lands after the audit stopped
    # looking (measured on orangecrab: GND had 3 links at audit time and 7
    # at ship -- the P-rail tap retries ripped GND mid-finalize and the
    # restores were incomplete). One bounded extra oracle pass over the
    # SAME nets on the final written board welds exactly that damage; on a
    # healthy board it costs one refill and exits at round 0.
    if (_reaudit9 is not None and output_file and not return_results
            and not skip_routing
            and os.environ.get('KICAD_FINALIZE_REAUDIT', '0') == '1'):
        try:
            from kicad_oracle import oracle_reconnect as _orc_fn10
            # #659: widen the re-audit scope from zone nets to EVERY net
            # KiCad reports unconnected within this run's scope -- signal
            # micro-gap opens (the IO_SDA class: model-credited, KiCad-
            # rejected) previously had no in-run owner. Full-airwire links
            # on failed nets will fail the weld honestly (bounded rounds);
            # the sub-mm gap class is exactly what the weld router closes.
            _scope10 = set(_reaudit9[0])
            try:
                from kicad_oracle import find_kicad_cli as _fkc10, \
                    kicad_unconnected as _ku10
                _cli10 = _fkc10()
                _links10 = _ku10(output_file, _cli10) if _cli10 else None
                if _links10:
                    _run_names10 = {n for n, _i in net_ids}
                    _extra10 = sorted({lk[0] for lk in _links10}
                                      & _run_names10 - _scope10)
                    if _extra10:
                        print(f"  re-audit scope +{len(_extra10)} "
                              f"KiCad-flagged signal net(s) (#659): "
                              f"{', '.join(_extra10[:12])}"
                              + ("..." if len(_extra10) > 12 else ""))
                        _scope10 |= set(_extra10)
            except Exception as _se10:
                print(f"  (#659 scope widen failed: {_se10})")
            print("\nPost-reconciliation oracle re-audit (#589/#659): "
                  "re-checking flagged nets on the final board...")
            _orc10 = _orc_fn10(
                output_file, sorted(_scope10), _reaudit9[1],
                track_via_clearance=defaults.PLANE_TRACK_VIA_CLEARANCE,
                hole_to_hole_clearance=config.hole_to_hole_clearance,
                project_from=input_file)
            try:
                results_data['post_reconcile_oracle'] = _orc10
            except (NameError, UnboundLocalError):
                pass  # results_data only exists on the GUI path
        except Exception as _e10:
            print(f"  post-reconciliation re-audit failed: {_e10}")

    _late_orphan_sweep659(
        pcb_data, output_file, return_results,
        locals().get('results_data'), _protect_unfinished, keep_input_copper,
        skip_routing)

    # Per-net story dump (KICAD_NET_STORY=1): the complete journey of every
    # net -- bus membership, ordering, failures with named blockers, rips,
    # rescues, Phase-3 tap order, costs -- assembled from state.
    if json_out:
        try:
            from route_summary import merge_summaries
            _merged = merge_summaries(list(_SUMMARY_SINK), _RECONCILE_RAISED[0])
            with open(json_out, 'w', encoding='utf-8') as _jf:
                json.dump(_merged if _merged is not None else {}, _jf, indent=1)
            print(f"  route summary written to {json_out}")
        except Exception as _e:
            print(f"  WARNING: could not write --json-out {json_out}: {_e}")

    from net_story import net_story_enabled, dump_net_story
    if net_story_enabled():
        try:
            dump_net_story(state, output_file or input_file)
        except Exception as _e:
            print(f"  net story dump failed: {_e}")

    # Route trace dump (KICAD_ROUTE_TRACE=1): the per-copper add/rip/restore
    # timeline recorded at the choke points, for animate_route.py (#482).
    from route_trace import dump_trace as _dump_route_trace
    _dump_route_trace(state.pcb_data, output_file)

    # KICAD_DUP_TRAP=1: final verdict. Says whether the traps SURVIVED the
    # run -- a rebind swaps in a plain list and disarms them, so a bare
    # "0 hits" is only meaningful once this confirms they were still armed.
    from dup_trap import verify as _verify_dup_trap
    _verify_dup_trap(pcb_data)

    # #572 lap-authority channel: export the pre-existing-blocker hints of
    # forced-link nets THIS run still failed, so the caller's reconcile lap
    # loop can guard-filter them into --rip-existing-nets and re-lap. An
    # in-process function attribute (same lifetime discipline as the
    # finalize depth global); {} whenever this run had no oracle links or
    # every forced-link net landed.
    try:
        _flh_out = {}
        _fll_out = []
        if oracle_links and state.oracle_links_by_net:
            for _fnid in state.oracle_links_by_net:
                _fnm = (pcb_data.nets[_fnid].name
                        if _fnid in pcb_data.nets else None)
                if _fnid in (state.routed_results or {}):
                    # Landed: the caller prunes this net's links so the
                    # next lap neither re-bypasses the skip nor re-routes
                    # (duplicates) the strap.
                    if _fnm:
                        _fll_out.append(_fnm)
                    continue
                _fnames = []
                for _ev in (state.net_history.get(_fnid) or []):
                    if _ev.get('event') == 'preexisting_blockers':
                        for _bn in (_ev.get('details') or {}).get('blockers') or []:
                            if _bn not in _fnames:
                                _fnames.append(_bn)
                if _fnm and _fnames:
                    _flh_out[_fnm] = _fnames
        batch_route._forced_link_hints = _flh_out
        batch_route._forced_link_landed = _fll_out
    except Exception:
        batch_route._forced_link_hints = {}
        batch_route._forced_link_landed = []

    # ---- IMPROVEMENT GATE (#600) -------------------------------------------
    # Never ship a board this run made WORSE. route.py may rip already-routed
    # copper from two entry points -- the in-run finalize/reconciliation
    # (clearing a blocker, placing a plane tap) and --rip-existing-nets retry
    # rounds -- and a rip whose restore is refused leaves that net broken. The
    # damage was reported honestly and the board written anyway: in the
    # sets-21-27 wave that was the largest single source of lost connectivity,
    # bigger than routing failure (bms_sensor turned a 3-pad problem into a
    # 20-pad one; spartan6_4layer lost 20 nets all of their copper). Detection
    # was never the missing half -- ROLLBACK was.
    #
    # It runs HERE, at the very end, on the WRITTEN file: that is the run's
    # true final artifact, after the finalize AND every reconciliation lap, so
    # one check covers both rip entry points instead of chasing each. It is
    # also why this is not a route.py main() post-pass -- inside batch_route
    # it is on the shared engine path, and the verdict reaches the GUI too.
    #
    # BOTH fronts measure AND revert, by the same verdict. The two reverts are
    # spelled differently only because the artifact is: the file front rewrites
    # the output as the input board, and the GUI front returns an EMPTY
    # change-set. No pcbnew surgery is involved on either -- the GUI applier
    # runs AFTER batch_route returns, so withholding the change-set is a true
    # rollback, not an un-apply.
    #
    # Only the run that OWNS the artifact gates. Every nested batch_route --
    # the reconciliation sub-run, the plane finalize's reconnects -- carries
    # final_reconcile=False, and a nested verdict is both meaningless (it
    # measures one slice of the board) and actively misleading (it would print
    # a REVERT notice for a sub-run whose caller is still working). The CLI
    # sub-run is additionally in-place (input == output), which the file branch
    # excludes anyway; the GUI sub-run is not, which is how this surfaced.
    #
    # A board with no input copper cannot regress -- there is nothing yet to
    # break -- so the first (and usually largest) route step of a chain pays
    # nothing for the gate, and only the later steps, which are exactly where
    # the rips live, are measured.
    _gate_report = None
    _gate_ok = (env_knobs.IMPROVEMENT_GATE and not _ckpt_stop
                and final_reconcile
                and any(_orig_seg_by_net.values())
                and (return_results
                     or (output_file and input_file
                         and os.path.isfile(output_file)
                         and os.path.isfile(input_file)
                         and os.path.abspath(output_file)
                         != os.path.abspath(input_file))))
    if _gate_ok:
        try:
            from improvement_gate import (net_connectivity_map,
                                          compare_connectivity, gate_verdict,
                                          format_report)
            if return_results:
                # The board the GUI applier will produce: input copper MINUS
                # what it removes PLUS what it adds. NOT pcb_data, which also
                # carries orphan copper from rip/reroute that reaches no
                # apply channel and would grade a broken net as connected
                # (the same trap the #8 write-model sweep documents).
                _drop_s = {id(s) for s in (results_data.get('segments_to_remove') or [])}
                _drop_v = {id(v) for v in (results_data.get('vias_to_remove') or [])}
                _after_s = {nid: [s for s in lst if id(s) not in _drop_s]
                            for nid, lst in _orig_seg_by_net.items()}
                _after_v = {nid: [v for v in lst if id(v) not in _drop_v]
                            for nid, lst in _orig_via_by_net.items()}
                for _r6 in results_data.get('results', []):
                    for _s6 in (_r6.get('new_segments') or []):
                        _after_s.setdefault(_s6.net_id, []).append(_s6)
                    for _v6 in (_r6.get('new_vias') or []):
                        _after_v.setdefault(_v6.net_id, []).append(_v6)
                for _s6 in (results_data.get('all_swap_segments') or []):
                    _after_s.setdefault(_s6.net_id, []).append(_s6)
                for _v6 in (results_data.get('all_swap_vias') or []):
                    _after_v.setdefault(_v6.net_id, []).append(_v6)
                _before_map = net_connectivity_map(
                    pcb_data, segs_by_net=_orig_seg_by_net,
                    vias_by_net=_orig_via_by_net)
                _after_map = net_connectivity_map(
                    pcb_data, segs_by_net=_after_s, vias_by_net=_after_v)
                _name_of = (lambda nid: (pcb_data.nets[nid].name
                                         if nid in pcb_data.nets
                                         else f"Net {nid}"))
            else:
                from kicad_parser import parse_kicad_pcb as _pk600
                _before_pcb = _pk600(input_file)
                _after_pcb = _pk600(output_file)
                _before_map = net_connectivity_map(_before_pcb)
                _after_map = net_connectivity_map(_after_pcb)
                _name_of = (lambda nid: (_after_pcb.nets[nid].name
                                         if nid in _after_pcb.nets
                                         else f"Net {nid}"))
            _cmp = compare_connectivity(_before_map, _after_map, _name_of)
            _verdict = gate_verdict(_cmp)
            _why = ("This run did not fail to execute -- it ran and was "
                    "REJECTED, so re-running it with MORE rip authority "
                    "cannot help: change the approach (thinner track / finer "
                    "grid / different layers), or accept the open nets and "
                    "report them. See docs/rip-up-reroute.md 'Improvement "
                    "gate'. KICAD_IMPROVEMENT_GATE=0 ships the regression "
                    "instead.")
            if _verdict == 'reject' and return_results:
                # Withhold the change-set: the applier has not touched the
                # live board yet, so an empty result IS the rollback. Keep
                # the diagnostics (blockers / open pad pairs) -- they are
                # why the caller asked -- and the gate report itself.
                _keep6 = {k: results_data.get(k) or []
                          for k in ('blockers', 'pad_pairs_open',
                                    'boxed_in')}
                results_data = _empty_results_data()
                results_data.update(_keep6)
                _action = ("DISCARDED this run's changes (nothing is applied "
                           "to the board; the pre-rip board is the better "
                           "artifact). " + _why)
            elif _verdict == 'reject':
                # Restore the input board. Passing it through keeps the chain
                # intact (the next step still has a board) instead of failing
                # the run and stranding the pipeline.
                #
                # copy_board, NOT a bare .kicad_pcb copy: the SIBLINGS have to
                # travel with it (#441). The rejected run may already have
                # stamped its own .kicad_pro DRC floor next to the output, and
                # leaving that beside a reverted board is precisely the
                # stranded-project trap -- the next step would resolve floors
                # from a project describing copper that is no longer there.
                if (input_file.endswith('.kicad_pcb')
                        and output_file.endswith('.kicad_pcb')):
                    from copy_board import copy_board as _cb600
                    _cb600(input_file, output_file)
                else:
                    from pcb_io_utils import passthrough_copy as _pt600
                    _pt600(input_file, output_file)
                batch_route._improvement_gate_reverted = True
                _action = (f"REVERTED {output_file} to the input board (the "
                           f"pre-rip board is the better artifact). " + _why)
            else:
                _action = ("shipped (the run connected at least as many nets "
                           "as it broke)")
            _gate_report = dict(_cmp, verdict=_verdict)
            if _cmp['lost'] or _verdict == 'reject':
                print("\n" + RED + format_report(_cmp, _verdict, _action)
                      + RESET)
            print(f"JSON_IMPROVEMENT_GATE: {json.dumps(_gate_report)}")
            if return_results:
                results_data['improvement_gate'] = _gate_report
        except Exception as _ge:
            # A gate that crashes must not take the run's board with it.
            print(f"  (improvement gate skipped: {_ge})")

    # ONE compact authoritative line per outermost run, CLI and GUI alike.
    # The big JSON_SUMMARY lines are 6-20KB each with scope semantics the log
    # itself has to warn about; this one is the merged tally in <1KB.
    # `final_reconcile` is True only on the outermost call (every sub-run --
    # plane finalize, end-of-run reconcile -- passes False, the #562 recursion
    # guard), so nested passes never emit a second MIN line. Deliberately the
    # LAST thing printed: after the reconcile block, so it carries the merged
    # tally, and after the improvement gate, so it describes the copper that
    # is actually on the board -- a rejected run says so here, because its
    # tallies describe a board that was reverted. It is the last line
    # batch_route prints, NOT the last line of a run: main()'s .kicad_pro
    # DRC-floor writeback reports after this, so read the log for the line,
    # do not tail it.
    if final_reconcile:
        _emit_summary_min(_gate_report)

    if return_results:
        return successful, failed, total_time, results_data
    return successful, failed, total_time

if __name__ == "__main__":
    import argparse
    # #666's scoped cap move calls `write_placed_output`, which makes this a
    # registered pose author (LEVER_REGISTRY). Declare it for the whole
    # process: this `__main__` is one long inline block rather than a call to
    # a main(), so an ExitStack closed at interpreter exit gives exactly the
    # lifetime a `with` around the block would. Best-effort -- a flat install
    # without py_placer/ must still route.
    try:
        import atexit
        import contextlib
        # Imported for its SIDE EFFECT only (it appends py_placer/ to
        # sys.path), and without binding a name, because two later blocks in
        # this same __main__ import it under that name for the same reason.
        __import__('_placer_path')
        from placement.provenance import declare_lever as _declare_lever
        _lever_stack = contextlib.ExitStack()
        _lever_stack.enter_context(_declare_lever('route.py', sys.argv))
        atexit.register(_lever_stack.close)
    except Exception:                                        # noqa: BLE001
        pass
    # Windows consoles default to cp1252, which can't encode the non-ASCII glyphs
    # some log lines use (arrows in bus order, Ohm in impedance, the fab-floor
    # warning sign); reconfigure stdout/stderr to UTF-8 so a print never crashes
    # the run (issue #152).
    from console_encoding import enable_utf8_console
    enable_utf8_console()
    # CMD/EXIT self-echo (run-3 B1). CLI-`__main__`-only by construction: the
    # GUI imports batch_route and never runs this block.
    #
    # Caveat this banner cannot escape: an EXTERNAL kill (a harness timeout, and
    # on Windows TerminateProcess in particular) skips `atexit`, so the promised
    # EXIT= line never arrives. The tool has no self-budget to fall back on --
    # deliberately, since no result it produces may depend on a wall clock -- so
    # a harness that kills this process owns that gap. The gap the banner does
    # close: the convergence-ledger protocol says to paste a tool's own CMD:
    # line into `converge record --argv`, which was unsatisfiable for a routing
    # lap -- run 11 hand-wrote replay scripts instead.
    #
    # NOT under --capabilities, whose whole stdout is ONE JSON document a
    # consumer does `json.loads()` on (see the block near parse_args, and
    # converge.py:787 for the same rule stated for the same reason). A `CMD:`
    # line ahead of it is a JSONDecodeError at char 0, which is a broken API
    # rather than a noisy log. Raw argv, because the flag is answered before
    # argparse runs -- the banner has to be decided before then too.
    if '--capabilities' not in sys.argv[1:]:
        import cli_banner
        cli_banner.install()
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set

    parser = argparse.ArgumentParser(
        description="Batch PCB Router - Routes single-ended nets using Rust-accelerated A*. For differential pairs, use route_diff.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Wildcard patterns supported:
  "Net-(U2A-DATA_*)"  - matches Net-(U2A-DATA_0), Net-(U2A-DATA_1), etc.
  "Net-(*CLK*)"       - matches any net containing CLK

Examples:
  python py_router/route.py fanout_starting_point.kicad_pcb routed.kicad_pcb "Net-(U2A-DATA_*)"
  python py_router/route.py input.kicad_pcb output.kicad_pcb "Net-(U2A-DATA_*)" --ordering mps

For differential pair routing, use route_diff.py:
  python py_router/route_diff.py input.kicad_pcb output.kicad_pcb --nets "*lvds*"
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
    parser.add_argument("--component", "-C", nargs="+", metavar="REF",
                        help="Route all nets connected to these components (e.g., U1, or U3 U4 'J1*'). "
                             "fnmatch globs allowed; a bare reference is exact, so U1 does not "
                             "match U10. Excludes GND/VCC/VDD unless net patterns also specified.")
    parser.add_argument("--group", metavar="BLOCK",
                        help="Route only the nets of one PLACEMENT BLOCK "
                             "(#459), e.g. 'decap:U3' or a sheet block. The "
                             "N-part generalization of --component; composes "
                             "with net patterns the same way (patterns given -> "
                             "intersect). Use --group-by to choose how blocks "
                             "are derived, and --list-groups to see them.")
    parser.add_argument("--group-by", default="auto", metavar="SOURCES",
                        help="How to derive placement blocks for --group: comma "
                             "list of kicad, sheet, netprefix, decap; 'auto' = "
                             "kicad,sheet (default: auto)")
    parser.add_argument("--group-scope", default=None,
                        choices=("touching", "internal"),
                        help="Which of a block's nets to act on: 'touching' = "
                             "any pad in the block, including its interface to "
                             "the rest of the board (what --component means); "
                             "'internal' = every pad inside the block. A "
                             "schematic sheet is 60-70%% internal; a decap block "
                             "is 0%% internal, so 'touching' is the only useful "
                             "reading there. DEFAULT DEPENDS ON THE OPERATION: "
                             "'touching' when routing (route the block's "
                             "interface too), 'internal' with --undo -- because "
                             "a block's touching set includes GND/VCC, so an "
                             "undo at 'touching' would strip those nets' copper "
                             "across the WHOLE board, not just this block")
    parser.add_argument("--list-groups", action="store_true",
                        help="List the placement blocks --group-by would derive, "
                             "with their net counts, and exit")
    parser.add_argument("--preview", action="store_true",
                        help="Route, report exactly what WOULD change, and write "
                             "no output file. The engine already supports this; "
                             "this exposes it on the CLI.")
    parser.add_argument("--preview-png", metavar="FILE",
                        help="With --preview, also render the board with the "
                             "proposed copper highlighted (needs Pillow)")
    parser.add_argument("--undo", action="store_true",
                        help="Remove the scoped nets' routed copper instead of "
                             "routing them, putting them back to unrouted. "
                             "Refuses KiCad-locked copper. Does NOT invert pad "
                             "swaps -- see the printed report.")
    # Ordering and strategy options
    parser.add_argument("--ordering", "-o", choices=["inside_out", "mps", "original", "bus"],
                        default=defaults.DEFAULT_ORDERING_STRATEGY,
                        help="Net ordering strategy: mps (default, crossing conflicts), "
                             "inside_out, original, or bus (detected bus groups first, "
                             "members middle-out, rest by mps; ordering only -- "
                             "corridor attraction still needs --bus)")
    parser.add_argument("--direction", "-d", choices=["forward", "backward"],
                        default=None,
                        help="Direction search order for each net route")
    # #381 D9: accept the singular --no-bga-zone spelling too (the plane/fanout
    # scripts spell it singular); same nargs='*' dest -- additive, plural kept.
    parser.add_argument("--no-bga-zones", "--no-bga-zone", nargs="*", default=None,
                        help="Disable BGA exclusion zones. No args = disable all. With component refs (e.g., U1 U3) = disable only those.")
    parser.add_argument("--rip-existing-nets", nargs="+", default=None,
                        metavar="PATTERN",
                        help="Net name patterns of PRE-EXISTING routed nets that may be "
                             "ripped up and re-routed when they block a net being routed "
                             "(e.g. on a board routed by a previous run). Use '*' to allow "
                             "any non-plane net. Without this flag, committed tracks are "
                             "never ripped.")
    parser.add_argument("--force-reroute", action="store_true",
                        help="Rip and re-route from scratch every net selected by "
                             "--nets, even if already fully connected (#515's "
                             "manual recipe, automated; PR #533). Requires an "
                             "explicit --nets scope. Protected nets (length-matched "
                             "groups, routed diff pairs) are skipped unless named "
                             "EXACTLY in --nets; KiCad-locked copper is never "
                             "ripped; plane (zone-owning) nets are skipped -- use "
                             "route_planes for those. If the re-route fails "
                             "outright, the original copper is restored.")
    parser.add_argument("--layers", "-l", nargs="+",
                        default=None,
                        help="Routing layers to use (default: all of the board's "
                             "copper layers)")

    # Track and via geometry
    parser.add_argument("--track-width", type=float, default=None,
                        help="Track width in mm. Default: the board's Default net-class "
                             f"track_width (sibling .kicad_pro), else {defaults.TRACK_WIDTH}. "
                             "Ignored if --impedance is specified.")
    parser.add_argument("--impedance", type=float, default=None,
                        help="Target single-ended impedance in ohms (e.g., 50). Calculates track width per layer from board stackup.")
    parser.add_argument("--coplanar-gap", type=float, default=defaults.COPLANAR_GAP,
                        help="Declare that impedance-controlled traces run through a "
                             "ground pour on their OWN layer, this far (mm, trace edge "
                             "to pour edge) from it. Outer layers then use the "
                             "coplanar-waveguide-over-ground model instead of microstrip, "
                             "which gives a NARROWER trace for the same target ohms. "
                             "The pour does not exist yet at route time, so this is a "
                             "DECLARATION: pour the plane layers with a matching "
                             "'route_planes --zone-clearance', then verify with "
                             "'check_impedance.py --coplanar-gap'. Requires --impedance.")
    parser.add_argument("--coplanar-nets", nargs="+", default=None, metavar="PATTERN",
                        help="Limit --coplanar-gap to these nets (fnmatch patterns). "
                             "Omitted: every net in this call is treated as coplanar. "
                             "Given: only matching nets get CPW widths; the rest stay "
                             "microstrip.")
    parser.add_argument("--clearance", type=float, default=None,
                        help="Copper clearance CEILING in mm. When given, every net class "
                             "(Default included) is capped at min(class, this). When OMITTED, "
                             "each net routes at its own net-class clearance (base = the board's "
                             f"Default class from the sibling .kicad_pro, else {defaults.CLEARANCE}). "
                             "Use --net-clearances <json> for explicit per-net values.")
    parser.add_argument("--via-size", type=float, default=None,
                        help="Via outer diameter in mm. Default: the board's Default net-class "
                             f"via_diameter (sibling .kicad_pro), else {defaults.VIA_SIZE}.")
    parser.add_argument("--via-drill", type=float, default=None,
                        help="Via drill size in mm. Default: the board's Default net-class "
                             f"via_drill (sibling .kicad_pro), else {defaults.VIA_DRILL}.")
    parser.add_argument("--net-clearances", metavar="JSON", default=None,
                        help="Explicit override for the cross-class clearance map: a JSON object "
                             "mapping net name -> that net's net-class clearance in mm. When OMITTED, "
                             "the map is AUTO-READ from the sibling .kicad_pro's non-Default "
                             "netclasses (all-Default boards -> empty -> inert). Every pre-placed AND "
                             "in-run via/pad/segment obstacle of a different class is priced at "
                             "max(this call's routing floor, that obstacle net's own clearance) so a "
                             "foreign higher-clearance net (POWER_HI 0.25 while routing a Default "
                             "0.15 group) is not under-blocked (cross-class via-via/DRC). The GUI "
                             "derives the same map from the board's live net classes.")

    # Power net routing options
    parser.add_argument("--power-nets", nargs="*", default=[],
                        help="Glob patterns for power nets (e.g., '*GND*' '*VCC*'). Must pair with --power-nets-widths.")
    parser.add_argument("--power-nets-widths", nargs="*", type=float, default=[],
                        help="Track widths in mm for each power-net pattern (must match --power-nets length)")
    parser.add_argument("--no-power-tap-neckdown", action="store_true",
                        help="Disable neck-down retry of failed power-net tap edges (issue #72): by default a "
                             "wide tap that cannot fit is re-routed at the layer's default width near the pad")
    parser.add_argument("--capabilities", action="store_true",
                        help="Print this clone's module and flag inventory as JSON and "
                             "exit. A consumer that pins a KRT clone can only check that "
                             "route.py exists as a file, which passes for a clone on the "
                             "wrong branch or missing the module its chain needs -- the "
                             "run then prints green while describing an engine the repo "
                             "does not pin. See krt_capabilities.py --require.")
    parser.add_argument("--json-out", metavar="FILE", default=None,
                        help="Also write the run's JSON_SUMMARY to FILE. This is the MERGED "
                             "tally: when the end-of-run reconciliation fires, route.py emits a "
                             "SECOND, reconcile-subset-scoped summary to stdout, and scraping "
                             "either one alone is wrong -- the first counts every recovery as a "
                             "failure, the second narrows the whole-board pad-pair denominator to "
                             "the retried subset. Prefer this over parsing stdout.")
    parser.add_argument("--neckdown-length", type=float, default=defaults.NECKDOWN_LENGTH,
                        help="Length in mm of narrow track from the target pad on neck-down tap routes; the track "
                             "returns to the power width beyond this where clearance allows (default: 2.5)")
    parser.add_argument("--neckdown-taper-length", type=float, default=defaults.NECKDOWN_TAPER_LENGTH,
                        help="Length in mm of the stepped narrow-to-wide width taper on neck-down tap routes "
                             "(0 = abrupt width change, default: 0.5)")

    # Router algorithm parameters
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help=f"Grid resolution in mm (default: {defaults.GRID_STEP})")
    parser.add_argument("--via-cost", type=int, default=defaults.VIA_COST,
                        help=f"Penalty for placing a via, in 0.1mm grid steps (default: {defaults.VIA_COST} = 5mm of path; mm-equivalent at any --grid-step)")
    parser.add_argument("--via-proximity-cost", type=int, default=defaults.VIA_PROXIMITY_COST,
                        help=f"Via cost multiplier in stub/BGA proximity zones (default: {defaults.VIA_PROXIMITY_COST}, 0=no extra cost)")
    parser.add_argument("--max-iterations", type=int, default=defaults.MAX_ITERATIONS,
                        help=f"Max A* iterations before giving up (default: {defaults.MAX_ITERATIONS})")
    parser.add_argument("--max-probe-iterations", type=int, default=defaults.MAX_PROBE_ITERATIONS,
                        help="Max iterations for quick probe phase per direction (default: 5000)")
    parser.add_argument("--heuristic-weight", type=float, default=defaults.HEURISTIC_WEIGHT,
                        help=f"A* heuristic weight, higher=faster but less optimal (default: {defaults.HEURISTIC_WEIGHT})")
    parser.add_argument("--turn-cost", type=int, default=defaults.TURN_COST,
                        help=f"Penalty for direction changes, encourages straighter paths (default: {defaults.TURN_COST})")
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
    parser.add_argument("--guide-corridor", action="store_true",
                        help="Steer routed nets along a guide polyline drawn on a User layer (issue #7)")
    parser.add_argument("--guide-corridor-layer", type=str, default=defaults.GUIDE_CORRIDOR_LAYER,
                        help=f"User layer the guide polyline is drawn on (default: {defaults.GUIDE_CORRIDOR_LAYER})")
    parser.add_argument("--guide-corridor-spacing", type=float, default=defaults.GUIDE_CORRIDOR_SPACING,
                        help=f"Max mm between waypoints; 0 = use only the drawn segment endpoints, "
                             f">0 subdivides long segments to follow curves more tightly (default: {defaults.GUIDE_CORRIDOR_SPACING})")
    parser.add_argument("--keepout", action="store_true",
                        help="Keep routed tracks out of one or more polygons drawn on a User layer (issue #27)")
    parser.add_argument("--keepout-layer", type=str, default=defaults.KEEPOUT_LAYER,
                        help=f"User layer the keepout polygons are drawn on (default: {defaults.KEEPOUT_LAYER})")
    parser.add_argument("--proximity-heuristic-factor", type=float,
                        default=defaults.PROXIMITY_HEURISTIC_FACTOR,
                        help=f"Factor for proximity heuristic estimation (default: {defaults.PROXIMITY_HEURISTIC_FACTOR}, higher=faster but may find suboptimal paths)")

    # Stub proximity penalty
    parser.add_argument("--stub-proximity-radius", type=float, default=defaults.STUB_PROXIMITY_RADIUS,
                        help=f"Radius around stubs to penalize routing in mm (default: {defaults.STUB_PROXIMITY_RADIUS})")
    parser.add_argument("--stub-proximity-cost", type=float, default=defaults.STUB_PROXIMITY_COST,
                        help=f"Cost penalty near stubs in mm equivalent (default: {defaults.STUB_PROXIMITY_COST})")

    # BGA proximity penalty
    parser.add_argument("--bga-proximity-radius", type=float, default=defaults.BGA_PROXIMITY_RADIUS,
                        help="Radius around BGA edges to penalize routing in mm (default: 7.0)")
    parser.add_argument("--bga-proximity-cost", type=float, default=defaults.BGA_PROXIMITY_COST,
                        help="Cost penalty near BGA edges in mm equivalent (default: 0.2)")

    # Track proximity penalty (same layer only)
    parser.add_argument("--track-proximity-distance", type=float, default=defaults.TRACK_PROXIMITY_DISTANCE,
                        help=f"Radius around routed tracks in mm, same layer only (0 = disabled, default: {defaults.TRACK_PROXIMITY_DISTANCE})")
    parser.add_argument("--track-proximity-cost", type=float, default=defaults.TRACK_PROXIMITY_COST,
                        help=f"Cost penalty near routed tracks (0 = disabled, default: {defaults.TRACK_PROXIMITY_COST})")

    # Layer swap and target swap options
    parser.add_argument("--no-stub-layer-swap", action="store_true",
                        help="Disable stub layer switching optimization (enabled by default)")
    parser.add_argument("--no-crossing-layer-check", action="store_true",
                        help="Count crossings regardless of layer overlap (by default, only same-layer crossings count)")
    parser.add_argument("--can-swap-to-top-layer", action="store_true",
                        help="Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.")
    parser.add_argument("--swappable-nets", nargs="+",
                        help="Glob patterns for nets that can have targets swapped (e.g., '*DATA_*')")
    parser.add_argument("--schematic-dir", default=None,
                        help="Directory containing .kicad_sch files to update with pad swaps (default: no schematic update)")
    parser.add_argument("--crossing-penalty", type=float, default=defaults.CROSSING_PENALTY,
                        help="Penalty for crossing assignments in target swap optimization (default: 1000.0)")
    parser.add_argument("--mps-reverse-rounds", action="store_true",
                        help="Reverse MPS round order: route most-conflicting groups first instead of least-conflicting")
    parser.add_argument("--mps-layer-swap", action="store_true",
                        help="Enable MPS-aware layer swaps to reduce crossing conflicts")
    parser.add_argument("--mps-segment-intersection", action="store_true",
                        help="Force MPS to use segment intersection for crossing detection (auto-enabled when no BGA chips)")
    _sm = parser.add_mutually_exclusive_group()
    _sm.add_argument("--smoothing", dest="smoothing", action="store_true", default=True,
                     help="Collapse grid-A* staircase micro-jogs into octolinear shortcuts "
                          "(#536). ON by default -- measured worth ~20 incomplete nets "
                          "across 147 corpus boards versus off.")
    _sm.add_argument("--no-smoothing", dest="smoothing", action="store_false",
                     help="Disable #536 octolinear smoothing for this step. "
                          "KICAD_SMOOTH_ROUTE=0/1 overrides either way.")
    parser.add_argument("--keep-input-copper", action="store_true",
                        help="Treat the input file's own copper as read-only: the post-route "
                             "cleanup passes (dead-end sweep, orphan islands, cycle/redundancy "
                             "prunes, graze re-bends) never remove or rewrite it, only this "
                             "run's new copper. For chained flows whose earlier stages author "
                             "copper (fanout escape stubs, hand-routed nets) that later stages "
                             "or checks must still see verbatim - including stubs of nets this "
                             "run FAILED to route. Default: off (issue #84 semantics: dead "
                             "input stubs on in-scope nets are swept).")

    # Length matching options
    parser.add_argument("--length-match-group", action="append", nargs="+", dest="length_match_groups",
                        help="Net patterns to length-match as a group (can be repeated). Use 'auto' for DDR4 auto-grouping")
    parser.add_argument("--length-match-tolerance", type=float, default=defaults.LENGTH_MATCH_TOLERANCE,
                        help="Acceptable length variance within group in mm (default: 0.1)")
    parser.add_argument("--meander-amplitude", type=float, default=defaults.MEANDER_AMPLITUDE,
                        help="Height of meander perpendicular to trace in mm (default: 1.0)")
    parser.add_argument("--meander-spacing", type=float, default=defaults.MEANDER_SPACING,
                        help="Centre-to-centre spacing of adjacent meander arms, in multiples of "
                             "the net's routed track width (default: 2.0 = 2W)")

    # Time matching options (alternative to length matching)
    parser.add_argument("--time-matching", action="store_true",
                        help="Match by propagation time instead of length (accounts for layer dielectric)")
    parser.add_argument("--time-match-tolerance", type=float, default=defaults.TIME_MATCH_TOLERANCE,
                        help="Acceptable time variance in picoseconds (default: 1.0)")

    # Rip-up and retry options
    parser.add_argument("--max-ripup", type=int, default=defaults.MAX_RIPUP,
                        help=f"Maximum blockers to rip up at once during rip-up and retry (default: {defaults.MAX_RIPUP})")
    parser.add_argument("--ripup-abandon-metric",
                        choices=list(defaults.RIPUP_ABANDON_METRIC_CHOICES),
                        default=os.environ.get('KICAD_RIPUP_ABANDON_METRIC',
                                               defaults.RIPUP_ABANDON_METRIC),
                        help="How a Phase 3 tap rip-up decides keep-retry vs abandon "
                             "(see docs/rip-up-reroute.md). Env override: "
                             f"KICAD_RIPUP_ABANDON_METRIC (default: {defaults.RIPUP_ABANDON_METRIC})")
    parser.add_argument("--ripup-blocker-select",
                        choices=list(defaults.RIPUP_BLOCKER_SELECT_CHOICES),
                        default=os.environ.get('KICAD_RIPUP_BLOCKER_SELECT',
                                               defaults.RIPUP_BLOCKER_SELECT),
                        help="Blocker SELECTION algorithm for the rip-up ladder. "
                             "'count' = historical weighted cell "
                             "count; 'near-target' = endpoint-proximity first "
                             "(the true last-mile blocker hugs the failing pad "
                             "but has few cells); 'bidir' = boost nets blocking "
                             "BOTH search directions (genuine separating walls); "
                             "'mincut' = soft-cost probe on a map clone that "
                             "reads the actual crossing set (names the true "
                             "joint cut; falls back to count order when the "
                             "wall is static copper). Default: count.")
    parser.add_argument("--routing-clearance-margin", type=float, default=defaults.ROUTING_CLEARANCE_MARGIN,
                        help=f"Multiplier on track-via clearance ({defaults.ROUTING_CLEARANCE_MARGIN} = minimum DRC)")
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
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)

    # Vertical alignment attraction options
    parser.add_argument("--vertical-attraction-radius", type=float, default=defaults.VERTICAL_ATTRACTION_RADIUS,
                        help="Radius in mm for cross-layer track attraction (0 = disabled, default: 1.0)")
    parser.add_argument("--vertical-attraction-cost", type=float, default=defaults.VERTICAL_ATTRACTION_COST,
                        help=f"Cost bonus for aligning with tracks on other layers (0 = disabled, default: {defaults.VERTICAL_ATTRACTION_COST})")

    # Ripped route avoidance options
    parser.add_argument("--ripped-route-avoidance-radius", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS,
                        help=f"Radius in mm around ripped route segments/vias for soft penalty (default: {defaults.RIPPED_ROUTE_AVOIDANCE_RADIUS})")
    parser.add_argument("--ripped-route-avoidance-cost", type=float, default=defaults.RIPPED_ROUTE_AVOIDANCE_COST,
                        help=f"Soft penalty cost for routing through ripped corridors (0 = disabled, default: {defaults.RIPPED_ROUTE_AVOIDANCE_COST})")

    # Layer preference options
    parser.add_argument("--layer-costs", nargs="+", type=float, default=[],
                        help="Per-layer cost multipliers (1.0-1000, default: F.Cu=1.0, others=3.0), "
                             "or any negative value (e.g. -1) = FORBIDDEN: the layer stays an obstacle "
                             "(its copper blocks vias) and through-vias may span it, but NO track copper "
                             "is placed on it. Order matches --layers. Example (route only F.Cu/B.Cu, "
                             "In1/In2 obstacle-only): --layers F.Cu In1.Cu In2.Cu B.Cu --layer-costs 1.0 -1 -1 3.0")

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
    parser.add_argument("--stats", action="store_true",
                        help="Collect and print A* search statistics for debugging heuristic efficiency")

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    if '--capabilities' in sys.argv[1:]:
        # Answered BEFORE parse_args, and before any board is touched:
        # `input_file` is a required positional, and "is this the engine I
        # pinned?" has to be answerable without naming a board -- a consumer
        # asks it precisely because it does not yet trust anything else this
        # clone would tell it. sys.exit rather than return: this block lives
        # directly under `if __name__ == "__main__":`, not inside a main().
        # krt_capabilities lives at the REPO root, one level above this
        # engine dir (#522 layout), so hop up before importing it.
        _repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if _repo_root not in sys.path:
            sys.path.insert(0, _repo_root)
        from krt_capabilities import capabilities as _caps
        print(json.dumps(_caps(), indent=1, sort_keys=True))
        sys.exit(0)
    # #597: pin the dash-digit matcher BEFORE parsing, or a net named like a
    # negative rail (-12V, -5V) is read as an unknown option and argparse exits
    # 2 before this line's result is ever used.
    args = __import__("cli_nets").pin_dash_digit_values(parser).parse_args()
    # #439: the PRESENCE of --clearance is the clamp switch. Given -> it is the
    # ceiling: non-Default classes are capped at min(class, --clearance) and the
    # output .kicad_pro clamps them to the routed floor. Omitted -> honor the
    # classes: the base clearance defaults to the board's OWN Default net-class
    # clearance, each non-Default net routes at its full class value (uncapped), and
    # the writeback preserves the classes. --hole-to-hole-clearance and
    # --board-edge-clearance work the same way (omitted -> the board's own
    # constraint minimum). Resolved here, before enforce_fab_floors and every
    # downstream use. Stashed on args for drc_fix_kwargs (the writeback clamp).
    from list_nets import (board_default_netclass_clearance, board_default_netclass_param,
                           resolve_cli_floor)
    # #435 companion: whether --track-width was EXPLICITLY set. If NOT, each net
    # routes at its OWN netclass track width engine-side (not the single board
    # Default-class width). --impedance no longer counts as explicit here (#610):
    # the engine guards the netclass-width path itself when impedance is set, and
    # uses this bit to floor impedance-solved widths at the fab tier instead of
    # the resolved default width. Captured before the fill below overwrites None.
    _tw_explicit = args.track_width is not None
    # track_width / via_size / via_drill: when omitted, default to the board's OWN
    # Default net-class value (else the routing_defaults constant), so a bare route
    # uses the board's own geometry -- parity with the GUI's per-control override.
    for _pname, _nckey, _fallback in (('track_width', 'track_width', defaults.TRACK_WIDTH),
                                      ('via_size', 'via_diameter', defaults.VIA_SIZE),
                                      ('via_drill', 'via_drill', defaults.VIA_DRILL)):
        if getattr(args, _pname) is None:
            _v = board_default_netclass_param(args.input_file, _nckey)
            setattr(args, _pname, _v if _v is not None else _fallback)
            print(f"--{_pname.replace('_', '-')} not given; using "
                  f"{'the board Default net-class' if _v is not None else 'the fallback'} "
                  f"{getattr(args, _pname)}mm.")
    # --clearance, when given, is a pure CEILING on EVERY class -- Default included,
    # nothing special. Each net routes at min(its class, ceiling): the base clearance
    # (Default-class nets) is min(Default class, ceiling), and non-Default classes are
    # capped at the ceiling in the map below. Omitted -> no ceiling: every net routes
    # at its own class (base = the board's Default class).
    _ceiling = args.clearance                       # None iff --clearance omitted
    args._clamp_netclasses = _ceiling is not None
    args._clearance_ceiling = _ceiling
    from fix_kicad_drc_settings import warn_if_missing_project_floor
    warn_if_missing_project_floor(args.input_file)  # #441: a dropped sibling .kicad_pro strands the DRC floor
    _dflt_clr = board_default_netclass_clearance(args.input_file)
    if _ceiling is None:
        args.clearance = _dflt_clr if _dflt_clr is not None else defaults.CLEARANCE
        print(f"--clearance not given; honoring net classes with base = "
              f"{'the board Default net-class' if _dflt_clr is not None else 'the fallback'} "
              f"clearance {args.clearance}mm.")
    else:
        # min(Default class, ceiling) so Default is capped like every other class.
        args.clearance = min(_dflt_clr, _ceiling) if _dflt_clr is not None else _ceiling
    # Both floors go through the SHARED resolver (list_nets.resolve_cli_floor),
    # so a declared 0 -- KiCad's "not configured" -- reads as UNSET here exactly
    # as it does on the placement half of the loop. Read straight, these two
    # constraints made the two halves disagree about one declared number: 0.55
    # [fixed default] in render_placement/check_floorplan/converge against a
    # REAL 0.0 here, announced as "the board min_copper_edge_clearance 0.0mm".
    args.hole_to_hole_clearance = resolve_cli_floor(
        args.input_file, 'hole_to_hole', args.hole_to_hole_clearance,
        defaults.HOLE_TO_HOLE_CLEARANCE, '--hole-to-hole-clearance')
    args.board_edge_clearance = resolve_cli_floor(
        args.input_file, 'board_edge_clearance', args.board_edge_clearance,
        defaults.BOARD_EDGE_CLEARANCE, '--board-edge-clearance')
    set_default_fab_tier(*fab_tier_from_args(args))
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None),
        board_edge_clearance=getattr(args, 'board_edge_clearance', None))
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

    # Default --layers to ALL of the board's copper layers (issue #98: the old
    # F.Cu/B.Cu default silently routed 4-layer boards as 2-layer). The layer
    # cost defaults keep the documented bias (2-layer: F=1.0/B=3.0; 4+: 1.0).
    if args.layers is None:
        copper = pcb_data.board_info.copper_layers
        args.layers = list(copper) if copper else list(defaults.DEFAULT_LAYERS)
        if len(args.layers) > 2:
            print(f"Using all {len(args.layers)} copper layers: "
                  f"{' '.join(args.layers)} (override with --layers)")

    if args.list_groups:
        from group_routing import block_net_names
        import _placer_path  # noqa: F401  (placement lives in py_placer/)
        from placement.groups import (GroupError, derive_groups, parse_sources,
                                      short_name)
        try:
            _srcs = parse_sources(args.group_by)
        except GroupError as exc:
            print(f"ERROR: {exc}")
            sys.exit(2)
        _blocks = derive_groups(pcb_data, _srcs)
        if not _blocks:
            print(f"No placement blocks from sources {args.group_by!r}. "
                  f"Try --group-by sheet,netprefix,decap")
            sys.exit(0)
        print(f"{len(_blocks)} placement block(s) from {args.group_by!r}:")
        for _n, _r in sorted(_blocks.items(), key=lambda kv: (-len(kv[1]), kv[0])):
            _t = len(block_net_names(pcb_data, _r, 'touching'))
            _i = len(block_net_names(pcb_data, _r, 'internal'))
            print(f"  {short_name(_n):34s} parts={len(_r):3d}  nets: "
                  f"touching={_t:3d} internal={_i:3d}")
        sys.exit(0)

    # An EMPTY --group is a caller bug (a shell expansion that produced nothing).
    # Left alone it is falsy, so the scope silently widens to "*" -- and with
    # --undo that erases the board. Fail loudly instead of guessing.
    if args.group is not None and not args.group.strip():
        print("route.py: error: --group was given an empty name. Use "
              "--list-groups to see what exists.", file=sys.stderr)
        sys.exit(2)

    # Combine positional net_patterns and --nets argument.
    # Strip surrounding whitespace, as route_diff.py already does: a net list
    # read from a CRLF file hands every name a trailing '\r', which matches
    # nothing and (before the guard in batch_route) reported success over an
    # untouched board. Names are never meaningfully whitespace-padded.
    all_patterns = list(args.net_patterns) if args.net_patterns else []
    if args.nets:
        all_patterns.extend(args.nets)
    all_patterns = [p.strip() for p in all_patterns if p.strip()]

    # --force-reroute rips every selected net; without an explicit scope the
    # default-'*' below would silently select the WHOLE BOARD for rip+reroute.
    # (batch_route can't tell the difference -- main() always passes expanded
    # names -- so the guard lives here.) An explicit --nets '*' is honored:
    # the operator said so, and #521 protection still shields matched groups
    # (glob is not an override).
    # --component now takes one OR MORE references, with fnmatch globs. A
    # single one behaves exactly as before, so recorded redo_commands.sh
    # manifests and .claude/skills/* keep replaying unchanged.
    component_patterns = list(args.component or [])

    if args.force_reroute and not all_patterns and not component_patterns:
        parser.error("--force-reroute requires an explicit net scope "
                     "(--nets, positional patterns, or --component): it rips "
                     "and re-routes every selected net from scratch.")

    # Default to "*" (all nets) if no patterns and no component specified
    _scope_defaulted = False
    if not all_patterns and not component_patterns and not args.group:
        all_patterns = ["*"]
        _scope_defaulted = True   # nobody asked for "everything"; --undo checks this

    # Get nets from patterns and/or component
    if all_patterns:
        net_names = expand_net_patterns(pcb_data, all_patterns)
    else:
        net_names = []  # Will be populated by component filter below

    # Filter by component if specified
    if component_patterns:
        # Power/ground is dropped only when the components ARE the whole net
        # scope; with --nets also given the operator has named what they want.
        sel = nets_for_components(
            pcb_data, component_patterns,
            exclude_patterns=None if all_patterns else POWER_NET_EXCLUSION_PATTERNS)

        # A reference that matched no footprint is a typo, not an empty result:
        # routing the remaining subset would quietly do the wrong thing.
        if sel.unmatched_patterns:
            all_refs = sorted(set(pcb_data.footprints or {}))
            detail = "; ".join(
                f"{p!r}{suggest_component_refs(all_refs, p)}"
                for p in sel.unmatched_patterns)
            # --component takes one or more references, so it greedily consumes
            # the tokens after it -- a POSITIONAL output path written after it
            # lands here as a bogus reference. Say so rather than making the
            # operator work it out from a filename in a footprint error.
            swallowed = [p for p in sel.unmatched_patterns
                         if p.endswith('.kicad_pcb') or os.sep in p]
            hint = ""
            if swallowed:
                hint = (f" NOTE: --component accepts several references, so it "
                        f"consumed the positional argument {swallowed[0]!r} -- "
                        f"pass the output via --output, or put --component last.")
            parser.error(
                f"--component matched no footprint on this board: {detail}. "
                f"Board has {len(all_refs)} references"
                f"{' (e.g. ' + ', '.join(all_refs[:5]) + ')' if all_refs else ''}."
                f"{hint}")

        if all_patterns:
            # Intersect with pattern-matched nets
            _comp_nets = set(sel.net_names)
            net_names = [n for n in net_names if n in _comp_nets]
        else:
            net_names = list(sel.net_names)
        print(f"Matched {len(sel.matched_refs)} footprint(s) "
              f"({', '.join(sel.matched_refs[:8])}"
              f"{', ...' if len(sel.matched_refs) > 8 else ''}) "
              f"-> {len(net_names)} nets")
        if sel.excluded_names:
            # Historically silent, which is how "--component U1 routed nothing"
            # turned into a mystery on a part whose nets are mostly rails.
            print(f"  dropped {len(sel.excluded_names)} power/ground net(s): "
                  f"{', '.join(sel.excluded_names[:5])}"
                  f"{', ...' if len(sel.excluded_names) > 5 else ''} "
                  f"(name them in --nets to route them)")

    # --group: the N-part generalization of --component (#459). Same composition
    # rule -- patterns given, intersect; no patterns, take the block's nets.
    if args.group:
        from group_routing import (GroupRoutingError, block_net_names,
                                   block_refs, describe_scope, resolved_name)
        import _placer_path  # noqa: F401  (placement lives in py_placer/)
        from placement.groups import GroupError, parse_sources
        try:
            _srcs = parse_sources(args.group_by)
            _refs = block_refs(pcb_data, args.group, _srcs)
        except (GroupError, GroupRoutingError) as exc:
            print(f"ERROR: {exc}")
            sys.exit(2)
        # ECHO what a short/fuzzy name resolved to. '--group U3 --undo' can match
        # 'decap:U3' and delete copper without ever naming the block it picked.
        _resolved = resolved_name(pcb_data, args.group, _srcs)
        if _resolved and _resolved != args.group:
            print(f"  --group {args.group!r} resolved to block {_resolved!r}")
        # Scope default depends on the OPERATION. A block's 'touching' set
        # includes GND/VCC (a decap block is nothing BUT those), so an undo at
        # 'touching' strips those nets across the whole board -- measured on the
        # rp2350 fixture: 170 segments, 54 of them nowhere near the block.
        # Routing that same set is fine and desirable: it routes the interface.
        _scope = args.group_scope or ('internal' if args.undo else 'touching')
        if args.undo and args.group_scope == 'touching':
            print("  WARNING: --undo at --group-scope touching removes copper "
                  "from nets that LEAVE this block (GND/VCC included), across "
                  "the whole board -- not just the block's own copper.")
        _gnets = set(block_net_names(pcb_data, _refs, _scope))
        if all_patterns:
            net_names = [n for n in net_names if n in _gnets]
        else:
            net_names = sorted(_gnets)
        # Print the scope AFTER composing with any patterns, and report the count
        # we will actually act on -- otherwise the banner advertises the block's
        # full net count while an intersection quietly routes a fraction of it.
        print(describe_scope(pcb_data, _refs, _scope,
                             final=len(net_names) if all_patterns else None))

    if not net_names:
        print("No nets matched the given patterns!")
        sys.exit(1)

    # ...and the case that guard cannot see: net_names is NON-empty but every
    # entry is a literal that matched nothing. expand_net_patterns deliberately
    # passes an unmatched literal through (callers may name nets a board does
    # not carry), so the list is full of names that will all fail to resolve.
    # batch_route then found zero net_ids, wrote an unchanged passthrough copy
    # and returned rc 0 -- indistinguishable from "nothing left to do". Run 11
    # lost a lap to it: a CRLF net list gave all 88 names a trailing '\r'.
    # batch_route now raises for the GUI's benefit; the CLI refuses here so the
    # exit code is a clean 2 rather than a traceback.
    if not resolve_net_ids(pcb_data, net_names):
        print(f"route.py: error: none of the {len(net_names)} requested net "
              f"name(s) exist on this board -- nothing would be routed. Check "
              f"for stray whitespace or a stale net list. First few: "
              f"{', '.join(repr(n) for n in net_names[:5])}", file=sys.stderr)
        sys.exit(2)

    # --undo: strip the scoped nets' copper instead of routing them.
    if args.undo:
        # An unscoped run defaults to "*" (line ~3080) because routing the whole
        # board is the sensible default for ROUTING. For an undo it is not: it
        # silently means "erase every track on the board". Make the caller say so.
        if _scope_defaulted:
            print("route.py: error: --undo needs an explicit scope -- pass "
                  "--group BLOCK, --component REF, or net patterns. It does NOT "
                  "default to every net, because for an undo that would erase "
                  "the whole board. To really do that, pass '*' explicitly.",
                  file=sys.stderr)
            sys.exit(2)
        from group_routing import unroute_nets
        with open(args.input_file, 'r', encoding='utf-8') as _f:
            _content = _f.read()
        _content, _rep = unroute_nets(_content, pcb_data, net_names,
                                      args.input_file)
        # Report arcs separately: `segments_found` counts each arc's LINEARIZED
        # pieces (the parser expands one (arc) into many Segments), so a bare
        # removed/found ratio reads like a failure on an arc-routed board even
        # when every arc was deleted.
        _arcs = (f", {_rep['arcs_removed']} arc track(s)"
                 if _rep.get('arcs_removed') else "")
        print(f"Undo: {_rep['segments_removed']}/{_rep['segments_found']} "
              f"segment(s){_arcs} and {_rep['vias_removed']}/{_rep['vias_found']} "
              f"via(s) removed across {len(_rep['nets'])} net(s)")
        if _rep['locked_skipped']:
            print(f"  REFUSED (KiCad-locked, no override by design, #521): "
                  f"{', '.join(_rep['locked_skipped'][:8])}")
        if _rep['protected_overridden']:
            print(f"  protected nets removed by exact-name targeting: "
                  f"{', '.join(_rep['protected_overridden'][:8])}")
        if _rep['unmatched']:
            # (arc) tracks and copper graphics: in pcb_data.segments, but not
            # expressible as a (segment) deletion. Never report this as success.
            print(f"  WARNING: {_rep['unmatched']} piece(s) of copper could NOT "
                  f"be removed -- (arc) tracks or copper graphics have no "
                  f"(segment) block to delete. Nets still carrying copper: "
                  f"{', '.join(_rep['unmatched_nets'][:8])}. Remove them in "
                  f"KiCad, or these nets are NOT fully unrouted.")
        if _rep['zones_left']:
            print(f"  NOTE: {len(_rep['zones_left'])} net(s) still have a filled "
                  f"ZONE, which is copper: {', '.join(_rep['zones_left'][:8])}. "
                  f"An undo strips tracks and vias, never a pour -- deleting a "
                  f"plane is a plane decision. Re-routing these will produce a "
                  f"track web beside the pour, not the original plane.")
        print("  NOTE: pad/target swaps are NOT inverted -- no inverse exists, "
              "and they can touch nets outside this scope. This returns the "
              "named nets to 'no copper', not the board to a prior state.")
        if args.preview:
            print("Preview: no output file written")
            sys.exit(0)
        with open(args.output_file, 'w', encoding='utf-8') as _f:
            _f.write(_content)
        # Carry the sibling project files across VERBATIM (#441). An undo only
        # REMOVES copper, so the board's DRC floor is unchanged and the input's
        # floor is exactly the right one to keep -- unlike a routing step, there
        # is nothing new to write back. Dropping them would strand the floor and
        # make the next step resolve it from the looser stock netclass, which
        # KiCad then grades as phantom sub-clearance DRC on correct copper.
        import shutil
        from copy_board import SIBLING_EXTS
        # splitext, NOT a fixed [:-10] slice. Any output that is not *.kicad_pcb
        # (os.devnull, a bare name) made that slice produce a WRONG base -- an
        # empty one for 'nul', which then created a stray '.kicad_pro' dotfile in
        # the CWD and printed "carried the DRC floor" about it. copy_board and
        # protected_nets.pro_path_for_board both already do it this way.
        _sb, _db = (os.path.splitext(args.input_file)[0],
                    os.path.splitext(args.output_file)[0])
        _carried = []
        _real_out = args.output_file.endswith('.kicad_pcb')
        for _ext in (SIBLING_EXTS if _real_out else ()):
            if os.path.isfile(_sb + _ext) and os.path.abspath(_sb) != os.path.abspath(_db):
                try:
                    shutil.copy2(_sb + _ext, _db + _ext)
                    _carried.append(_ext)
                except OSError as _e:
                    print(f"  WARNING: could not carry {_ext} (#441): {_e}")
        if _carried:
            print(f"  carried the DRC floor: {', '.join(_carried)}")
        elif not os.path.isfile(_sb + '.kicad_pro'):
            print("  NOTE: the input has no sibling .kicad_pro, so none was "
                  "carried; the output's DRC floor stays undefined (#441).")
        print(f"Wrote {args.output_file}")
        sys.exit(0)

    print(f"Routing {len(net_names)} nets: {net_names[:5]}{'...' if len(net_names) > 5 else ''}")

    # Cross-class clearance map: resolve {net name -> clearance} to {net_id -> clearance} against
    # this board's nets so the obstacle-map builders price each pre-placed obstacle at its own
    # net-class clearance (KiCad's max(classA, classB)). None/absent -> empty map -> prior
    # behaviour. The GUI front builds the same map from live net classes (swig_gui).
    # #439: ALWAYS build the non-Default netclass map. When --clearance was GIVEN
    # (args._clamp_netclasses) it is the CEILING -- each class is capped at
    # min(class, --clearance), because stock classes are often aspirational (the
    # human-routed zynq itself violates its 0.2 class, routed ~0.1). When --clearance
    # was OMITTED the classes are honored in full (each net routes at its own class),
    # and the writeback preserves them. --net-clearances <json> overrides with
    # explicit per-net values, used as-is.
    _net_clearances_map = None
    if args.net_clearances:
        with open(args.net_clearances, encoding="utf-8") as _f:
            _name_to_clr = json.load(_f)
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
        if _net_clearances_map and args._clamp_netclasses:
            _net_clearances_map = {nid: min(clr, args._clearance_ceiling)
                                   for nid, clr in _net_clearances_map.items()}
        if _net_clearances_map:
            _classes = sorted({round(v, 4) for v in _net_clearances_map.values()})
            _mode = (f"capped at --clearance {args._clearance_ceiling}"
                     if args._clamp_netclasses
                     else "honored in full (--clearance omitted)")
            print(f"Netclass clearances for {len(_net_clearances_map)} net(s), {_mode} "
                  f"(mm: {_classes}); cross-class max(A,B) respected.")

    # --preview: the engine already supports this -- return_results=True with
    # an empty output_file routes fully, mutates only the in-memory PCBData
    # and writes nothing. This just exposes it on the CLI (#459 follow-on).
    _preview_out = batch_route(args.input_file,
                "" if args.preview else args.output_file, net_names,
                return_results=args.preview,
                direction_order=args.direction,
                ordering_strategy=args.ordering,
                disable_bga_zones=args.no_bga_zones,
                rip_existing_nets=args.rip_existing_nets,
                force_reroute=args.force_reroute,
                # RAW patterns (pre-expansion): the #521 protection override
                # must see what the operator TYPED, not the expanded names.
                net_name_patterns=all_patterns,
                layers=args.layers,
                track_width=args.track_width,
                track_width_from_class=not _tw_explicit,
                impedance=args.impedance,
                coplanar_gap=args.coplanar_gap,
                coplanar_nets=args.coplanar_nets,
                power_nets=args.power_nets,
                power_nets_widths=args.power_nets_widths,
                power_tap_neckdown=not args.no_power_tap_neckdown,
                neckdown_length=args.neckdown_length,
                neckdown_taper_length=args.neckdown_taper_length,
                json_out=args.json_out,
                clearance=args.clearance,
                net_clearances=_net_clearances_map,
                keep_input_copper=args.keep_input_copper,
                smoothing=args.smoothing,
                via_size=args.via_size,
                via_drill=args.via_drill,
                grid_step=args.grid_step,
                via_cost=args.via_cost,
                max_iterations=args.max_iterations,
                max_probe_iterations=args.max_probe_iterations,
                heuristic_weight=args.heuristic_weight,
                turn_cost=args.turn_cost,
                direction_preference_cost=args.direction_preference_cost,
                bus_enabled=args.bus,
                bus_detection_radius=args.bus_detection_radius,
                bus_attraction_radius=args.bus_attraction_radius,
                bus_attraction_bonus=args.bus_attraction_bonus,
                bus_min_nets=args.bus_min_nets,
                guide_corridor_enabled=args.guide_corridor,
                guide_corridor_layer=args.guide_corridor_layer,
                guide_corridor_spacing=args.guide_corridor_spacing,
                keepout_enabled=args.keepout,
                keepout_layer=args.keepout_layer,
                proximity_heuristic_factor=args.proximity_heuristic_factor,
                stub_proximity_radius=args.stub_proximity_radius,
                stub_proximity_cost=args.stub_proximity_cost,
                via_proximity_cost=args.via_proximity_cost,
                bga_proximity_radius=args.bga_proximity_radius,
                bga_proximity_cost=args.bga_proximity_cost,
                track_proximity_distance=args.track_proximity_distance,
                track_proximity_cost=args.track_proximity_cost,
                debug_lines=args.debug_lines,
                verbose=args.verbose,
                max_rip_up_count=args.max_ripup,
                ripup_abandon_metric=args.ripup_abandon_metric,
                ripup_blocker_select=args.ripup_blocker_select,
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
                debug_memory=args.debug_memory,
                mps_reverse_rounds=args.mps_reverse_rounds,
                mps_layer_swap=args.mps_layer_swap,
                mps_segment_intersection=args.mps_segment_intersection,
                schematic_dir=args.schematic_dir,
                layer_costs=args.layer_costs,
                add_teardrops=args.add_teardrops,
                collect_stats=args.stats)

    if args.preview:
        _ok, _fail, _t, _data = _preview_out
        _res = _data.get('results') or []
        _segs = sum(len(r.get('new_segments', []) or []) for r in _res)
        _vias = sum(len(r.get('new_vias', []) or []) for r in _res)
        print("")
        print("=== PREVIEW (no output file written) ===")
        print(f"  routed       : {_ok} net(s) ok, {_fail} failed, {_t:.1f}s")
        print(f"  would ADD    : {_segs} segment(s), {_vias} via(s)")
        print(f"  would REMOVE : {len(_data.get('segments_to_remove') or [])} "
              f"input segment(s), {len(_data.get('vias_to_remove') or [])} input via(s)")
        _sw = len(_data.get('all_swap_vias') or []) + len(_data.get('pad_swaps') or [])
        if _sw:
            print(f"  would SWAP   : {_sw} pad/via swap(s) -- these can touch nets "
                  f"OUTSIDE the scope and have no inverse, so they are the part an "
                  f"--undo cannot take back")
        if args.preview_png:
            try:
                from route_render import BoardRenderer
                _new = [x for r in _res for x in (r.get('new_segments', []) or [])]
                BoardRenderer(pcb_data).frame(
                    highlight_segments=_new, label="preview").save(args.preview_png)
                print(f"  rendered     : {args.preview_png}")
            except Exception as _e:
                print(f"  (render skipped: {_e})")
        # Exit 0 even when nets failed: the PREVIEW succeeded, and the routing
        # outcome is the report above (and the JSON_SUMMARY). A normal route.py
        # run with failed nets also exits 0, so exiting 1 here would make the
        # read-only mode the only one that can kill a `set -e` redo_commands.sh
        # replay -- at a step that changes nothing.
        sys.exit(0)
    # #600: when the improvement gate reverted the output, it IS the input
    # board -- byte for byte, siblings included. Every post-pass below must
    # stand down: the castellated retract MUTATES the board (so the revert
    # would no longer be faithful) and the DRC writeback would stamp this
    # run's requested floors onto copper this run did not produce, which is
    # the stranded-project trap the revert exists to avoid (#441).
    _gate_reverted = getattr(batch_route, '_improvement_gate_reverted', False)
    if _gate_reverted:
        print("  (improvement gate reverted the output -- skipping the "
              "castellated retract and the DRC-settings writeback; the "
              "output is the input board, project and rules included)")

    # Castellated landings (run-6 fix 1.7): pull track ends that landed inside
    # a castellated pad's edge-clearance zone back to the pad's inner reach.
    # No-op on boards without pad_prop_castellated pads or an edge rule.
    if not args.skip_routing and args.output_file and not _gate_reverted \
            and os.path.isfile(args.output_file):
        try:
            from fix_kicad_drc_settings import effective_board_edge_clearance
            from pcb_modification import retract_castellated_landings
            _edge = effective_board_edge_clearance(
                args.input_file, args.board_edge_clearance or 0.0)
            if _edge > 0:
                retract_castellated_landings(args.output_file, _edge)
        except Exception as e:
            print(f"  (skipped castellated-landing retract: {e})")

    # Make the written project's KiCad DRC constraints consistent with the
    # clearances/sizes we just routed to, so a manual DRC only flags genuine
    # problems instead of stock-default noise (issue #160). Only edits the
    # .kicad_pro, never the .kicad_pcb, so the board's KiCad version is preserved.
    if not args.no_fix_drc_settings and not args.skip_routing \
            and not _gate_reverted \
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
                **drc_fix_kwargs(args))
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")
        # #521: record this step's protection-worthy nets (matched groups) and
        # impedance declarations in the output project so later chain steps
        # refuse to rip the former and redo the latter at the same widths.
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


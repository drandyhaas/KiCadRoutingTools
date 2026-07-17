#!/usr/bin/env python3
"""
Route Disconnected Planes - Connects disconnected regions within power plane zones.

After power planes are created, regions may be effectively split due to vias and
traces from other nets cutting through the plane. This script detects disconnected
regions and routes wide, short tracks between them to ensure electrical continuity.

Usage:
    # Auto-detect all zones in PCB:
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb

    # Specific nets and layers:
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND --plane-layers B.Cu
"""
from __future__ import annotations

import sys
import os
import math
import argparse
from dataclasses import replace
from typing import List, Tuple, Dict, Optional, Set

# Run startup checks first
from startup_checks import run_all_checks
run_all_checks()

from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, KICAD_10_MIN_VERSION, pad_is_plated_through
from kicad_writer import generate_segment_sexpr, generate_gr_line_sexpr, generate_via_sexpr
from routing_config import GridRouteConfig, GridCoord
from plane_io import extract_zones, ZoneInfo
from plane_region_connector import route_disconnected_regions, build_base_obstacles, add_route_to_obstacles
import plane_pad_tap
from plane_pad_tap import (find_unconnected_plane_pads, tap_pad_with_escalation,
                           SharedViaMaps)
from plane_component_oracle import PlaneComponentOracle
from plane_blocker_detection import find_route_blocker_from_frontier, find_via_position_blocker
from terminal_colors import GREEN, RED, RESET
import routing_defaults as defaults
import re

# Outcome of the end-of-run self-reconnect of rip-blocker-nets casualties
# (#347); read by main() for the JSON_SUMMARY. None = no reconnect ran.
LAST_RIPPED_RECONNECT: Optional[Dict] = None



def _rip_net_from_pcb(pcb_data: PCBData, rip_net_id: int):
    """Remove a net's segments and vias from pcb_data so a blocked repair can
    re-attempt through the freed space. Returns (segments, vias) removed."""
    # graphic=True copper is immutable input art (#337): never ripped.
    rsegs = [s for s in pcb_data.segments
             if s.net_id == rip_net_id and not getattr(s, 'graphic', False)]
    rvias = [v for v in pcb_data.vias if v.net_id == rip_net_id]
    if rsegs:
        pcb_data.segments = [s for s in pcb_data.segments
                             if s.net_id != rip_net_id or getattr(s, 'graphic', False)]
    if rvias:
        pcb_data.vias = [v for v in pcb_data.vias if v.net_id != rip_net_id]
    return rsegs, rvias


def _via_site_consensus_blocker(pad, pcb_data, blocker_config, net_id,
                                protected_net_ids, exclude_net_ids,
                                max_search_radius):
    """The net most often blocking CANDIDATE VIA SITES around the pad (#329).

    The frontier blocker only names whoever stopped the failed ROUTE attempt;
    on ottercast the C69.2 tap ripped three frontier nets while the net whose
    trace actually denied every good via site (Net-(C63-Pad1), 0.325mm from
    the best spot) was never identified. Vote find_via_position_blocker over a
    coarse ring of sites within the search radius and return the most common
    non-protected, not-yet-ripped net."""
    from collections import Counter
    votes = Counter()
    step = max(blocker_config.grid_step * 4, 0.2)
    # Vote only the NEAR band: distant sites belong to other neighborhoods and
    # dilute the vote toward whatever net dominates the region at large
    # (ottercast: a full-radius vote elected C64 while C63 held every site the
    # pad could actually use). Closer sites also count for more.
    r = min(1.2, max_search_radius) if max_search_radius > 0 else 1.2
    n = max(3, int(r / step))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            d = math.hypot(i, j) * step
            if d < 0.2 or d > r:
                continue
            b = find_via_position_blocker(
                pad.global_x + i * step, pad.global_y + j * step,
                pcb_data, blocker_config, net_id, protected_net_ids, quiet=True)
            if b is not None and b not in protected_net_ids and b not in exclude_net_ids:
                votes[b] += 1.0 / (0.3 + d)
    return votes.most_common(1)[0][0] if votes else None


def _tap_pad_with_ripup(pad, pad_layer, net_id, pcb_data, tap_config, blocker_config,
                        max_search_radius, via_size, via_drill, max_rip_nets,
                        protected_net_ids, first_failure, ripped_net_ids, verbose,
                        distant_trace_radius=0.0, shared_via_maps=None,
                        partial_restores=None, plane_oracle=None):
    """A plane-net pad too small to drop a via in needs a trace to the plane (or
    to an adjacent same-net pad); if signal nets block that trace, rip them (up
    to max_rip_nets), retry the tap. Identifies the blocker from the failed
    route's frontier (or, when that repeats/runs dry, the consensus net denying
    the candidate via sites, #329), never ripping a protected (plane) net.

    On SUCCESS the ripped net ids are appended to ripped_net_ids for the later
    route.py reconnect pass. On FAILURE every ripped net's copper is RESTORED
    (#329): nothing else routed while this pad's tap retried (a failed tap adds
    no copper), so an immediate restore cannot create the #141 restore-shorts --
    while shipping the rips destroyed whole nets the reconnect pass then could
    not reroute (ottercast MIPI_SDA + Net-(C61-Pad1) ended at ZERO copper).
    Returns a successful TapResult, or None."""
    failure = first_failure
    ripped_local = []  # (net_id, segments, vias) in rip order
    ripped_ids_local = set()
    for attempt in range(max_rip_nets):
        blocker = None
        # The frontier blocker names whoever stopped the failed ROUTE; the net
        # denying the via SITES can be a different one that the frontier never
        # reaches (ottercast C69.2 ripped 3 fresh frontier nets while C63 held
        # every good site). Spend the LAST rip on the site-consensus net.
        if attempt < max_rip_nets - 1:
            if failure.blocked_cells:
                blocker = find_route_blocker_from_frontier(
                    failure.blocked_cells, pcb_data, blocker_config, net_id, protected_net_ids)
            else:
                blocker = find_via_position_blocker(
                    pad.global_x, pad.global_y, pcb_data, blocker_config, net_id, protected_net_ids)
            if blocker in ripped_ids_local:
                blocker = None  # frontier keeps naming an already-ripped net
        if blocker is None or blocker in protected_net_ids:
            blocker = _via_site_consensus_blocker(
                pad, pcb_data, blocker_config, net_id, protected_net_ids,
                ripped_ids_local, max_search_radius)
        if blocker is None or blocker in protected_net_ids:
            break
        bname = pcb_data.nets[blocker].name if blocker in pcb_data.nets else f"net_{blocker}"
        print(f"{RED}blocked by {bname} - ripping{RESET}...", end=" ", flush=True)
        if shared_via_maps is not None:
            # Remove the blocker's stamps from the shared via maps BEFORE its
            # copper leaves pcb_data (the stamps are computed from it), then
            # resync the copper counts after the rip (#263).
            shared_via_maps.note_net_ripped(blocker)
        rsegs, rvias = _rip_net_from_pcb(pcb_data, blocker)
        ripped_local.append((blocker, rsegs, rvias))
        ripped_ids_local.add(blocker)
        if shared_via_maps is not None:
            shared_via_maps.resync()
            if plane_pad_tap._TAP_MAP_VERIFY:
                # Rips are the #208-risk removal path: assert the incrementally
                # updated maps match a fresh rebuild of the post-rip board.
                shared_via_maps.verify_maps_full()
        result = tap_pad_with_escalation(
            pad, pad_layer, net_id, pcb_data, tap_config,
            max_search_radius=max_search_radius, via_size=via_size, via_drill=via_drill,
            verbose=verbose, fine_for_all=True, distant_trace_radius=distant_trace_radius,
            shared_via_maps=shared_via_maps, plane_oracle=plane_oracle)
        if result.success:
            # Collision-checked restore on SUCCESS too (#329): give back every
            # ripped net whose copper does not conflict with the NEW tap
            # copper. Only genuinely conflicting nets stay ripped for the
            # route.py reconnect pass -- shipping every rip gambled N routed
            # nets on that pass (which the recorded chains often run with
            # --max-ripup 0): ottercast ended with 5 zero-copper nets when
            # only the true corridor nets actually conflicted.
            from plane_blocker_detection import _restored_piece_collides
            new_segs = list(result.segments or [])
            new_vias = [result.via] if result.via else []
            clr = result.clearance_used or tap_config.clearance
            for blocker, rsegs, rvias in ripped_local:
                keep_segs, keep_vias, dropped = [], [], 0
                for s in rsegs:
                    sd = {'start': (s.start_x, s.start_y),
                          'end': (s.end_x, s.end_y),
                          'width': s.width, 'layer': s.layer}
                    if _restored_piece_collides(sd, None, new_vias, new_segs,
                                                via_size, clr):
                        dropped += 1
                    else:
                        keep_segs.append(s)
                for v in rvias:
                    vd = {'x': v.x, 'y': v.y, 'size': v.size}
                    if _restored_piece_collides(None, vd, new_vias, new_segs,
                                                via_size, clr):
                        dropped += 1
                    else:
                        keep_vias.append(v)
                if not keep_segs and not keep_vias:
                    # Nothing restorable: honest full rip for the reconnect pass.
                    if blocker not in ripped_net_ids:
                        ripped_net_ids.append(blocker)
                    continue
                pcb_data.segments.extend(keep_segs)
                pcb_data.vias.extend(keep_vias)
                if shared_via_maps is not None:
                    shared_via_maps.note_net_restored(blocker)
                if dropped:
                    # Partial: the writer must strip the net's input copper and
                    # emit the kept pieces (board==file), and the reconnect
                    # pass closes the small gap instead of re-threading the
                    # whole net through congestion.
                    if partial_restores is not None:
                        partial_restores.append((blocker, keep_segs, keep_vias, dropped))
                        bn = pcb_data.nets[blocker].name if blocker in pcb_data.nets else blocker
                        print(f"(partial restore {bn}: -{dropped} piece(s))", end=" ", flush=True)
                    elif blocker not in ripped_net_ids:
                        # No partial channel (defensive): fall back to full rip.
                        pcb_data.segments = [x for x in pcb_data.segments if x not in keep_segs]
                        pcb_data.vias = [x for x in pcb_data.vias if x not in keep_vias]
                        ripped_net_ids.append(blocker)
            return result
        failure = result
    # FINAL FAILURE: restore every ripped net's copper (#329).
    if ripped_local:
        for blocker, rsegs, rvias in reversed(ripped_local):
            pcb_data.segments.extend(rsegs)
            pcb_data.vias.extend(rvias)
            if shared_via_maps is not None:
                shared_via_maps.note_net_restored(blocker)
        print(f"(restored {len(ripped_local)} ripped net(s))", end=" ", flush=True)
    return None


def _report_unrouted_ripped_nets(pcb_data, ripped_net_ids):
    """Report the nets ripped to clear blocked pad repairs and left UNROUTED.

    route_disconnected_planes no longer re-routes ripped nets in-step (issue #141
    reverted -- its restore-on-failure put a failed net's original copper back on
    top of whatever had been routed through its freed corridor, shorting them). The
    ripped nets are stripped from the output and reconnected by a route.py pass run
    afterward, which handles rip-up/restore safely against the live obstacle map.
    """
    ripped_names = [pcb_data.nets[r].name for r in ripped_net_ids if r in pcb_data.nets]
    if not ripped_names:
        return
    print(f"\nNote: {len(ripped_names)} net(s) ripped to clear pad repairs were left "
          f"UNROUTED; reconnect them with route.py (matching signal width): "
          f"{', '.join(ripped_names)}")


def extract_zone_properties(input_file: str) -> Dict[Tuple[str, str], Dict]:
    """
    Extract zone properties (clearance, min_thickness) from PCB file.

    Returns:
        Dict mapping (net_name, layer) -> {'clearance': float, 'min_thickness': float}
    """
    with open(input_file, 'r') as f:
        content = f.read()

    zone_props = {}
    zone_pattern = r'\(zone\s*\n\s*\(net\s+\d+\)'
    matches = list(re.finditer(zone_pattern, content))

    for m in matches:
        start = m.start()
        depth = 0
        end = start
        for i, c in enumerate(content[start:]):
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
                if depth == 0:
                    end = start + i + 1
                    break

        zone_text = content[start:end]

        net_name = re.search(r'\(net_name\s+"([^"]*)"\)', zone_text)
        layer = re.search(r'\(layer\s+"([^"]+)"\)', zone_text)
        clearance = re.search(r'\(clearance\s+([\d.]+)\)', zone_text)
        min_thick = re.search(r'\(min_thickness\s+([\d.]+)\)', zone_text)

        if net_name and layer:
            key = (net_name.group(1), layer.group(1))
            zone_props[key] = {
                'clearance': float(clearance.group(1)) if clearance else 0.2,
                'min_thickness': float(min_thick.group(1)) if min_thick else 0.1
            }

    return zone_props


def auto_detect_zones(
    input_file: str,
    filter_nets: Optional[List[str]] = None,
    filter_layers: Optional[List[str]] = None
) -> List[Tuple[str, str]]:
    """
    Auto-detect zone net/layer pairs from the PCB file.

    Args:
        input_file: Path to KiCad PCB file
        filter_nets: If provided, only include these nets
        filter_layers: If provided, only include these layers

    Returns:
        List of (net_name, layer) tuples for zones to process
    """
    zones = extract_zones(input_file)

    if not zones:
        return []

    # Build list of (net_name, layer) pairs
    zone_pairs: List[Tuple[str, str]] = []
    seen = set()

    for zone in zones:
        # Apply filters
        if filter_nets and zone.net_name not in filter_nets:
            continue
        if filter_layers and zone.layer not in filter_layers:
            continue

        key = (zone.net_name, zone.layer)
        if key not in seen:
            seen.add(key)
            zone_pairs.append(key)

    return zone_pairs


def route_planes(
    input_file: str,
    output_file: str,
    net_names: List[str],
    plane_layers: List[str],
    track_width: float = defaults.TRACK_WIDTH,
    clearance: float = defaults.CLEARANCE,
    zone_clearance: float = defaults.PLANE_ZONE_CLEARANCE,
    grid_step: float = defaults.GRID_STEP,
    analysis_grid_step: float = defaults.REPAIR_ANALYSIS_GRID_STEP,
    max_track_width: float = defaults.REPAIR_MAX_TRACK_WIDTH,
    min_track_width: float = defaults.REPAIR_MIN_TRACK_WIDTH,
    track_via_clearance: float = defaults.PLANE_TRACK_VIA_CLEARANCE,
    hole_to_hole_clearance: float = defaults.HOLE_TO_HOLE_CLEARANCE,
    board_edge_clearance: float = defaults.PLANE_EDGE_CLEARANCE,
    via_size: float = defaults.VIA_SIZE,
    via_drill: float = defaults.VIA_DRILL,
    max_iterations: int = defaults.MAX_ITERATIONS,
    verbose: bool = False,
    dry_run: bool = False,
    debug_lines: bool = False,
    routing_layers: Optional[List[str]] = None,
    pcb_data: Optional[PCBData] = None,
    return_results: bool = False,
    repair_pads: bool = True,
    max_search_radius: float = defaults.PLANE_MAX_SEARCH_RADIUS,
    rip_blocker_nets: bool = False,
    max_rip_nets: int = 3,
    reroute_ripped_nets: bool = False,
    power_nets: Optional[List[str]] = None,
    power_nets_widths: Optional[List[float]] = None,
    no_bga_zone: bool = False,
    progress_callback=None,
    cancel_check=None,
    net_clearances: Optional[dict] = None,
) -> Tuple[int, int]:
    """
    Route between disconnected regions in power plane zones.

    Args:
        input_file: Path to input KiCad PCB file
        output_file: Path to output KiCad PCB file
        net_names: List of net names to process (e.g., ['GND', '+3.3V'])
        plane_layers: List of layers for each net (e.g., ['B.Cu', 'In1.Cu'])
        track_width: Default track width for routing config (mm)
        clearance: Clearance between traces (mm)
        zone_clearance: Zone fill clearance around obstacles (mm)
        grid_step: Routing grid step (mm)
        max_track_width: Maximum track width for region connections (mm)
        min_track_width: Minimum track width for region connections (mm)
        track_via_clearance: Clearance from tracks to other nets' vias (mm)
        hole_to_hole_clearance: Minimum clearance between drill holes (mm)
        board_edge_clearance: Clearance from board edge (mm)
        via_size: Via outer diameter for config (mm)
        via_drill: Via drill diameter for config (mm)
        max_iterations: Maximum A* iterations per route attempt
        verbose: Print detailed debug info
        dry_run: Analyze without writing output
        routing_layers: List of layers that can be used for routing (if None, auto-detect from PCB)
        repair_pads: If True (default), also repair pad-level plane connection
            failures (issue #99): pads of the plane net with no via/segment of
            the net within reach and not directly on a zone layer get a tap
            retry with route_planes' parameter escalation (default params,
            then scoped fine params for fine-pitch pads).
        max_search_radius: Max radius to search for a via position during pad
            repair (mm).
        progress_callback: Optional callable(current, total, label) invoked at
            phase milestones (pad repair k/N, region discovery, per-region
            connects, cleanup, ripped-net reconnect), mirroring batch_route's
            callback (issue #364). (0, 0, label) marks an indeterminate phase.
            Called from whatever thread runs the engine; GUI callers must
            marshal to the UI thread themselves.

    Returns:
        Tuple of (total_routes_added, total_regions_connected)
    """
    from route import _dump_engine_config
    _dump_engine_config('repair_planes', dict(locals()))
    # Board-setup copper-to-edge rule (#338): engine-side so the GUI planes
    # tab and plan replays inherit it; see batch_route.
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
    if pcb_data is None:
        print(f"Loading PCB from {input_file}...")
        pcb_data = parse_kicad_pcb(input_file)

    # Resolve net IDs
    net_ids = []
    for net_name in net_names:
        net_id = None
        for nid, net in pcb_data.nets.items():
            if net.name == net_name:
                net_id = nid
                break
        if net_id is None:
            print(f"Error: Net '{net_name}' not found in PCB")
            return (0, 0)
        net_ids.append(net_id)

    # Get board bounds
    board_bounds = pcb_data.board_info.board_bounds
    if not board_bounds:
        print("Error: Could not determine board bounds")
        return (0, 0)

    min_x, min_y, max_x, max_y = board_bounds
    print(f"Board bounds: ({min_x:.2f}, {min_y:.2f}) to ({max_x:.2f}, {max_y:.2f})")

    # Zone bounds with edge clearance
    zone_bounds = (
        min_x + board_edge_clearance,
        min_y + board_edge_clearance,
        max_x - board_edge_clearance,
        max_y - board_edge_clearance
    )

    # Build routing config
    config = GridRouteConfig(
        track_width=track_width,
        clearance=clearance,
        via_size=via_size,
        via_drill=via_drill,
        grid_step=grid_step,
        board_edge_clearance=board_edge_clearance
    )

    # Cross-class clearance (#434): the repair step's own copper (region joins,
    # pad taps) and its ripped-blocker reconnects were priced at the uniform
    # clearance only, so repair copper landed inside fat-class bands (cparti
    # BTN4 reconnect 0.20-0.31mm from SMA-class copper whose class demands
    # 0.35). Mirror batch_route's contract: auto-read the board's non-Default
    # netclasses from the INPUT's sibling .kicad_pro when no map was passed
    # (id-keyed; all-Default boards -> empty map -> inert), stamp them on the
    # repair config, and forward the map to the reconnect sub-runs below --
    # those route from the OUTPUT file, whose sibling .kicad_pro may not exist
    # yet (same hazard as the #338 edge resolution).
    if net_clearances is None and input_file and os.path.isfile(input_file):
        try:
            from list_nets import net_clearance_map_by_id
            net_clearances = net_clearance_map_by_id(
                input_file, {nid: n.name for nid, n in pcb_data.nets.items()})
            if net_clearances:
                print(f"Auto-read netclass clearances for "
                      f"{len(net_clearances)} net(s) (cross-class max(A,B) "
                      f"respected during plane repair).")
        except Exception as _e:
            print(f"Warning: could not auto-read netclass clearances ({_e}); "
                  f"repairing at the uniform clearance.")
            net_clearances = None
    if net_clearances:
        config.net_clearances = dict(net_clearances)

    # Auto-detect routing layers if not specified
    if routing_layers is None:
        routing_layers = pcb_data.board_info.copper_layers
        if not routing_layers:
            routing_layers = ['F.Cu', 'B.Cu']  # Fallback
    # NOTE: unlike batch_route, routing_layers here directly selects layers
    # region joins may PLACE copper on (not cost-driven), so no full-stack
    # append -- the default above is already the whole board, and the
    # off-layer via guard in build_base_obstacle_map/obstacle_cache covers
    # the explicit-subset case for via placement.
    print(f"Routing layers: {', '.join(routing_layers)}")

    # Issue #293 guard: snapshot every multi-pad SIGNAL net's connectivity so a
    # regression this repair pass causes (rip cascades, tap side effects) is
    # caught and reported loudly at the end instead of shipping silently. The
    # per-net union-find over the whole board costs well under a second.
    from check_connected import check_net_connectivity as _cnc293
    _zones_by_net_293: Dict[int, list] = {}
    for _z in (getattr(pcb_data, 'zones', None) or []):
        _zones_by_net_293.setdefault(_z.net_id, []).append(_z)
    _segs_293: Dict[int, list] = {}
    for _s in pcb_data.segments:
        _segs_293.setdefault(_s.net_id, []).append(_s)
    _vias_293: Dict[int, list] = {}
    for _v in pcb_data.vias:
        _vias_293.setdefault(_v.net_id, []).append(_v)
    _plane_net_ids = set(net_ids)
    _pre_connected_293 = set()
    for _nid, _pads in pcb_data.pads_by_net.items():
        if _nid in _plane_net_ids or len(_pads) < 2:
            continue
        if not (_segs_293.get(_nid) or _vias_293.get(_nid) or _zones_by_net_293.get(_nid)):
            continue  # unrouted before us; not ours to regress
        _r = _cnc293(_nid, _segs_293.get(_nid, []), _vias_293.get(_nid, []),
                     _pads, _zones_by_net_293.get(_nid, []))
        if _r.get('connected'):
            _pre_connected_293.add(_nid)

    all_new_segments: List[Dict] = []
    all_new_vias: List[Dict] = []
    all_debug_lines: List[str] = []
    total_routes = 0
    total_regions = 0
    total_vias = 0
    total_pads_unconnected = 0
    # Pads the repair taps are fill-unreachable BY DIAGNOSIS -- their tap
    # copper must never be removed by the graze/dead-end cleanup (whose
    # connectivity gate credits the pour OUTLINE and would grade the taps
    # redundant: Andy's bitaxe Q2 shredded-stub opens).
    _tapped_pads = []
    total_pads_repaired = 0
    failed_repair_pads: List[str] = []

    # Extract per-zone clearances and min_thickness from PCB file
    zone_props = extract_zone_properties(input_file)
    if verbose:
        print(f"Zone properties:")
        for (net, layer), props in zone_props.items():
            print(f"  {net} on {layer}: clearance={props['clearance']}mm, min_thickness={props['min_thickness']}mm")

    print(f"\n{'='*60}")
    print(f"Routing disconnected plane regions")
    print(f"{'='*60}")

    # Group zones by net - process each net once with all its zone layers
    unique_nets: Dict[int, Tuple[str, Set[str]]] = {}  # net_id -> (net_name, set of layers)
    for net_name, plane_layer, net_id in zip(net_names, plane_layers, net_ids):
        if net_id not in unique_nets:
            unique_nets[net_id] = (net_name, set())
        unique_nets[net_id][1].add(plane_layer)

    # --reroute-ripped-nets is deprecated (issue #141 reverted). This step used to
    # rip signal blockers, route plane/pad repairs into the freed space, then
    # re-route the ripped nets -- but a ripped net that FAILED to re-route had its
    # ORIGINAL copper restored on top of whatever had meanwhile been routed through
    # its corridor, creating P-to-N shorts the obstacle map never saw (a restore
    # bypasses it). Rerouting is now left to a route.py pass run AFTER this step,
    # which handles rip-up/restore safely. We still rip blockers for pad repair and
    # leave them UNROUTED (stripped) for that route.py pass to reconnect.
    if reroute_ripped_nets:
        print("Note: --reroute-ripped-nets is deprecated and now a no-op. Ripped nets "
              "are left unrouted; run route.py afterward to reconnect them (it does "
              "rip-up/restore safely).")
        reroute_ripped_nets = False

    # Plane nets are never ripped to clear a blocker (--rip-blocker-nets); only
    # signal nets are, and they are left unrouted for a subsequent route.py pass.
    plane_net_ids = set(unique_nets.keys())
    ripped_net_ids: List[int] = []
    # (net_id, kept_segs, kept_vias, dropped_count) for nets partially
    # restored by the success-path settle: input copper stripped at write,
    # kept pieces emitted as new copper (board==file), gap left for reconnect.
    partial_restores: List = []
    # Trace-to-existing-plane-copper reaches the full via-search radius
    # (max_search_radius, the --max-search-radius CLI value) so a boxed pad whose
    # nearest existing same-net via sits past a smaller cap is still reachable
    # (issue #180: castor_pollux U5.4's nearest GND via was at 4.62mm).
    # Without --rip-blocker-nets the trace step still runs at STRAP scale
    # (issue #349): an unconnected pad adjacent to an already-repaired same-net
    # pad straps to it instead of drilling another via, so a fine-pitch pad
    # cluster shares one via + short straps.
    distant_radius = (max_search_radius if rip_blocker_nets
                      else min(max_search_radius, defaults.PLANE_PAD_STRAP_RADIUS))

    for net_id, (net_name, net_zone_layers) in unique_nets.items():
        if cancel_check and cancel_check():
            print("\nPlane repair cancelled")
            break
        # Build per-layer zone clearances for all layers with zones for this net
        # These are used in flood fill to determine what the zone fill connects
        zone_clearances: Dict[str, float] = {}
        for layer in net_zone_layers:
            zk = (net_name, layer)
            if zk in zone_props:
                zone_clearances[layer] = zone_props[zk]['clearance']

        # Use maximum clearance as fallback (per-layer clearances used in flood fill)
        max_zone_clearance = max(zone_clearances.values()) if zone_clearances else zone_clearance

        # Pick first zone layer as "primary" (for plane_layer_idx in routing)
        primary_layer = sorted(net_zone_layers)[0]

        layers_str = ", ".join(sorted(net_zone_layers))
        clearances_str = ", ".join(f"{l}={zone_clearances.get(l, zone_clearance)}mm" for l in sorted(net_zone_layers))
        print(f"\n[{net_name}] on {layers_str} (clearances: {clearances_str}):")

        # Per-pad repair pass (issue #99): reconnect pads route_planes could
        # not via down to the plane. Runs before island repair so the new
        # vias participate in the region connectivity analysis.
        if repair_pads:
            if progress_callback:
                progress_callback(0, 0, f"{net_name}: checking pad-plane connections...")
            unconnected = find_unconnected_plane_pads(pcb_data, net_id, net_zone_layers)
            total_pads_unconnected += len(unconnected)
            if not unconnected:
                print(f"  Pad repair: all {net_name} pads reach the plane")
            else:
                print(f"  Pad repair: {len(unconnected)} pad(s) with no connection to the {net_name} plane:")
                tap_config = replace(
                    config,
                    layers=routing_layers,
                    hole_to_hole_clearance=hole_to_hole_clearance
                )
                # Cross-pad via-obstacle-map reuse for this net's repair pass (#263).
                shared_maps = SharedViaMaps(pcb_data, net_id)
                # T6 mutual-floating-strap guard: components of this net's
                # copper, built ONCE (same union-find as check_connected) and
                # updated incrementally as taps commit. Tap targets not in the
                # MAIN plane component are rejected, so two pads can no longer
                # strap to each other's floating island and both report success.
                plane_oracle = PlaneComponentOracle(pcb_data, net_id)
                if plane_oracle.n_floating_items:
                    print(f"    (plane oracle: {plane_oracle.n_floating_items} "
                          f"floating same-net item(s) excluded as tap targets)")
                for _pr_idx, (pad, pad_layer) in enumerate(unconnected):
                    if cancel_check and cancel_check():
                        print("    (cancelled)")
                        break
                    _tapped_pads.append(pad)
                    if progress_callback:
                        progress_callback(_pr_idx + 1, len(unconnected),
                                          f"{net_name}: pad repair "
                                          f"{pad.component_ref}.{pad.pad_number}")
                    print(f"    Pad {pad.component_ref}.{pad.pad_number} ({pad_layer})...", end=" ", flush=True)
                    result = tap_pad_with_escalation(
                        pad, pad_layer, net_id, pcb_data, tap_config,
                        max_search_radius=max_search_radius,
                        via_size=via_size,
                        via_drill=via_drill,
                        verbose=verbose,
                        fine_for_all=True,  # last-resort repair: escalate every failed pad
                        distant_trace_radius=distant_radius,
                        shared_via_maps=shared_maps,
                        plane_oracle=plane_oracle
                    )
                    if not result.success and rip_blocker_nets:
                        # Rip the signal net(s) blocking this pad's trace and retry
                        # (the ripped nets are re-routed after the repair pass).
                        rr = _tap_pad_with_ripup(
                            pad, pad_layer, net_id, pcb_data, tap_config, config,
                            max_search_radius, via_size, via_drill, max_rip_nets,
                            plane_net_ids, result, ripped_net_ids, verbose,
                            distant_trace_radius=distant_radius,
                            shared_via_maps=shared_maps,
                            partial_restores=partial_restores,
                            plane_oracle=plane_oracle)
                        if rr is not None:
                            result = rr
                    if result.success:
                        total_pads_repaired += 1
                        params_note = " [fine params]" if result.params_label == 'fine' else ""
                        new_via_objs = []
                        new_seg_objs = []
                        if result.via is not None:
                            all_new_vias.append(result.via)
                            total_vias += 1
                            new_via_objs.append(Via(
                                x=result.via['x'], y=result.via['y'],
                                size=result.via['size'], drill=result.via['drill'],
                                layers=['F.Cu', 'B.Cu'], net_id=net_id
                            ))
                            pcb_data.vias.append(new_via_objs[0])
                            where = f"placed via at ({result.via['x']:.2f}, {result.via['y']:.2f})"
                        else:
                            where = (f"reused via at ({result.reused_via_pos[0]:.2f}, "
                                     f"{result.reused_via_pos[1]:.2f})")
                        for s in result.segments:
                            all_new_segments.append(s)
                            seg_obj = Segment(
                                start_x=s['start'][0], start_y=s['start'][1],
                                end_x=s['end'][0], end_y=s['end'][1],
                                width=s['width'], layer=s['layer'], net_id=s['net_id']
                            )
                            new_seg_objs.append(seg_obj)
                            pcb_data.segments.append(seg_obj)
                        shared_maps.note_pass_copper(new_via_objs, new_seg_objs)
                        # T6: this tap was oracle-verified to reach the main
                        # plane; credit its pad + copper so later pads may
                        # strap to them (transitive, no graph rebuild).
                        plane_oracle.note_tap_committed(pad, new_via_objs,
                                                        new_seg_objs)
                        print(f"{GREEN}{where}, {len(result.segments)} trace segment(s){params_note}{RESET}")
                    else:
                        failed_repair_pads.append(f"{pad.component_ref}.{pad.pad_number} ({net_name})")
                        print(f"{RED}FAILED{RESET}")

        # Build obstacle map for this net
        if progress_callback:
            progress_callback(0, 0, f"{net_name}: building obstacle map...")
        print(f"  Building obstacle map...", end=" ", flush=True)
        base_obstacles, layer_map = build_base_obstacles(
            exclude_net_ids={net_id},
            routing_layers=routing_layers,
            pcb_data=pcb_data,
            config=config,
            # Base keep-outs use the min connection width; the widen step then
            # passes the extra half-width to the router, which now does an exact
            # swept-capsule clearance check (issues #156/#173), so a wide (e.g.
            # 0.4mm) connection's diagonal no longer grazes foreign copper.
            track_width=min_track_width,
            track_via_clearance=track_via_clearance,
            hole_to_hole_clearance=hole_to_hole_clearance
        )
        print("done")

        region_segments, region_vias, routes_added, route_paths, _ = route_disconnected_regions(
            net_id=net_id,
            net_name=net_name,
            plane_layer=primary_layer,
            zone_bounds=zone_bounds,
            pcb_data=pcb_data,
            config=config,
            base_obstacles=base_obstacles,
            layer_map=layer_map,
            zone_clearance=max_zone_clearance,
            max_track_width=max_track_width,
            min_track_width=min_track_width,
            track_via_clearance=track_via_clearance,
            hole_to_hole_clearance=hole_to_hole_clearance,
            analysis_grid_step=analysis_grid_step,
            max_iterations=max_iterations,
            verbose=verbose,
            zone_layers=net_zone_layers,
            zone_clearances=zone_clearances,
            progress_callback=progress_callback,
            cancel_check=cancel_check
        )

        if routes_added > 0:
            all_new_segments.extend(region_segments)
            all_new_vias.extend(region_vias)
            total_routes += routes_added
            total_regions += routes_added + 1  # N routes connect N+1 regions
            total_vias += len(region_vias)

            # Generate debug lines for this net's routes (on User.4)
            if debug_lines and route_paths:
                for route_path in route_paths:
                    for i in range(len(route_path) - 1):
                        p1, p2 = route_path[i], route_path[i + 1]
                        all_debug_lines.append(generate_gr_line_sexpr(p1, p2, 0.1, "User.4"))

            # Add segments to pcb_data so subsequent nets see them as obstacles
            for s in region_segments:
                start = s['start']
                end = s['end']
                pcb_data.segments.append(Segment(
                    start_x=start[0], start_y=start[1],
                    end_x=end[0], end_y=end[1],
                    width=s['width'], layer=s['layer'], net_id=s['net_id']
                ))

            # Add vias to pcb_data so subsequent nets see them as obstacles
            for v in region_vias:
                pcb_data.vias.append(Via(
                    x=v['x'], y=v['y'],
                    size=v['size'], drill=v['drill'],
                    layers=['F.Cu', 'B.Cu'],  # Through-hole vias
                    net_id=v['net_id']
                ))

    # Final fill-aware verification (glasgow U30 U1.27 +3V3). The per-pad check
    # (find_unconnected_plane_pads / _smd_pad_reaches_layer) is layer-aware: it
    # treats a pad as connected once it reaches the zone LAYER, even via a
    # floating island, so a reuse-tap onto an island reports success while the pad
    # never reaches the connected plane FILL. Re-check each plane net with the same
    # zone/fill-aware union-find check_connected uses (cheap - ~0.5s/board, once at
    # the end), and force a real via (disable_reuse) for any pad still floating, so
    # no plane pad is left SILENTLY disconnected after reporting success.
    if repair_pads:
        from check_connected import check_net_connectivity
        # The ripped signal nets' old copper is excluded from the OUTPUT but still
        # sits in pcb_data here (stripped only at write time). Drop it before the
        # sweep so a forced via sees the same obstacles as the written board -- else
        # a via site that is clear in the output looks blocked and the pad is wrongly
        # left floating.
        if ripped_net_ids:
            pcb_data.segments = [s for s in pcb_data.segments
                                 if s.net_id not in ripped_net_ids]
            pcb_data.vias = [v for v in pcb_data.vias if v.net_id not in ripped_net_ids]
        zones_by_net: Dict[int, list] = {}
        for z in (getattr(pcb_data, 'zones', None) or []):
            if getattr(z, 'net_id', None) is not None:
                zones_by_net.setdefault(z.net_id, []).append(z)
        # Forced last-resort via sizes, largest first, as fab-manufacturable
        # (diameter, drill) pairs: the configured via, then the active fab-tier floor
        # ladder (nominal floor, then any escalation rung -- the more-costly advanced
        # 0.25/0.15 via 'standard' escalates to, #237). A fine-pitch pad flanked by
        # other-net copper often cannot take the nominal via but fits a smaller
        # fab-legal one; we never go below the deepest fab floor.
        from list_nets import fab_floor_ladder, warn_fab_escalation
        _ncu = len([l for l in (pcb_data.board_info.copper_layers or routing_layers)
                    if l.endswith('.Cu')]) or 2
        _ladder = fab_floor_ladder(_ncu)
        _cands = [(via_size, via_drill, False)]
        _cands += [(f['via_diameter'], f['via_drill'], _i > 0)
                   for _i, f in enumerate(_ladder)]
        via_pairs, _escalated_pairs = [], set()
        for _vd, _dr, _is_esc in _cands:
            _vd, _dr = round(_vd, 3), round(_dr, 3)
            if _dr < _vd <= via_size + 1e-9 and (_vd, _dr) not in via_pairs:
                via_pairs.append((_vd, _dr))
                if _is_esc:
                    _escalated_pairs.add((_vd, _dr))
        for net_id, (net_name, net_zone_layers) in unique_nets.items():
            net_segs = [s for s in pcb_data.segments if s.net_id == net_id]
            net_vias = [v for v in pcb_data.vias if v.net_id == net_id]
            net_pads = pcb_data.pads_by_net.get(net_id, [])
            res = check_net_connectivity(net_id, net_segs, net_vias, net_pads,
                                         zones_by_net.get(net_id, []))
            if res.get('connected'):
                continue
            pad_by_key = {}
            for p in net_pads:
                pl = next((l for l in p.layers
                           if l.endswith('.Cu') and not l.startswith('*')), None)
                pad_by_key[(round(p.global_x, 3), round(p.global_y, 3),
                            p.component_ref)] = (p, pl)
            # Relax the board-edge clearance for this forced last-resort tap: the
            # pad being repaired is already placed at the edge, so a via INSIDE it
            # is no closer to the edge than the pad itself (the fab accepts that),
            # and an edge pad would otherwise be unconnectable -- which is what made
            # the normal tap fall back to a bogus reuse in the first place.
            tap_config = replace(config, layers=routing_layers,
                                 hole_to_hole_clearance=hole_to_hole_clearance,
                                 board_edge_clearance=0.0)
            # Cross-pad via-map reuse for this net's forced-via sweep (#263). A
            # fresh instance (not the repair pass's): region-connect copper was
            # added since, and this pass's edge-relaxed config keys differ anyway.
            shared_maps = SharedViaMaps(pcb_data, net_id)
            # Fresh T6 oracle for the same reason (copper changed since the
            # repair pass): forces the last-resort via into a MAIN zone outline
            # and keeps the #373 track fallback off floating same-net copper.
            sweep_oracle = PlaneComponentOracle(pcb_data, net_id)
            reported = False
            for (fx, fy, _flayer, fref) in res.get('disconnected_pads', []):
                pp = pad_by_key.get((round(fx, 3), round(fy, 3), fref))
                if pp is None:
                    continue
                pad, pad_layer = pp
                # Plated barrels are already plane-tied by the fill; NPTH holes
                # have no copper at all to connect (#328). Both skip.
                if (pad_layer is None or pad_is_plated_through(pad)
                        or getattr(pad, 'pad_type', '') == 'np_thru_hole'):
                    continue
                name = f"{pad.component_ref}.{pad.pad_number} ({net_name})"
                if not reported:
                    print(f"\n[{net_name}] fill-aware re-check: pad(s) reported tapped but "
                          f"still floating (reached an island, not the plane) -- forcing a via:")
                    reported = True
                print(f"    Pad {pad.component_ref}.{pad.pad_number} ({pad_layer})...",
                      end=" ", flush=True)
                # Try each fab-legal via (largest first): a fine-pitch pad flanked
                # by other-net copper often cannot take the nominal via but fits a
                # smaller fab-floor one. Search the full max_search_radius so a pad
                # whose only open via site is farther out is still reached (the
                # batched grid_router query keeps the wide search cheap). Skip the
                # distant-trace fallback - we want a real via, nearest-first.
                result = None
                for vtry, dtry in via_pairs:
                    result = tap_pad_with_escalation(
                        pad, pad_layer, net_id, pcb_data,
                        replace(tap_config, via_size=vtry, via_drill=dtry),
                        max_search_radius=max_search_radius, via_size=vtry,
                        via_drill=dtry, verbose=verbose, fine_for_all=True,
                        distant_trace_radius=0.0, disable_reuse=True,
                        shared_via_maps=shared_maps, plane_oracle=sweep_oracle)
                    if result.success and result.via is not None:
                        if (vtry, dtry) in _escalated_pairs:
                            warn_fab_escalation(f"last-resort plane via for net "
                                                f"{net_id} ({vtry}/{dtry}mm)")
                        break
                if result.success and result.via is not None:
                    new_via_obj = Via(
                        x=result.via['x'], y=result.via['y'], size=result.via['size'],
                        drill=result.via['drill'], layers=['F.Cu', 'B.Cu'], net_id=net_id)
                    pcb_data.vias.append(new_via_obj)
                    new_seg_objs = []
                    for s in result.segments:
                        seg_obj = Segment(
                            start_x=s['start'][0], start_y=s['start'][1],
                            end_x=s['end'][0], end_y=s['end'][1],
                            width=s['width'], layer=s['layer'], net_id=s['net_id'])
                        new_seg_objs.append(seg_obj)
                        pcb_data.segments.append(seg_obj)
                    # VERIFY the forced via actually joins this pad to the plane
                    # (issue #287, neptune): a "successful" via can land in the
                    # gap between Voronoi cells -- DRC-clean, touching no fill --
                    # so re-run the fill-aware union-find for the pad before
                    # claiming success, and UNDO the via if it is still floating.
                    _v_segs = [s for s in pcb_data.segments if s.net_id == net_id]
                    _v_vias = [v for v in pcb_data.vias if v.net_id == net_id]
                    _v_res = check_net_connectivity(net_id, _v_segs, _v_vias, net_pads,
                                                    zones_by_net.get(net_id, []))
                    _pad_key = (round(pad.global_x, 3), round(pad.global_y, 3),
                                pad.component_ref)
                    _still = {(round(x, 3), round(y, 3), ref)
                              for (x, y, _l, ref) in (_v_res.get('disconnected_pads') or [])}
                    if _pad_key in _still:
                        pcb_data.vias.remove(new_via_obj)
                        for seg_obj in new_seg_objs:
                            pcb_data.segments.remove(seg_obj)
                        if name not in failed_repair_pads:
                            failed_repair_pads.append(name)
                            total_pads_repaired = max(0, total_pads_repaired - 1)
                        print(f"{RED}STILL FLOATING (forced via at "
                              f"({result.via['x']:.2f}, {result.via['y']:.2f}) reaches "
                              f"no plane copper - removed){RESET}")
                        continue
                    all_new_vias.append(result.via)
                    total_vias += 1
                    for s in result.segments:
                        all_new_segments.append(s)
                    shared_maps.note_pass_copper([new_via_obj], new_seg_objs)
                    sweep_oracle.note_tap_committed(pad, [new_via_obj], new_seg_objs)
                    if name in failed_repair_pads:
                        failed_repair_pads.remove(name)
                        total_pads_repaired += 1
                    print(f"{GREEN}forced via at ({result.via['x']:.2f}, "
                          f"{result.via['y']:.2f}){RESET}")
                else:
                    # #373 last resort: no via could tie this pad to the plane
                    # (boxed-in pocket, fine-pitch WLCSP, deep-layer pour). Route
                    # a plain track from the pad to the nearest same-net copper /
                    # its own pour on the pad's layer -- the via-or-nothing ladder
                    # otherwise abandons a pad a short trace would connect. The
                    # island-join fill test validates the target; re-run the SAME
                    # fill-aware check and UNDO the track if still floating.
                    track_res = tap_pad_with_escalation(
                        pad, pad_layer, net_id, pcb_data,
                        replace(tap_config, via_size=via_size, via_drill=via_drill),
                        max_search_radius=max_search_radius,
                        via_size=via_size, via_drill=via_drill,
                        verbose=verbose, fine_for_all=True, pour_trace_only=True,
                        distant_trace_radius=max_search_radius, disable_reuse=True,
                        plane_oracle=sweep_oracle)
                    connected = False
                    if track_res.success and track_res.segments:
                        new_seg_objs = []
                        for s in track_res.segments:
                            seg_obj = Segment(
                                start_x=s['start'][0], start_y=s['start'][1],
                                end_x=s['end'][0], end_y=s['end'][1],
                                width=s['width'], layer=s['layer'], net_id=s['net_id'])
                            new_seg_objs.append(seg_obj)
                            pcb_data.segments.append(seg_obj)
                        _t_segs = [s for s in pcb_data.segments if s.net_id == net_id]
                        _t_vias = [v for v in pcb_data.vias if v.net_id == net_id]
                        _t_res = check_net_connectivity(net_id, _t_segs, _t_vias, net_pads,
                                                        zones_by_net.get(net_id, []))
                        _pad_key = (round(pad.global_x, 3), round(pad.global_y, 3),
                                    pad.component_ref)
                        _still = {(round(x, 3), round(y, 3), ref)
                                  for (x, y, _l, ref) in (_t_res.get('disconnected_pads') or [])}
                        if _pad_key in _still:
                            for seg_obj in new_seg_objs:
                                pcb_data.segments.remove(seg_obj)
                        else:
                            connected = True
                            for s in track_res.segments:
                                all_new_segments.append(s)
                            shared_maps.note_pass_copper([], new_seg_objs)
                            sweep_oracle.note_tap_committed(pad, [], new_seg_objs)
                            if name in failed_repair_pads:
                                failed_repair_pads.remove(name)
                                total_pads_repaired += 1
                            print(f"{GREEN}connected by track to same-net copper{RESET}")
                    if not connected:
                        if name not in failed_repair_pads:
                            failed_repair_pads.append(name)
                            total_pads_repaired = max(0, total_pads_repaired - 1)
                        print(f"{RED}STILL FLOATING{RESET}")

    # Drop a redundant plane-repair tap that grazes a foreign pad below clearance,
    # or re-bend a load-bearing one around the pad (#224). A tap that merely bridges
    # two pads already tied into the pour is redundant -- dropping it clears the
    # graze (e.g. ddr5 GND tap grazing the LBDQ connector pad). Connectivity-gated
    # (WITH the pour), so a load-bearing tap is kept and re-bent instead. route.py's
    # reconnect excludes the plane nets, so they are only ever cleaned up here.
    if all_new_segments:
        if progress_callback:
            progress_callback(0, 0, "Cleaning up repair copper (graze prune/nudge)...")
        from pcb_modification import cleanup_plane_taps_grazing
        _scope = {s['net_id'] for s in all_new_segments}
        all_new_segments, _gz_rm, _gz_nudge, _gz_swept = cleanup_plane_taps_grazing(
            pcb_data, all_new_segments, _scope, clearance=clearance,
            max_shift=config.grid_step / 2, all_new_vias=all_new_vias,
            hole_to_hole=config.hole_to_hole_clearance,
            protected_pads=_tapped_pads)
        if _gz_rm:
            print(f"  Graze prune: removed {_gz_rm} grazing repair segment(s)")
        if _gz_nudge:
            print(f"  Graze nudge: re-bent grazing tap jog(s) on {_gz_nudge} net(s)")
        if _gz_swept:
            print(f"  Dead-end sweep: trimmed {_gz_swept} orphaned repair segment(s)")

    # Issue #293: re-verify the signal nets that were connected when we started.
    # Ripped nets are excluded (they are honestly reported + stripped for a
    # reconnect pass); anything ELSE this pass broke is a repair bug -- surface
    # it loudly so the pipeline reconnects it instead of shipping it silently.
    _regressed_293 = []
    if _pre_connected_293:
        _segs_now: Dict[int, list] = {}
        for _s in pcb_data.segments:
            _segs_now.setdefault(_s.net_id, []).append(_s)
        _vias_now: Dict[int, list] = {}
        for _v in pcb_data.vias:
            _vias_now.setdefault(_v.net_id, []).append(_v)
        for _nid in sorted(_pre_connected_293):
            if _nid in (ripped_net_ids or []) or \
               _nid in {pr[0] for pr in partial_restores}:
                continue  # ripped/partial nets are reported for reconnect separately
            _pads = pcb_data.pads_by_net.get(_nid, [])
            _r = _cnc293(_nid, _segs_now.get(_nid, []), _vias_now.get(_nid, []),
                         _pads, _zones_by_net_293.get(_nid, []))
            if not _r.get('connected'):
                _net = pcb_data.nets.get(_nid)
                _regressed_293.append(
                    (_net.name if _net else f"net_{_nid}",
                     len(_r.get('disconnected_pads') or [])))
        if _regressed_293:
            print(f"\n{RED}WARNING: this repair pass DISCONNECTED "
                  f"{len(_regressed_293)} previously-connected signal net(s) "
                  f"(issue #293) - re-route them before shipping:{RESET}")
            for _nm, _ndisc in _regressed_293:
                print(f"  {RED}{_nm}: {_ndisc} pad(s) now disconnected{RESET}")

    # Print summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"  Zones processed: {len(net_names)}")
    print(f"  Total routes added: {total_routes}")
    if total_vias > 0:
        print(f"  Total vias added: {total_vias}")
    if repair_pads:
        print(f"  Pad repair: {total_pads_repaired}/{total_pads_unconnected} unconnected pad(s) reconnected")
        if failed_repair_pads:
            print(f"  Pads still unconnected: {', '.join(failed_repair_pads)}")
    if debug_lines and all_debug_lines:
        print(f"  Debug lines on User.4: {len(all_debug_lines)}")

    # Close soft joints in this run's copper (#334) -- see route_planes;
    # repair/reroute copper never passed through the cleanup pipeline.
    try:
        from pcb_modification import close_soft_joints
        _bridge_results: List[Dict] = []
        _nb = close_soft_joints(_bridge_results, pcb_data, None, config)
        if _nb:
            for _br in _bridge_results:
                for _bs in _br.get('new_segments', []):
                    all_new_segments.append({
                        'start': (_bs.start_x, _bs.start_y),
                        'end': (_bs.end_x, _bs.end_y),
                        'width': _bs.width, 'layer': _bs.layer,
                        'net_id': _bs.net_id})
            print(f"  Closed {_nb} soft joint(s) in repair copper")
    except Exception as _e:
        print(f"  (soft-joint close skipped: {_e})")

    kv10_names = pcb_data.net_id_to_name if pcb_data.kicad_version >= KICAD_10_MIN_VERSION else None

    # GUI (return_results): hand the plane/repair copper + the ripped net ids back.
    # The caller deletes the ripped nets' old tracks (leaving them unrouted); the
    # user reconnects them by running the routing tab afterward (#141 reverted - no
    # in-step reroute). No file is written here.
    # Partial restores: emit kept pieces as new copper and strip the nets'
    # input copper (replacement semantics -- same as route_planes b2557cd).
    # A net can be partially restored more than once (re-ripped by a later
    # pad); only its LATEST kept-set is live in pcb_data -- emitting earlier
    # sets would duplicate copper (the route_planes stale-emission bug).
    _latest: Dict[int, tuple] = {}
    for _entry in partial_restores:
        _latest[_entry[0]] = _entry
    # A net re-ripped later and left FULLY ripped must not emit its stale
    # earlier kept-set: full rip wins (it is in ripped_net_ids, zero copper).
    for _rid in ripped_net_ids:
        _latest.pop(_rid, None)
    partial_ids: List[int] = []
    for _pid, _ksegs, _kvias, _dropped in _latest.values():
        if _pid not in partial_ids:
            partial_ids.append(_pid)
        for _ks in _ksegs:
            all_new_segments.append({'start': (_ks.start_x, _ks.start_y),
                                     'end': (_ks.end_x, _ks.end_y),
                                     'width': _ks.width, 'layer': _ks.layer,
                                     'net_id': _pid})
        for _kv in _kvias:
            all_new_vias.append({'x': _kv.x, 'y': _kv.y, 'size': _kv.size,
                                 'drill': _kv.drill, 'layers': _kv.layers,
                                 'net_id': _pid})
    if partial_ids:
        _pnames = [pcb_data.nets[p].name if p in pcb_data.nets else f"net_{p}"
                   for p in partial_ids]
        print(f"Note: {len(partial_ids)} net(s) partially preserved (colliding pieces "
              f"dropped for pad repairs); reconnect them with route.py: "
              f"{', '.join(_pnames)}")

    # Reuse same-net vias that violate hole-to-hole (a region join can place a via a
    # grid cell from an existing same-net one). After ALL vias are collected (incl.
    # partial-restore kept vias above) and before both the GUI-return and file-write
    # paths, so CLI and GUI emit the same merged set.
    from pcb_modification import merge_close_same_net_vias
    merge_close_same_net_vias(all_new_vias, all_new_segments, pcb_data,
                              config.hole_to_hole_clearance)

    if return_results:
        if ripped_net_ids:
            _report_unrouted_ripped_nets(pcb_data, ripped_net_ids)
        # GUI/stress parity (gap closure): the CLI main runs three
        # post-passes on its written file that the GUI path used to skip --
        # rip-casualty self-reconnect, the shared plane-copper cleanup, and
        # the kicad-oracle recheck (the oracle runs in the planes tab after
        # apply, where the LIVE board can be temp-saved). The first two run
        # here, in memory.
        _cas = list(dict.fromkeys(ripped_net_ids + partial_ids))
        # (GUI passes dry_run=True meaning 'no file write'; routing already
        # happened, so the reconnect still runs.)
        if _cas:
            _cnames = [pcb_data.nets[n].name for n in _cas
                       if n in pcb_data.nets]
            if _cnames:
                print(f"\nReconnecting {len(_cnames)} net(s) this run ripped "
                      f"for pad repairs (in-memory): {', '.join(_cnames)}")
                if progress_callback:
                    progress_callback(0, 0, f"Reconnecting {len(_cnames)} ripped net(s)...")
                try:
                    from route import batch_route
                    _ok, _fail, _t, _rdata = batch_route(
                        input_file, "", _cnames,
                        layers=routing_layers,
                        track_width=track_width, clearance=clearance,
                        via_size=via_size, via_drill=via_drill,
                        grid_step=grid_step, max_iterations=max_iterations,
                        power_nets=power_nets,
                        power_nets_widths=power_nets_widths,
                        disable_bga_zones=([] if no_bga_zone else None),
                        net_clearances=net_clearances,
                        return_results=True, pcb_data=pcb_data)
                    def _sd(_s):
                        return {'start': (_s.start_x, _s.start_y),
                                'end': (_s.end_x, _s.end_y),
                                'width': _s.width, 'layer': _s.layer,
                                'net_id': _s.net_id}

                    def _vd(_v):
                        return {'x': _v.x, 'y': _v.y, 'size': _v.size,
                                'drill': _v.drill, 'layers': _v.layers,
                                'net_id': _v.net_id}
                    for _r in _rdata.get('results', []):
                        all_new_segments.extend(
                            _sd(_s) for _s in (_r.get('new_segments') or []))
                        all_new_vias.extend(
                            _vd(_v) for _v in (_r.get('new_vias') or []))
                    all_new_vias.extend(
                        _vd(_v) for _v in (_rdata.get('all_swap_vias') or []))
                    all_new_segments.extend(
                        _sd(_s) for _s in
                        (_rdata.get('all_swap_segments') or []))
                    if _fail:
                        print(f"{RED}  {_fail} ripped net(s) could NOT be "
                              f"reconnected{RESET}")
                except Exception as _e:
                    print(f"{RED}  in-memory reconnect failed: {_e}{RESET}")
        # Shared plane-copper cleanup, in memory (the CLI runs
        # clean_plane_copper on its written file). Removed emissions drop
        # from all_new_* in place; removed INPUT copper is returned in the
        # new strip channel for the GUI applier.
        _strip_segments = []
        try:
            from types import SimpleNamespace
            from cleanup_pipeline import run_post_route_cleanup
            _scope = set()
            for _nname in net_names:
                for _nid, _net in pcb_data.nets.items():
                    if _net.name == _nname:
                        _scope.add(_nid)
            # The emissions here are DICTS for the GUI applier, but this
            # run's copper also lives in pcb_data as real objects (the
            # engine appends as it routes) -- so run the pipeline against
            # pcb_data with an empty write-list: every removal comes back
            # as an input strip, and additions come back as cleanup result
            # entries. Strips matching an emission dict drop that dict
            # (the applier adds AFTER it deletes, so a strip of not-yet-
            # added copper would no-op and the deleted copper would ship);
            # the rest go to the GUI strip channel.
            _res_wrap = []
            _out = run_post_route_cleanup(
                _res_wrap, pcb_data, _scope,
                SimpleNamespace(clearance=clearance, grid_step=grid_step),
                label='Plane ', phantom=False, via_nudge=False, neck=False,
                microshift_max_shift=grid_step)
            for _r in _res_wrap:
                for _s in (_r.get('new_segments') or []):
                    all_new_segments.append(
                        {'start': (_s.start_x, _s.start_y),
                         'end': (_s.end_x, _s.end_y),
                         'width': _s.width, 'layer': _s.layer,
                         'net_id': _s.net_id})
                for _v in (_r.get('new_vias') or []):
                    all_new_vias.append(
                        {'x': _v.x, 'y': _v.y, 'size': _v.size,
                         'drill': _v.drill, 'layers': _v.layers,
                         'net_id': _v.net_id})

            def _skey(_s):
                return (round(_s.start_x, 3), round(_s.start_y, 3),
                        round(_s.end_x, 3), round(_s.end_y, 3), _s.net_id)

            def _dkey(_d):
                return (round(_d['start'][0], 3), round(_d['start'][1], 3),
                        round(_d['end'][0], 3), round(_d['end'][1], 3),
                        _d['net_id'])
            _stripped = {}
            for _s in _out.input_strip_segments:
                _stripped[_skey(_s)] = _s
                _stripped[(_skey(_s)[2], _skey(_s)[3], _skey(_s)[0],
                           _skey(_s)[1], _skey(_s)[4])] = _s
            _kept_dicts = []
            for _d in all_new_segments:
                _hit = _stripped.pop(_dkey(_d), None)
                if _hit is None:
                    _kept_dicts.append(_d)
            all_new_segments[:] = _kept_dicts
            _strip_segments = list(
                {id(_s): _s for _s in _stripped.values()}.values())
            _strip_segments += list(getattr(_out, 'input_strip_vias', []) or [])
        except Exception as _e:
            print(f"{RED}  in-memory plane cleanup failed: {_e}{RESET}")
        # The GUI deletes every returned net's old board copper before adding
        # all_new_*; partial nets' kept pieces ride the emissions, so include
        # them in the deletion set (strip-and-replace parity).
        if progress_callback:
            progress_callback(1, 1, "Plane repair complete")
        return (total_routes, total_regions, all_new_vias, all_new_segments,
                ripped_net_ids + partial_ids, _strip_segments)

    if dry_run:
        print("\nDry run - no output file written")
    elif total_routes > 0 or total_pads_repaired > 0 or ripped_net_ids or partial_ids:
        print(f"\nWriting output to {output_file}...")
        # Strip the ripped signal nets' copper from the output - they are left
        # unrouted for a subsequent route.py pass to reconnect (#141 reverted).
        # Partially-restored nets are stripped too; their kept pieces are in
        # all_new_segments/all_new_vias (replacement).
        _write_output(input_file, output_file, all_new_segments, all_new_vias, all_debug_lines,
                      net_id_to_name=kv10_names,
                      exclude_net_ids=ripped_net_ids + partial_ids)
        print(f"Output written to {output_file}")
        print("Note: Open in KiCad and press 'B' to refill zones")
    else:
        print("\nNo routes added - copying input to output unchanged")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)

    # CLI: the ripped nets were stripped from the output and left unrouted; report
    # them so the caller reconnects them with a route.py pass (#141 reverted).
    if ripped_net_ids and not dry_run:
        _report_unrouted_ripped_nets(pcb_data, ripped_net_ids)

    # #347 (core1106 CLK1P): a net ripped or partially dropped for a pad
    # repair must not depend on a LATER chain step existing to reconnect it.
    # When this repair is the chain's FINAL command, the "reconnect them with
    # route.py" note used to be the end of the story and the net shipped OPEN
    # -- route.py connects the core1106 casualty in <1s when actually asked.
    # Self-run the standard route.py reconnect on the written output, scoped
    # to this run's own casualties. The routing is FRESH against the live
    # board (batch_route's own obstacle map + rip-up); the #141-revert hazard
    # (restoring stale ripped copper on top of newer routes) does not apply.
    # GUI (return_results) keeps its documented contract: ripped ids are
    # returned and the user reconnects via the routing tab.
    global LAST_RIPPED_RECONNECT
    LAST_RIPPED_RECONNECT = None
    _casualties = list(dict.fromkeys(ripped_net_ids + partial_ids))
    if _casualties and not dry_run and not return_results:
        _cnames = [pcb_data.nets[n].name for n in _casualties if n in pcb_data.nets]
        if _cnames:
            print(f"\nReconnecting {len(_cnames)} net(s) this run ripped for pad "
                  f"repairs: {', '.join(_cnames)}")
            if progress_callback:
                progress_callback(0, 0, f"Reconnecting {len(_cnames)} ripped net(s)...")
            try:
                from route import batch_route
                # #338: this self-invocation routes from OUTPUT_FILE, whose
                # sibling .kicad_pro does not exist yet, so batch_route's own
                # edge resolution reads nothing and the reconnect stamps its
                # edge band at the track-clearance fallback (openstint /A-
                # shipped 0.4mm from a 0.5mm rule). Resolve from the ORIGINAL
                # input's project here. Do NOT forward this function's
                # board_edge_clearance -- that is the plane-zone inset, not an
                # enforced routing floor.
                try:
                    from fix_kicad_drc_settings import read_project_edge_clearance
                    _edge = read_project_edge_clearance(input_file)
                except Exception:
                    _edge = 0.0
                _ok, _fail, _t = batch_route(
                    output_file, output_file, _cnames,
                    layers=routing_layers,
                    track_width=track_width, clearance=clearance,
                    via_size=via_size, via_drill=via_drill,
                    grid_step=grid_step, max_iterations=max_iterations,
                    power_nets=power_nets, power_nets_widths=power_nets_widths,
                    board_edge_clearance=_edge,
                    disable_bga_zones=([] if no_bga_zone else None),
                    # #434: OUTPUT_FILE's sibling .kicad_pro may not exist yet
                    # (same hazard as the #338 edge resolution above), so
                    # batch_route's own auto-read would find no netclasses and
                    # the reconnect would route class-blind into fat-class
                    # bands (cparti BTN4 vs the SMA 0.35 band). Forward the
                    # map resolved from the ORIGINAL input's project.
                    net_clearances=net_clearances)
                LAST_RIPPED_RECONNECT = {'nets': _cnames,
                                         'successful': _ok, 'failed': _fail}
                if _fail:
                    print(f"{RED}  {_fail} ripped net(s) could NOT be reconnected "
                          f"-- the board ships with them open{RESET}")
            except Exception as _e:
                print(f"{RED}  ripped-net reconnect pass failed: {_e}{RESET}")

    if progress_callback:
        progress_callback(1, 1, "Plane repair complete")
    return (total_routes, total_regions)


def _write_output(input_file: str, output_file: str, segments: List[Dict], vias: List[Dict] = None,
                  debug_lines: List[str] = None, net_id_to_name: Dict = None,
                  exclude_net_ids: List[int] = None):
    """Write the output PCB file with new segments, vias, and optional debug lines.

    exclude_net_ids: nets whose existing copper is stripped from the output (used
    for nets ripped to clear a blocked pad repair, which are re-routed separately).
    """
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    if exclude_net_ids:
        from plane_io import filter_nets_from_content
        names = ([net_id_to_name[n] for n in exclude_net_ids if net_id_to_name and n in net_id_to_name]
                 or None)
        content = filter_nets_from_content(content, exclude_net_ids, names)

    # Generate segment S-expressions
    segment_sexprs = []
    for seg in segments:
        seg_net_name = net_id_to_name.get(seg['net_id']) if net_id_to_name else None
        sexpr = generate_segment_sexpr(
            start=seg['start'],
            end=seg['end'],
            width=seg['width'],
            layer=seg['layer'],
            net_id=seg['net_id'],
            net_name=seg_net_name
        )
        segment_sexprs.append(sexpr)

    # Generate via S-expressions
    via_sexprs = []
    if vias:
        for via in vias:
            via_net_name = net_id_to_name.get(via['net_id']) if net_id_to_name else None
            sexpr = generate_via_sexpr(
                x=via['x'],
                y=via['y'],
                size=via['size'],
                drill=via['drill'],
                layers=['F.Cu', 'B.Cu'],  # Through-hole vias
                net_id=via['net_id'],
                net_name=via_net_name
            )
            via_sexprs.append(sexpr)

    routing_text = '\n'.join(segment_sexprs + via_sexprs)

    # Add debug lines if provided
    if debug_lines:
        routing_text += '\n' + '\n'.join(debug_lines)

    # Insert before final closing paren
    last_paren = content.rfind(')')
    new_content = content[:last_paren] + '\n' + routing_text + '\n' + content[last_paren:]

    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)


def main():
    from redo_record import record_invocation
    record_invocation()  # stress-test redo manifest (#132); no-op unless REDO_MANIFEST set
    parser = argparse.ArgumentParser(
        description="Route between disconnected regions in power plane zones",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Auto-detect all zones in PCB:
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb

    # Only process specific layers (all nets on those layers):
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb \\
        --plane-layers B.Cu In1.Cu

    # Only process specific nets (on all layers they have zones):
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND +3.3V

    # Specific net/layer pairs (counts must match):
    python route_disconnected_planes.py input.kicad_pcb output.kicad_pcb \\
        --nets GND +3.3V --plane-layers B.Cu In1.Cu \\
        --max-track-width 1.0
"""
    )

    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output KiCad PCB file (default: input_routed.kicad_pcb)")
    # #381 D9: accept --output FILE like route.py / route_diff.py.
    parser.add_argument("--output", metavar="FILE",
                        help="Output KiCad PCB file (flag alternative to the positional output)")
    parser.add_argument("--overwrite", "-O", action="store_true",
                        help="Overwrite input file instead of creating _routed copy")

    # Net and layer specification (now optional)
    parser.add_argument("--nets", "-n", nargs="+",
                        help="Net name(s) to process. If omitted, all nets with zones are processed.")
    parser.add_argument("--plane-layers", "-p", nargs="+",
                        help="Plane layer(s) to process. If omitted, all layers with zones are processed.")
    parser.add_argument("--layers", "-l", nargs="+",
                        help="Layer(s) available for routing (e.g., F.Cu B.Cu). If omitted, all copper layers are used.")

    # Track width options
    parser.add_argument("--max-track-width", type=float, default=defaults.REPAIR_MAX_TRACK_WIDTH,
                        help="Maximum track width for connections in mm (default: 2.0)")
    parser.add_argument("--min-track-width", type=float, default=defaults.REPAIR_MIN_TRACK_WIDTH,
                        help="Minimum track width for connections in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=defaults.TRACK_WIDTH,
                        help="Default track width for routing config in mm (default: 0.3)")

    # Clearance options
    parser.add_argument("--clearance", type=float, default=defaults.CLEARANCE,
                        help="Trace-to-trace clearance in mm (default: 0.25)")
    parser.add_argument("--zone-clearance", type=float, default=defaults.PLANE_ZONE_CLEARANCE,
                        help="Zone fill clearance around obstacles in mm (default: 0.2)")
    # #381 D9: accept route_planes.py's --plane-track-via-clearance spelling too
    # (same constant; dest stays track_via_clearance).
    parser.add_argument("--track-via-clearance", "--plane-track-via-clearance",
                        type=float, default=defaults.PLANE_TRACK_VIA_CLEARANCE,
                        help="Clearance from tracks to other nets' vias in mm (default: 0.8)")
    parser.add_argument("--board-edge-clearance", type=float, default=defaults.PLANE_EDGE_CLEARANCE,
                        help="Clearance from board edge in mm (default: 0.5)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=defaults.HOLE_TO_HOLE_CLEARANCE,
                        help=f"Minimum clearance between drill holes in mm (default: {defaults.HOLE_TO_HOLE_CLEARANCE}, the fab floor)")

    # Via options (for config)
    parser.add_argument("--via-size", type=float, default=defaults.VIA_SIZE,
                        help="Via outer diameter in mm (default: 0.5)")
    parser.add_argument("--via-drill", type=float, default=defaults.VIA_DRILL,
                        help="Via drill diameter in mm (default: 0.3)")

    # Grid step
    parser.add_argument("--grid-step", type=float, default=defaults.GRID_STEP,
                        help="Routing grid step in mm (default: 0.1)")
    parser.add_argument("--analysis-grid-step", type=float, default=defaults.REPAIR_ANALYSIS_GRID_STEP,
                        help="Grid step for connectivity analysis in mm (coarser = faster, default: 0.5)")

    # Routing options
    parser.add_argument("--no-kicad-recheck", action="store_true",
                        help="Skip the kicad-cli-verified reconnect pass on the output "
                             "(runs by default when kicad-cli is installed)")
    parser.add_argument("--max-iterations", type=int, default=defaults.MAX_ITERATIONS,
                        help="Maximum A* iterations per route attempt (default: 200000)")

    # Pad-level repair (issue #99)
    parser.add_argument("--repair-pads", dest="repair_pads", action="store_true", default=True,
                        help="Repair pad-level plane connection failures: retry a stitching "
                             "via + trace for plane-net pads with no connection to the plane, "
                             "escalating to fine parameters for fine-pitch pads (default: on)")
    parser.add_argument("--no-repair-pads", dest="repair_pads", action="store_false",
                        help="Disable the pad-level repair pass (only reconnect zone islands)")
    parser.add_argument("--max-search-radius", type=float, default=defaults.PLANE_MAX_SEARCH_RADIUS,
                        help=f"Max radius to search for a via position during pad repair in mm "
                             f"(default: {defaults.PLANE_MAX_SEARCH_RADIUS})")

    # Rip-blocker repair (mirror of route_planes): connect a plane-net pad that
    # cannot get its own via by tracing to an adjacent same-net pad, ripping the
    # signal net(s) blocking that trace, then re-routing them with the original
    # signal parameters - which must therefore be passed through.
    parser.add_argument("--rip-blocker-nets", action="store_true",
                        help="When a plane-net pad cannot be connected, trace to a nearby same-net "
                             "pad, ripping the signal net(s) blocking it, then re-route the ripped nets.")
    parser.add_argument("--max-rip-nets", type=int, default=defaults.PLANE_MAX_RIP_NETS,
                        help="Maximum number of blocker nets to rip per pad (default: 3)")
    parser.add_argument("--reroute-ripped-nets", action="store_true",
                        help="DEPRECATED / no-op (issue #141 reverted): ripped nets are always "
                             "left unrouted now -- run route.py afterward to reconnect them "
                             "(it handles rip-up/restore safely). Accepted for compatibility.")
    parser.add_argument("--power-nets", nargs="+", default=None,
                        help="Power net names that need wider tracks when re-routing ripped nets.")
    parser.add_argument("--power-nets-widths", nargs="+", type=float, default=None,
                        help="Track width (mm) per --power-nets entry, used when re-routing ripped nets.")
    # #381 D9: accept the plural --no-bga-zones spelling too (route.py uses it).
    parser.add_argument("--no-bga-zone", "--no-bga-zones", action="store_true",
                        help="Disable BGA auto-exclusion zones when re-routing ripped nets "
                             "(match the original signal run's --no-bga-zone).")

    # Debug options
    parser.add_argument("--dry-run", action="store_true",
                        help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed debug messages")
    parser.add_argument("--debug-lines", action="store_true",
                        help="Add debug lines on User.4 layer showing route paths")
    from fix_kicad_drc_settings import add_drc_fix_args
    add_drc_fix_args(parser)

    from fab_tiers import (add_fab_tier_args, fab_tier_from_args, set_default_fab_tier,
                           enforce_fab_floors, count_copper_layers_in_file)
    add_fab_tier_args(parser)
    args = parser.parse_args()
    set_default_fab_tier(*fab_tier_from_args(args))
    _pinned_floors = enforce_fab_floors(
        count_copper_layers_in_file(args.input_file),
        track_width=getattr(args, 'track_width', None),
        clearance=getattr(args, 'clearance', None),
        via_size=getattr(args, 'via_size', None),
        via_drill=getattr(args, 'via_drill', None),
        hole_to_hole_clearance=getattr(args, 'hole_to_hole_clearance', None))
    # Below-floor params are pinned up to the fab floor (warned); apply the clamps.
    for _pname, _pfloor in _pinned_floors.items():
        setattr(args, _pname, _pfloor)

    # #381 D9: --output FILE overrides the positional (matches route.py/route_diff).
    if getattr(args, 'output', None) is not None:
        if args.output_file is not None and args.output_file != args.output:
            parser.error("both a positional output and --output were given and differ")
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

    # Auto-detect zones if nets/layers not fully specified
    if args.nets and args.plane_layers:
        # Both specified - must match in count
        if len(args.nets) != len(args.plane_layers):
            print(f"Error: When both --nets and --plane-layers are specified, counts must match")
            print(f"  Got {len(args.nets)} net(s) and {len(args.plane_layers)} layer(s)")
            sys.exit(1)
        net_names = args.nets
        plane_layers = args.plane_layers
    else:
        # Auto-detect from PCB zones
        print(f"Auto-detecting zones from {args.input_file}...")
        zone_pairs = auto_detect_zones(
            args.input_file,
            filter_nets=args.nets,
            filter_layers=args.plane_layers
        )

        if not zone_pairs:
            if args.nets or args.plane_layers:
                print("No zones found matching the specified filters")
            else:
                print("No zones found in PCB file")
            sys.exit(1)

        net_names = [pair[0] for pair in zone_pairs]
        plane_layers = [pair[1] for pair in zone_pairs]

        print(f"Found {len(zone_pairs)} zone(s) to process:")
        for net, layer in zone_pairs:
            print(f"  {net} on {layer}")

    _rdp_result = route_planes(
        input_file=args.input_file,
        output_file=args.output_file,
        net_names=net_names,
        plane_layers=plane_layers,
        track_width=args.track_width,
        clearance=args.clearance,
        zone_clearance=args.zone_clearance,
        grid_step=args.grid_step,
        analysis_grid_step=args.analysis_grid_step,
        max_track_width=args.max_track_width,
        min_track_width=args.min_track_width,
        track_via_clearance=args.track_via_clearance,
        hole_to_hole_clearance=args.hole_to_hole_clearance,
        board_edge_clearance=args.board_edge_clearance,
        via_size=args.via_size,
        via_drill=args.via_drill,
        max_iterations=args.max_iterations,
        verbose=args.verbose,
        dry_run=args.dry_run,
        debug_lines=args.debug_lines,
        routing_layers=args.layers,
        repair_pads=args.repair_pads,
        max_search_radius=args.max_search_radius,
        rip_blocker_nets=args.rip_blocker_nets,
        max_rip_nets=args.max_rip_nets,
        reroute_ripped_nets=args.reroute_ripped_nets,
        power_nets=args.power_nets,
        power_nets_widths=args.power_nets_widths,
        no_bga_zone=args.no_bga_zone
    )

    # Dead-end sweep + gap-snap on the repaired plane copper (issue #84), gated
    # against connectivity + pours so it never breaks a net.
    if not args.dry_run:
        from pcb_modification import clean_plane_copper
        _snapped, _removed = clean_plane_copper(args.output_file, net_names,
                                                args.clearance, args.grid_step)
        if _snapped or _removed:
            print(f"Plane cleanup: closed {_snapped} stub gap(s), trimmed {_removed} dead-end segment(s)")

    # KiCad-oracle recheck (#217): our fill model over-credits, so gaps
    # KiCad's REAL fill produces can survive every model-based pass (castor
    # +3.3VA bare island, lumenpnp U5 pocket). Ask kicad-cli for the exact
    # missing links on the processed nets and route precisely those.
    if not args.dry_run and not args.no_kicad_recheck and args.output_file:
        from kicad_oracle import oracle_reconnect
        from routing_config import GridRouteConfig
        # #338: the oracle pass runs on OUTPUT_FILE, whose sibling .kicad_pro
        # is written only below (fix_project_for_output) -- so oracle_reconnect's
        # own project read finds nothing mid-chain. Resolve the board edge rule
        # from the ORIGINAL input's project here (the plane-zone inset
        # args.board_edge_clearance is NOT an enforced routing floor; see the
        # ripped-net reconnect above).
        try:
            from fix_kicad_drc_settings import read_project_edge_clearance
            _oracle_edge = read_project_edge_clearance(args.input_file)
        except Exception:
            _oracle_edge = 0.0
        _ocfg = GridRouteConfig(
            clearance=args.clearance, track_width=args.track_width,
            via_size=args.via_size, via_drill=args.via_drill,
            grid_step=args.grid_step,
            board_edge_clearance=_oracle_edge)
        _orc = oracle_reconnect(args.output_file, net_names, _ocfg,
                                track_via_clearance=args.track_via_clearance,
                                hole_to_hole_clearance=args.hole_to_hole_clearance,
                                verbose=args.verbose)
        try:
            import json as _json
            print('JSON_ORACLE: ' + _json.dumps(
                {k: v for k, v in _orc.items()
                 if k not in ('new_segments', 'new_vias')}))
        except Exception:
            pass
        if not _orc.get('available'):
            print('NOTE: kicad-cli not found -- the oracle reconnect pass '
                  'did not run; output may differ from machines that have '
                  'KiCad installed (replay-determinism caveat).')

    # Make the output project's DRC design rules consistent with the floors we
    # just routed to (issue #160), mirroring route_planes.py, so a manual DRC in
    # KiCad flags only genuine problems instead of stock-default noise.
    if not args.no_fix_drc_settings and not args.dry_run \
            and args.output_file and os.path.isfile(args.output_file):
        try:
            import clearance_ledger
            eff_clearance = clearance_ledger.effective(args.clearance)
            if eff_clearance < args.clearance:
                print(f"  Min clearance used: {eff_clearance:.4g} mm "
                      f"(below nominal {args.clearance:.4g}; fine-pitch taps) - "
                      f"grading at this floor")
            from fix_kicad_drc_settings import (fix_project_for_output, drc_fix_kwargs,
                                                read_project_edge_clearance)
            # #338: record the PROJECT's edge rule, not the plane-zone inset
            # (see route_planes.py -- openstint 0.3-design/0.5-recorded).
            fix_project_for_output(
                args.output_file, input_pcb=args.input_file,
                clearance=eff_clearance, hole_to_hole=args.hole_to_hole_clearance,
                edge_clearance=read_project_edge_clearance(args.input_file),
                track_width=args.track_width,
                via_diameter=args.via_size, via_drill=args.via_drill,
                **drc_fix_kwargs(args))
        except Exception as e:
            print(f"  (skipped DRC-settings fix: {e})")

    # Machine-readable summary (mirrors route.py/route_diff.py) so an orchestrator
    # and the next pipeline step can read the clearance this step actually used.
    import json as _json, clearance_ledger as _cl
    _routes, _regions = (_rdp_result if isinstance(_rdp_result, tuple)
                         and len(_rdp_result) >= 2 else (0, 0))
    _summary = {
        "total_routes": _routes,
        "total_regions": _regions,
        "min_clearance_used": _cl.effective(args.clearance),
    }
    if LAST_RIPPED_RECONNECT is not None:
        _summary["ripped_reconnect"] = LAST_RIPPED_RECONNECT
    # #408: plane-tap reconnects also route into the edge band to reach in-band
    # pads; emit those NET NAMES for the grader. Use the PROJECT's copper-to-edge
    # rule, not the plane-zone inset (--board-edge-clearance). Key omitted if empty.
    if not args.dry_run and args.output_file and os.path.isfile(args.output_file):
        try:
            from obstacle_map import compute_intentional_edge_band_nets
            from fix_kicad_drc_settings import read_project_edge_clearance
            from kicad_parser import parse_kicad_pcb as _parse
            _edge_rule = read_project_edge_clearance(args.input_file) or args.clearance
            _eb = compute_intentional_edge_band_nets(_parse(args.output_file), _edge_rule)
            if _eb:
                _summary["intentional_edge_band_nets"] = _eb
        except Exception:
            pass
    print("JSON_SUMMARY: " + _json.dumps(_summary))


if __name__ == "__main__":
    from console_encoding import enable_utf8_console
    enable_utf8_console()  # cp1252-safe non-ASCII prints (issue #152)
    main()

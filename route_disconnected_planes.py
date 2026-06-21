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

import sys
import os
import argparse
from dataclasses import replace
from typing import List, Tuple, Dict, Optional, Set

# Run startup checks first
from startup_checks import run_all_checks
run_all_checks()

from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via, KICAD_10_MIN_VERSION
from kicad_writer import generate_segment_sexpr, generate_gr_line_sexpr, generate_via_sexpr
from routing_config import GridRouteConfig, GridCoord
from plane_io import extract_zones, ZoneInfo
from plane_region_connector import route_disconnected_regions, build_base_obstacles, add_route_to_obstacles
from plane_pad_tap import find_unconnected_plane_pads, tap_pad_with_escalation
from plane_blocker_detection import find_route_blocker_from_frontier, find_via_position_blocker
from terminal_colors import GREEN, RED, RESET
import routing_defaults as defaults
import re

# Radius (mm) within which a plane-net pad that cannot get its own via may be
# connected to a nearby same-net pad/via with a trace (the human's connector-pin
# -> shield-pad strategy). Only used by the --rip-blocker-nets repair.
DISTANT_PAD_TRACE_RADIUS = 4.0


def _rip_net_from_pcb(pcb_data: PCBData, rip_net_id: int):
    """Remove a net's segments and vias from pcb_data so a blocked repair can
    re-attempt through the freed space. Returns (segments, vias) removed."""
    rsegs = [s for s in pcb_data.segments if s.net_id == rip_net_id]
    rvias = [v for v in pcb_data.vias if v.net_id == rip_net_id]
    if rsegs:
        pcb_data.segments = [s for s in pcb_data.segments if s.net_id != rip_net_id]
    if rvias:
        pcb_data.vias = [v for v in pcb_data.vias if v.net_id != rip_net_id]
    return rsegs, rvias


def _tap_pad_with_ripup(pad, pad_layer, net_id, pcb_data, tap_config, blocker_config,
                        max_search_radius, via_size, via_drill, max_rip_nets,
                        protected_net_ids, first_failure, ripped_net_ids, verbose,
                        distant_trace_radius=0.0):
    """A plane-net pad too small to drop a via in needs a trace to the plane (or
    to an adjacent same-net pad); if signal nets block that trace, rip them (up
    to max_rip_nets), retry the tap. Identifies the blocker from the failed
    route's frontier (or the via site for a via-placement block), never ripping a
    protected (plane) net. Ripped net ids are appended to ripped_net_ids for
    re-routing. Returns a successful TapResult, or None if no rip-up unblocked it."""
    failure = first_failure
    for _ in range(max_rip_nets):
        if failure.blocked_cells:
            blocker = find_route_blocker_from_frontier(
                failure.blocked_cells, pcb_data, blocker_config, net_id, protected_net_ids)
        else:
            blocker = find_via_position_blocker(
                pad.global_x, pad.global_y, pcb_data, blocker_config, net_id, protected_net_ids)
        if blocker is None or blocker in protected_net_ids:
            return None
        bname = pcb_data.nets[blocker].name if blocker in pcb_data.nets else f"net_{blocker}"
        print(f"{RED}blocked by {bname} - ripping{RESET}...", end=" ", flush=True)
        _rip_net_from_pcb(pcb_data, blocker)
        if blocker not in ripped_net_ids:
            ripped_net_ids.append(blocker)
        result = tap_pad_with_escalation(
            pad, pad_layer, net_id, pcb_data, tap_config,
            max_search_radius=max_search_radius, via_size=via_size, via_drill=via_drill,
            verbose=verbose, fine_for_all=True, distant_trace_radius=distant_trace_radius)
        if result.success:
            return result
        failure = result
    return None


def _reroute_ripped_nets(input_file, output_file, pcb_data, ripped_net_ids, do_reroute,
                         routing_layers, plane_layers, plane_vias, net_id_to_name,
                         plane_segments,
                         track_width, clearance, via_size, via_drill, grid_step,
                         hole_to_hole_clearance, power_nets, power_nets_widths,
                         no_bga_zone, verbose):
    """Re-route nets ripped to clear blocked pad repairs. Mirrors route_planes'
    reroute pass: a net that cannot re-route is RESTORED to its original trace
    (issue #88), never left disconnected. Re-route must use the same signal
    parameters as the original run (track/clearance/via and power-net widths) or
    nets that only routed with relaxed settings get silently dropped."""
    import sys
    ripped_names = [pcb_data.nets[r].name for r in ripped_net_ids if r in pcb_data.nets]
    if not ripped_names:
        return
    if not do_reroute:
        print(f"\nNote: {len(ripped_names)} net(s) ripped to clear pad repairs were "
              f"excluded from output; re-route them with route.py (matching width): "
              f"{', '.join(ripped_names)}")
        return

    print(f"\n{'='*60}\nRe-routing {len(ripped_names)} ripped net(s)...\n{'='*60}")
    from route import batch_route
    all_copper = sorted(set(routing_layers) | set(plane_layers))
    disable_bga = [] if no_bga_zone else None
    old_limit = sys.getrecursionlimit()
    sys.setrecursionlimit(max(old_limit, 100000))
    try:
        routed, failed, t = batch_route(
            input_file=output_file, output_file=output_file, net_names=ripped_names,
            layers=all_copper, track_width=track_width, clearance=clearance,
            via_size=via_size, via_drill=via_drill, grid_step=grid_step,
            hole_to_hole_clearance=hole_to_hole_clearance, verbose=verbose,
            minimal_obstacle_cache=True, power_nets=power_nets,
            power_nets_widths=power_nets_widths, disable_bga_zones=disable_bga)
        print(f"Re-routing complete: {routed} routed, {failed} failed in {t:.2f}s")
        try:
            from check_connected import check_net_connectivity
            from plane_io import restore_failed_reroute_nets
            out = parse_kicad_pcb(output_file)
            sb: Dict[int, List] = {}; vb: Dict[int, List] = {}; zb: Dict[int, List] = {}
            for s in out.segments: sb.setdefault(s.net_id, []).append(s)
            for v in out.vias: vb.setdefault(v.net_id, []).append(v)
            for z in out.zones: zb.setdefault(z.net_id, []).append(z)
            still = [r for r in ripped_net_ids if not check_net_connectivity(
                r, sb.get(r, []), vb.get(r, []), out.pads_by_net.get(r, []),
                zb.get(r, [])).get('connected', False)]
            if still:
                restored, vrem, srem = restore_failed_reroute_nets(
                    input_file=input_file, output_file=output_file, broken_net_ids=still,
                    plane_vias=plane_vias, net_id_to_name=net_id_to_name,
                    via_size=via_size, clearance=clearance, plane_segments=plane_segments)
                if restored:
                    names = [pcb_data.nets[r].name if r in pcb_data.nets else f"net_{r}" for r in restored]
                    print(f"Restored {len(restored)} ripped net(s) that failed to re-route "
                          f"(removed {vrem} colliding via(s), {srem} colliding segment(s)): "
                          f"{', '.join(names)}")
                unrest = [r for r in still if r not in restored]
                if unrest:
                    names = [pcb_data.nets[r].name if r in pcb_data.nets else f"net_{r}" for r in unrest]
                    print(f"WARNING: {len(unrest)} ripped net(s) remain disconnected: {', '.join(names)}")
        except Exception as e:
            print(f"  (post-reroute restore step skipped: {e})")
    finally:
        sys.setrecursionlimit(old_limit)


def _reroute_ripped_nets_in_memory(input_file, pcb_data, ripped_net_ids, repair_segments,
                                   repair_vias, net_id_to_name, track_width, clearance,
                                   via_size, via_drill, grid_step, hole_to_hole_clearance,
                                   power_nets, power_nets_widths, no_bga_zone,
                                   routing_layers, plane_layers, verbose):
    """Re-route the ripped nets and RETURN their new (segments, vias) so the caller
    can apply them via pcbnew (GUI return_results mode), instead of writing a file.

    Writes a temp board = input with the ripped nets excluded + the repair copper
    added, batch-routes the ripped nets on it, then extracts their copper - all of
    which is new, since they were excluded from the temp board's input."""
    import sys, tempfile, os
    ripped_names = [pcb_data.nets[r].name for r in ripped_net_ids if r in pcb_data.nets]
    if not ripped_names:
        return [], []
    fd1, tin = tempfile.mkstemp(suffix=".kicad_pcb", prefix="rdp_rr_in_"); os.close(fd1)
    fd2, tout = tempfile.mkstemp(suffix=".kicad_pcb", prefix="rdp_rr_out_"); os.close(fd2)
    try:
        _write_output(input_file, tin, repair_segments, repair_vias,
                      net_id_to_name=net_id_to_name, exclude_net_ids=ripped_net_ids)
        from route import batch_route
        all_copper = sorted(set(routing_layers) | set(plane_layers))
        old = sys.getrecursionlimit(); sys.setrecursionlimit(max(old, 100000))
        try:
            batch_route(
                input_file=tin, output_file=tout, net_names=ripped_names,
                layers=all_copper, track_width=track_width, clearance=clearance,
                via_size=via_size, via_drill=via_drill, grid_step=grid_step,
                hole_to_hole_clearance=hole_to_hole_clearance, verbose=verbose,
                minimal_obstacle_cache=True, power_nets=power_nets,
                power_nets_widths=power_nets_widths,
                disable_bga_zones=([] if no_bga_zone else None))
        finally:
            sys.setrecursionlimit(old)
        out = parse_kicad_pcb(tout)
        rid = set(ripped_net_ids)
        from check_connected import check_net_connectivity
        # Re-routed copper (from the temp output) grouped per ripped net.
        o_seg: Dict[int, List] = {}; o_via: Dict[int, List] = {}; o_zone: Dict[int, List] = {}
        for s in out.segments:
            if s.net_id in rid: o_seg.setdefault(s.net_id, []).append(s)
        for v in out.vias:
            if v.net_id in rid: o_via.setdefault(v.net_id, []).append(v)
        for z in out.zones:
            if z.net_id in rid: o_zone.setdefault(z.net_id, []).append(z)
        # Original copper from the input, to restore a net that fails to re-route.
        i_seg: Dict[int, List] = {}; i_via: Dict[int, List] = {}
        for s in pcb_data.segments:
            if s.net_id in rid: i_seg.setdefault(s.net_id, []).append(s)
        for v in pcb_data.vias:
            if v.net_id in rid: i_via.setdefault(v.net_id, []).append(v)

        segs: List[Dict] = []; vias: List[Dict] = []; restored: List[int] = []
        for r in ripped_net_ids:
            res = check_net_connectivity(r, o_seg.get(r, []), o_via.get(r, []),
                                         out.pads_by_net.get(r, []), o_zone.get(r, []))
            if res.get('connected', False):
                src_s, src_v = o_seg.get(r, []), o_via.get(r, [])
            else:
                # Issue #88 parity for the GUI/in-memory path: a net that fails to
                # re-route must NOT be dropped. The caller deletes the net's old
                # tracks before applying these, so fall back to its ORIGINAL copper
                # (restoring the pre-rip trace) instead of returning nothing.
                src_s, src_v = i_seg.get(r, []), i_via.get(r, [])
                if src_s or src_v:
                    restored.append(r)
            for s in src_s:
                segs.append({'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
                             'width': s.width, 'layer': s.layer, 'net_id': s.net_id})
            for v in src_v:
                vias.append({'x': v.x, 'y': v.y, 'size': v.size, 'drill': v.drill,
                             'layers': ['F.Cu', 'B.Cu'], 'net_id': v.net_id})
        msg = (f"Re-routed {len(ripped_names)} ripped net(s) in-memory: "
               f"{len(segs)} segment(s), {len(vias)} via(s)")
        if restored:
            names = [pcb_data.nets[r].name if r in pcb_data.nets else f"net_{r}" for r in restored]
            msg += f"; restored {len(restored)} net(s) that failed to re-route: {', '.join(names)}"
        print(msg)
        return segs, vias
    finally:
        for f in (tin, tout):
            try:
                os.remove(f)
            except OSError:
                pass


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

    Returns:
        Tuple of (total_routes_added, total_regions_connected)
    """
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

    # Auto-detect routing layers if not specified
    if routing_layers is None:
        routing_layers = pcb_data.board_info.copper_layers
        if not routing_layers:
            routing_layers = ['F.Cu', 'B.Cu']  # Fallback
    print(f"Routing layers: {', '.join(routing_layers)}")

    all_new_segments: List[Dict] = []
    all_new_vias: List[Dict] = []
    all_debug_lines: List[str] = []
    total_routes = 0
    total_regions = 0
    total_vias = 0
    total_pads_unconnected = 0
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

    # Plane nets are never ripped to clear a blocker (--rip-blocker-nets); only
    # signal nets are, and they are re-routed afterward.
    plane_net_ids = set(unique_nets.keys())
    ripped_net_ids: List[int] = []
    distant_radius = min(max_search_radius, DISTANT_PAD_TRACE_RADIUS) if rip_blocker_nets else 0.0

    for net_id, (net_name, net_zone_layers) in unique_nets.items():
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
                for pad, pad_layer in unconnected:
                    print(f"    Pad {pad.component_ref}.{pad.pad_number} ({pad_layer})...", end=" ", flush=True)
                    result = tap_pad_with_escalation(
                        pad, pad_layer, net_id, pcb_data, tap_config,
                        max_search_radius=max_search_radius,
                        via_size=via_size,
                        via_drill=via_drill,
                        verbose=verbose,
                        fine_for_all=True,  # last-resort repair: escalate every failed pad
                        distant_trace_radius=distant_radius
                    )
                    if not result.success and rip_blocker_nets:
                        # Rip the signal net(s) blocking this pad's trace and retry
                        # (the ripped nets are re-routed after the repair pass).
                        rr = _tap_pad_with_ripup(
                            pad, pad_layer, net_id, pcb_data, tap_config, config,
                            max_search_radius, via_size, via_drill, max_rip_nets,
                            plane_net_ids, result, ripped_net_ids, verbose,
                            distant_trace_radius=distant_radius)
                        if rr is not None:
                            result = rr
                    if result.success:
                        total_pads_repaired += 1
                        params_note = " [fine params]" if result.params_label == 'fine' else ""
                        if result.via is not None:
                            all_new_vias.append(result.via)
                            total_vias += 1
                            pcb_data.vias.append(Via(
                                x=result.via['x'], y=result.via['y'],
                                size=result.via['size'], drill=result.via['drill'],
                                layers=['F.Cu', 'B.Cu'], net_id=net_id
                            ))
                            where = f"placed via at ({result.via['x']:.2f}, {result.via['y']:.2f})"
                        else:
                            where = (f"reused via at ({result.reused_via_pos[0]:.2f}, "
                                     f"{result.reused_via_pos[1]:.2f})")
                        for s in result.segments:
                            all_new_segments.append(s)
                            pcb_data.segments.append(Segment(
                                start_x=s['start'][0], start_y=s['start'][1],
                                end_x=s['end'][0], end_y=s['end'][1],
                                width=s['width'], layer=s['layer'], net_id=s['net_id']
                            ))
                        print(f"{GREEN}{where}, {len(result.segments)} trace segment(s){params_note}{RESET}")
                    else:
                        failed_repair_pads.append(f"{pad.component_ref}.{pad.pad_number} ({net_name})")
                        print(f"{RED}FAILED{RESET}")

        # Build obstacle map for this net
        print(f"  Building obstacle map...", end=" ", flush=True)
        base_obstacles, layer_map = build_base_obstacles(
            exclude_net_ids={net_id},
            routing_layers=routing_layers,
            pcb_data=pcb_data,
            config=config,
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
            zone_clearances=zone_clearances
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

    kv10_names = pcb_data.net_id_to_name if pcb_data.kicad_version >= KICAD_10_MIN_VERSION else None

    # Issue #141: --reroute-ripped-nets should also pick up nets the earlier
    # route_planes pass ripped (and left unrouted), not just nets we ripped here.
    # Use the SAME selection route.py/batch_route uses to decide what to route -
    # filter_already_routed (2+ pads, not fully connected; zone-aware) - so the set
    # of nets we re-route matches what route.py would route. Plane nets (connected
    # via their zone) and nets we already ripped this pass are excluded.
    if reroute_ripped_nets:
        import io, contextlib
        from routing_common import filter_already_routed
        already = plane_net_ids | set(ripped_net_ids)
        candidates = [(net.name, nid) for nid, net in pcb_data.nets.items()
                      if nid > 0 and nid not in already]
        # filter_already_routed prints a line per already-routed net; over a whole
        # board that is dozens of lines of noise, so swallow its stdout here.
        with contextlib.redirect_stdout(io.StringIO()):
            nets_to_route, _ = filter_already_routed(pcb_data, candidates, config)
        if nets_to_route:
            names = [n for n, _ in nets_to_route]
            print(f"\nAlso re-routing {len(nets_to_route)} net(s) left unrouted by the "
                  f"plane pass (issue #141): {', '.join(names)}")
            ripped_net_ids.extend(nid for _, nid in nets_to_route)

    # GUI (return_results): apply via pcbnew, so re-route the ripped nets IN MEMORY
    # and hand their copper + ids back (the caller deletes the ripped tracks and
    # adds the new ones). No file is written here.
    if return_results:
        if ripped_net_ids and reroute_ripped_nets:
            rsegs, rvias = _reroute_ripped_nets_in_memory(
                input_file, pcb_data, ripped_net_ids, all_new_segments, all_new_vias,
                kv10_names, track_width, clearance, via_size, via_drill, grid_step,
                hole_to_hole_clearance, power_nets, power_nets_widths, no_bga_zone,
                routing_layers, plane_layers, verbose)
            all_new_segments = all_new_segments + rsegs
            all_new_vias = all_new_vias + rvias
        return (total_routes, total_regions, all_new_vias, all_new_segments, ripped_net_ids)

    if dry_run:
        print("\nDry run - no output file written")
    elif total_routes > 0 or total_pads_repaired > 0 or ripped_net_ids:
        print(f"\nWriting output to {output_file}...")
        # Strip the ripped signal nets' copper from the output - it is re-routed
        # below (or restored if that fails).
        _write_output(input_file, output_file, all_new_segments, all_new_vias, all_debug_lines,
                      net_id_to_name=kv10_names, exclude_net_ids=ripped_net_ids)
        print(f"Output written to {output_file}")
        print("Note: Open in KiCad and press 'B' to refill zones")
    else:
        print("\nNo routes added - copying input to output unchanged")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)

    # CLI: re-route the ripped nets into the written file (restoring any that fail).
    if ripped_net_ids and not dry_run:
        _reroute_ripped_nets(
            input_file, output_file, pcb_data, ripped_net_ids, reroute_ripped_nets,
            routing_layers, plane_layers, all_new_vias, kv10_names, all_new_segments,
            track_width=track_width, clearance=clearance, via_size=via_size,
            via_drill=via_drill, grid_step=grid_step,
            hole_to_hole_clearance=hole_to_hole_clearance,
            power_nets=power_nets, power_nets_widths=power_nets_widths,
            no_bga_zone=no_bga_zone, verbose=verbose)

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
    parser.add_argument("--max-track-width", type=float, default=2.0,
                        help="Maximum track width for connections in mm (default: 2.0)")
    parser.add_argument("--min-track-width", type=float, default=0.2,
                        help="Minimum track width for connections in mm (default: 0.2)")
    parser.add_argument("--track-width", type=float, default=0.3,
                        help="Default track width for routing config in mm (default: 0.3)")

    # Clearance options
    parser.add_argument("--clearance", type=float, default=0.25,
                        help="Trace-to-trace clearance in mm (default: 0.25)")
    parser.add_argument("--zone-clearance", type=float, default=0.2,
                        help="Zone fill clearance around obstacles in mm (default: 0.2)")
    parser.add_argument("--track-via-clearance", type=float, default=0.8,
                        help="Clearance from tracks to other nets' vias in mm (default: 0.8)")
    parser.add_argument("--board-edge-clearance", type=float, default=0.5,
                        help="Clearance from board edge in mm (default: 0.5)")
    parser.add_argument("--hole-to-hole-clearance", type=float, default=0.3,
                        help="Minimum clearance between drill holes in mm (default: 0.3)")

    # Via options (for config)
    parser.add_argument("--via-size", type=float, default=0.5,
                        help="Via outer diameter in mm (default: 0.5)")
    parser.add_argument("--via-drill", type=float, default=0.3,
                        help="Via drill diameter in mm (default: 0.3)")

    # Grid step
    parser.add_argument("--grid-step", type=float, default=0.1,
                        help="Routing grid step in mm (default: 0.1)")
    parser.add_argument("--analysis-grid-step", type=float, default=0.5,
                        help="Grid step for connectivity analysis in mm (coarser = faster, default: 0.5)")

    # Routing options
    parser.add_argument("--max-iterations", type=int, default=200000,
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
    parser.add_argument("--max-rip-nets", type=int, default=3,
                        help="Maximum number of blocker nets to rip per pad (default: 3)")
    parser.add_argument("--reroute-ripped-nets", action="store_true",
                        help="Automatically re-route the ripped nets after the repair pass "
                             "(otherwise they are excluded from output and listed for a later pass).")
    parser.add_argument("--power-nets", nargs="+", default=None,
                        help="Power net names that need wider tracks when re-routing ripped nets.")
    parser.add_argument("--power-nets-widths", nargs="+", type=float, default=None,
                        help="Track width (mm) per --power-nets entry, used when re-routing ripped nets.")
    parser.add_argument("--no-bga-zone", action="store_true",
                        help="Disable BGA auto-exclusion zones when re-routing ripped nets "
                             "(match the original signal run's --no-bga-zone).")

    # Debug options
    parser.add_argument("--dry-run", action="store_true",
                        help="Analyze without writing output")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print detailed debug messages")
    parser.add_argument("--debug-lines", action="store_true",
                        help="Add debug lines on User.4 layer showing route paths")

    args = parser.parse_args()

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

    route_planes(
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
        _snapped, _removed = clean_plane_copper(args.output_file, net_names, args.clearance)
        if _snapped or _removed:
            print(f"Plane cleanup: closed {_snapped} stub gap(s), trimmed {_removed} dead-end segment(s)")


if __name__ == "__main__":
    main()

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
from typing import List, Tuple, Dict, Optional

# Run startup checks first
from startup_checks import run_all_checks
run_all_checks()

from kicad_parser import parse_kicad_pcb, PCBData, Segment, Via
from kicad_writer import generate_segment_sexpr, generate_gr_line_sexpr, generate_via_sexpr
from routing_config import GridRouteConfig, GridCoord
from plane_io import extract_zones, ZoneInfo
from plane_region_connector import route_disconnected_regions, build_base_obstacles, add_route_to_obstacles
import re


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
    track_width: float = 0.2,
    clearance: float = 0.2,
    zone_clearance: float = 0.2,
    grid_step: float = 0.1,
    analysis_grid_step: float = 0.5,
    max_track_width: float = 2.0,
    min_track_width: float = 0.2,
    track_via_clearance: float = 0.8,
    hole_to_hole_clearance: float = 0.3,
    board_edge_clearance: float = 0.5,
    via_size: float = 0.5,
    via_drill: float = 0.4,
    max_iterations: int = 200000,
    verbose: bool = False,
    dry_run: bool = False,
    debug_lines: bool = False,
    routing_layers: Optional[List[str]] = None
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

    Returns:
        Tuple of (total_routes_added, total_regions_connected)
    """
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
        grid_step=grid_step
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
    if debug_lines and all_debug_lines:
        print(f"  Debug lines on User.4: {len(all_debug_lines)}")

    if dry_run:
        print("\nDry run - no output file written")
    elif total_routes > 0:
        print(f"\nWriting output to {output_file}...")
        _write_output(input_file, output_file, all_new_segments, all_new_vias, all_debug_lines)
        print(f"Output written to {output_file}")
        print("Note: Open in KiCad and press 'B' to refill zones")
    else:
        print("\nNo routes added - copying input to output unchanged")
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(content)

    return (total_routes, total_regions)


def _write_output(input_file: str, output_file: str, segments: List[Dict], vias: List[Dict] = None, debug_lines: List[str] = None):
    """Write the output PCB file with new segments, vias, and optional debug lines."""
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate segment S-expressions
    segment_sexprs = []
    for seg in segments:
        sexpr = generate_segment_sexpr(
            start=seg['start'],
            end=seg['end'],
            width=seg['width'],
            layer=seg['layer'],
            net_id=seg['net_id']
        )
        segment_sexprs.append(sexpr)

    # Generate via S-expressions
    via_sexprs = []
    if vias:
        for via in vias:
            sexpr = generate_via_sexpr(
                x=via['x'],
                y=via['y'],
                size=via['size'],
                drill=via['drill'],
                layers=['F.Cu', 'B.Cu'],  # Through-hole vias
                net_id=via['net_id']
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
        routing_layers=args.layers
    )


if __name__ == "__main__":
    main()

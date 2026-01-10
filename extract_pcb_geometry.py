#!/usr/bin/env python3
"""
Extract KiCad PCB geometry into a simple JSON format.

Usage:
    python extract_pcb_geometry.py input.kicad_pcb [output.json]
    python extract_pcb_geometry.py input.kicad_pcb --summary
    python extract_pcb_geometry.py input.kicad_pcb --nets "pattern"

Examples:
    python extract_pcb_geometry.py board.kicad_pcb geometry.json
    python extract_pcb_geometry.py board.kicad_pcb --summary
    python extract_pcb_geometry.py board.kicad_pcb --nets "*lvds*" --output lvds.json
"""

import sys
import json
import argparse
import fnmatch
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Any, Optional

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from kicad_parser import parse_kicad_pcb, PCBData, POSITION_DECIMALS


def extract_geometry(pcb: PCBData, net_patterns: Optional[List[str]] = None) -> Dict[str, Any]:
    """
    Extract all geometry from a PCBData object into a dictionary.

    Args:
        pcb: Parsed PCB data
        net_patterns: Optional list of glob patterns to filter nets (e.g., ["*lvds*", "*clk*"])

    Returns:
        Dictionary with all geometry data
    """

    def matches_pattern(net_name: str, patterns: List[str]) -> bool:
        """Check if net name matches any of the patterns."""
        if not patterns:
            return True
        return any(fnmatch.fnmatch(net_name, p) for p in patterns)

    # Build net ID to name mapping
    net_names = {net_id: net.name for net_id, net in pcb.nets.items()}

    # Filter net IDs if patterns provided
    if net_patterns:
        filtered_net_ids = {
            net_id for net_id, name in net_names.items()
            if matches_pattern(name, net_patterns)
        }
    else:
        filtered_net_ids = None  # Include all

    def should_include(net_id: int) -> bool:
        return filtered_net_ids is None or net_id in filtered_net_ids

    # Extract nets
    nets = {}
    for net_id, net in pcb.nets.items():
        if should_include(net_id):
            nets[net_id] = {
                "name": net.name,
                "id": net_id
            }

    # Extract segments (tracks)
    segments = []
    for seg in pcb.segments:
        if should_include(seg.net_id):
            segments.append({
                "net_id": seg.net_id,
                "net_name": net_names.get(seg.net_id, f"net_{seg.net_id}"),
                "start": {"x": seg.start_x, "y": seg.start_y},
                "end": {"x": seg.end_x, "y": seg.end_y},
                "layer": seg.layer,
                "width": seg.width,
                "uuid": seg.uuid
            })

    # Extract vias
    vias = []
    for via in pcb.vias:
        if should_include(via.net_id):
            vias.append({
                "net_id": via.net_id,
                "net_name": net_names.get(via.net_id, f"net_{via.net_id}"),
                "x": via.x,
                "y": via.y,
                "size": via.size,
                "drill": via.drill,
                "layers": via.layers,
                "uuid": via.uuid
            })

    # Extract pads (grouped by component)
    pads = []
    pads_by_component = defaultdict(list)
    for net_id, pad_list in pcb.pads_by_net.items():
        if should_include(net_id):
            for pad in pad_list:
                pad_data = {
                    "net_id": pad.net_id,
                    "net_name": pad.net_name,
                    "component": pad.component_ref,
                    "pad_number": pad.pad_number,
                    "x": pad.global_x,
                    "y": pad.global_y,
                    "size": {"x": pad.size_x, "y": pad.size_y},
                    "shape": pad.shape,
                    "layers": pad.layers,
                    "rotation": pad.rotation,
                    "pinfunction": pad.pinfunction,
                    "pintype": pad.pintype
                }
                pads.append(pad_data)
                pads_by_component[pad.component_ref].append(pad_data)

    # Extract footprints
    footprints = {}
    for ref, fp in pcb.footprints.items():
        footprints[ref] = {
            "reference": fp.reference,
            "footprint_name": fp.footprint_name,
            "x": fp.x,
            "y": fp.y,
            "rotation": fp.rotation,
            "layer": fp.layer,
            "pad_count": len(fp.pads) if fp.pads else 0
        }

    # Build stub analysis (segments with unconnected endpoints)
    segments_by_net = defaultdict(list)
    for seg in segments:
        segments_by_net[seg["net_id"]].append(seg)

    stubs = []
    for net_id, segs in segments_by_net.items():
        if len(segs) > 5:  # Skip fully routed nets
            continue

        # Find endpoints
        endpoints = defaultdict(int)
        for seg in segs:
            p1 = (round(seg["start"]["x"], POSITION_DECIMALS), round(seg["start"]["y"], POSITION_DECIMALS))
            p2 = (round(seg["end"]["x"], POSITION_DECIMALS), round(seg["end"]["y"], POSITION_DECIMALS))
            endpoints[p1] += 1
            endpoints[p2] += 1

        # Stub endpoints have count == 1
        for (x, y), count in endpoints.items():
            if count == 1:
                stubs.append({
                    "net_id": net_id,
                    "net_name": net_names.get(net_id, f"net_{net_id}"),
                    "x": x,
                    "y": y,
                    "segment_count": len(segs)
                })

    return {
        "source_file": None,  # Will be set by caller
        "nets": nets,
        "segments": segments,
        "vias": vias,
        "pads": pads,
        "pads_by_component": dict(pads_by_component),
        "footprints": footprints,
        "stubs": stubs,
        "summary": {
            "net_count": len(nets),
            "segment_count": len(segments),
            "via_count": len(vias),
            "pad_count": len(pads),
            "footprint_count": len(footprints),
            "stub_count": len(stubs)
        }
    }


def print_summary(data: Dict[str, Any]) -> None:
    """Print a human-readable summary of the geometry."""
    s = data["summary"]
    print(f"\n{'='*60}")
    print(f"PCB Geometry Summary: {data.get('source_file', 'unknown')}")
    print(f"{'='*60}")
    print(f"  Nets:       {s['net_count']:,}")
    print(f"  Segments:   {s['segment_count']:,}")
    print(f"  Vias:       {s['via_count']:,}")
    print(f"  Pads:       {s['pad_count']:,}")
    print(f"  Footprints: {s['footprint_count']:,}")
    print(f"  Stubs:      {s['stub_count']:,}")
    print()

    # Layer distribution
    layers = defaultdict(int)
    for seg in data["segments"]:
        layers[seg["layer"]] += 1
    if layers:
        print("Segments by layer:")
        for layer, count in sorted(layers.items()):
            print(f"  {layer}: {count:,}")
        print()

    # Top components by pad count
    if data["pads_by_component"]:
        print("Top components by pad count:")
        sorted_comps = sorted(
            data["pads_by_component"].items(),
            key=lambda x: len(x[1]),
            reverse=True
        )[:10]
        for ref, pads in sorted_comps:
            print(f"  {ref}: {len(pads)} pads")
        print()


def find_stubs_near_point(data: Dict[str, Any], x: float, y: float, radius: float) -> List[Dict]:
    """Find stub endpoints within radius of a point."""
    import math
    results = []
    for stub in data["stubs"]:
        dist = math.sqrt((stub["x"] - x)**2 + (stub["y"] - y)**2)
        if dist <= radius:
            results.append({**stub, "distance": dist})
    return sorted(results, key=lambda s: s["distance"])


def find_stubs_near_line(
    data: Dict[str, Any],
    x1: float, y1: float,
    x2: float, y2: float,
    max_distance: float
) -> List[Dict]:
    """Find stub endpoints within max_distance of a line segment."""
    import math

    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx*dx + dy*dy)
    if length < 0.001:
        return find_stubs_near_point(data, x1, y1, max_distance)

    dx /= length
    dy /= length

    results = []
    for stub in data["stubs"]:
        # Vector from start to stub
        vx = stub["x"] - x1
        vy = stub["y"] - y1

        # Distance along line
        along = vx * dx + vy * dy

        # Only consider points along the segment (with some margin)
        if -1 < along < length + 1:
            # Perpendicular distance
            perp_dist = abs(vx * dy - vy * dx)
            if perp_dist <= max_distance:
                results.append({
                    **stub,
                    "distance": perp_dist,
                    "along": along
                })

    return sorted(results, key=lambda s: s["distance"])


def main():
    parser = argparse.ArgumentParser(
        description="Extract KiCad PCB geometry to JSON",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s board.kicad_pcb                    # Output to board_geometry.json
  %(prog)s board.kicad_pcb output.json        # Output to specific file
  %(prog)s board.kicad_pcb --summary          # Print summary only
  %(prog)s board.kicad_pcb --nets "*lvds*"    # Filter to LVDS nets
  %(prog)s board.kicad_pcb --nets "*lvds*" "*clk*"  # Multiple patterns
        """
    )
    parser.add_argument("input_file", help="Input KiCad PCB file")
    parser.add_argument("output_file", nargs="?", help="Output JSON file (default: <input>_geometry.json)")
    parser.add_argument("--summary", action="store_true", help="Print summary and exit (no JSON output)")
    parser.add_argument("--nets", nargs="+", metavar="PATTERN",
                        help="Filter to nets matching glob patterns (e.g., '*lvds*')")
    parser.add_argument("--indent", type=int, default=2, help="JSON indent (default: 2, use 0 for compact)")
    parser.add_argument("-o", "--output", dest="output_file_alt", help="Alternative way to specify output file")

    args = parser.parse_args()

    input_path = Path(args.input_file)
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}", file=sys.stderr)
        sys.exit(1)

    # Determine output file
    output_file = args.output_file or args.output_file_alt
    if not output_file and not args.summary:
        output_file = input_path.stem + "_geometry.json"

    print(f"Loading {input_path}...")
    pcb = parse_kicad_pcb(str(input_path))

    print("Extracting geometry...")
    data = extract_geometry(pcb, args.nets)
    data["source_file"] = str(input_path)

    if args.nets:
        print(f"Filtered to {data['summary']['net_count']} nets matching: {args.nets}")

    print_summary(data)

    if not args.summary:
        indent = args.indent if args.indent > 0 else None
        with open(output_file, 'w') as f:
            json.dump(data, f, indent=indent)
        print(f"Wrote {output_file}")


# Convenience functions for interactive use
def load_geometry(json_file: str) -> Dict[str, Any]:
    """Load geometry from a JSON file."""
    with open(json_file) as f:
        return json.load(f)


def quick_extract(pcb_file: str, net_patterns: Optional[List[str]] = None) -> Dict[str, Any]:
    """Quick extraction without writing to file - useful for interactive use."""
    pcb = parse_kicad_pcb(pcb_file)
    data = extract_geometry(pcb, net_patterns)
    data["source_file"] = pcb_file
    return data


if __name__ == "__main__":
    main()

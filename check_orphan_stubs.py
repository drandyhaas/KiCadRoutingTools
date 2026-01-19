#!/usr/bin/env python3
"""
Check for orphan trace stubs in a KiCad PCB file.

An orphan stub is a trace endpoint that:
1. Has only one connected segment (degree-1 node)
2. Is NOT near a via
3. Is NOT near a through-hole pad

These represent traces that end without a proper connection point.

Usage:
    python check_orphan_stubs.py input.kicad_pcb [--net NET_NAME] [--layer LAYER]

Examples:
    python check_orphan_stubs.py board.kicad_pcb
    python check_orphan_stubs.py board.kicad_pcb --net "+3.3V" --layer F.Cu
    python check_orphan_stubs.py original.kicad_pcb modified.kicad_pcb --compare
"""

import argparse
import re
import sys
from collections import Counter
from typing import Set, Tuple, Dict, List, Optional

# Add current directory to path for imports
sys.path.insert(0, '.')


def load_pcb_data(filename: str):
    """Load PCB data using kicad_parser."""
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(filename)


def get_net_id(content: str, net_name: str) -> Optional[str]:
    """Find net ID for a given net name."""
    match = re.search(rf'\(net (\d+) "{re.escape(net_name)}"\)', content)
    return match.group(1) if match else None


def get_vias_for_net(content: str, net_id: str) -> Set[Tuple[float, float]]:
    """Get all via positions for a given net."""
    vias = set()
    pattern = r'\(via\s+\(at\s+([\d.]+)\s+([\d.]+)\)[^(]*(?:\([^)]+\)\s*)*\(net\s+' + net_id + r'\)'
    for m in re.finditer(pattern, content):
        vias.add((float(m.group(1)), float(m.group(2))))
    return vias


def get_segments_for_net_layer(content: str, net_id: str, layer: str) -> List[Dict]:
    """Get all segments for a given net and layer."""
    segments = []
    pattern = rf'\(segment\s+\(start\s+([\d.]+)\s+([\d.]+)\)\s+\(end\s+([\d.]+)\s+([\d.]+)\)\s+\(width[^)]+\)\s+\(layer\s+"{re.escape(layer)}"\)\s+\(net\s+{net_id}\)'
    for m in re.finditer(pattern, content):
        segments.append({
            'start': (float(m.group(1)), float(m.group(2))),
            'end': (float(m.group(3)), float(m.group(4)))
        })
    return segments


def get_through_hole_pads(pcb_data, net_name: str) -> Set[Tuple[float, float]]:
    """Get through-hole pad positions for a given net."""
    pads = set()
    for nid, net in pcb_data.nets.items():
        if net.name == net_name:
            for pad in net.pads:
                # Through-hole if has drill or is on all copper layers
                if pad.drill > 0 or '*.Cu' in pad.layers:
                    pads.add((pad.global_x, pad.global_y))
    return pads


def get_layer_pads(pcb_data, net_name: str, layer: str) -> Set[Tuple[float, float]]:
    """Get pad positions for a given net on a specific layer (including SMD pads)."""
    pads = set()
    for nid, net in pcb_data.nets.items():
        if net.name == net_name:
            for pad in net.pads:
                # Include pads that are on this specific layer or on all layers
                if layer in pad.layers or '*.Cu' in pad.layers:
                    pads.add((pad.global_x, pad.global_y))
    return pads


def find_single_endpoints(segments: List[Dict]) -> List[Tuple[float, float]]:
    """Find endpoints that appear only once (degree-1 nodes)."""
    endpoints = Counter()
    for seg in segments:
        endpoints[seg['start']] += 1
        endpoints[seg['end']] += 1
    return [pt for pt, count in endpoints.items() if count == 1]


def is_near_any(pt: Tuple[float, float], points: Set[Tuple[float, float]],
                tolerance: float = 0.15) -> bool:
    """Check if point is near any point in the set."""
    for p in points:
        if abs(pt[0] - p[0]) < tolerance and abs(pt[1] - p[1]) < tolerance:
            return True
    return False


def find_orphan_stubs(filename: str, net_name: Optional[str] = None,
                      layer: Optional[str] = None) -> Dict[str, Dict[str, Set[Tuple[float, float]]]]:
    """
    Find all orphan stubs in a PCB file.

    Returns dict of net_name -> {layer: set of orphan positions}
    """
    with open(filename, 'r') as f:
        content = f.read()

    pcb_data = load_pcb_data(filename)

    # Determine which nets to check
    nets_to_check = []
    if net_name:
        nets_to_check = [net_name]
    else:
        # Check all nets
        for nid, net in pcb_data.nets.items():
            if net.name:
                nets_to_check.append(net.name)

    # Determine which layers to check
    layers_to_check = [layer] if layer else ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']

    results = {}

    for net in nets_to_check:
        net_id = get_net_id(content, net)
        if not net_id:
            continue

        vias = get_vias_for_net(content, net_id)
        through_hole_pads = get_through_hole_pads(pcb_data, net)

        net_results = {}
        for lyr in layers_to_check:
            segments = get_segments_for_net_layer(content, net_id, lyr)
            if not segments:
                continue

            single_endpoints = find_single_endpoints(segments)

            # Get layer-specific pads (including SMD pads on this layer)
            layer_pads = get_layer_pads(pcb_data, net, lyr)
            # Combined valid endpoints: vias, through-hole pads, or layer-specific pads
            all_valid_endpoints = vias | through_hole_pads | layer_pads

            # Find orphans - not near any valid endpoint
            orphans = set()
            for pt in single_endpoints:
                if not is_near_any(pt, all_valid_endpoints):
                    orphans.add(pt)

            if orphans:
                net_results[lyr] = orphans

        if net_results:
            results[net] = net_results

    return results


def compare_orphans(file1: str, file2: str, net_name: Optional[str] = None,
                   layer: Optional[str] = None) -> Dict:
    """Compare orphan stubs between two files."""
    orphans1 = find_orphan_stubs(file1, net_name, layer)
    orphans2 = find_orphan_stubs(file2, net_name, layer)

    # Collect all orphan positions
    all_orphans_1 = set()
    all_orphans_2 = set()

    for net, layers in orphans1.items():
        for lyr, pts in layers.items():
            for pt in pts:
                all_orphans_1.add((net, lyr, pt))

    for net, layers in orphans2.items():
        for lyr, pts in layers.items():
            for pt in pts:
                all_orphans_2.add((net, lyr, pt))

    new_orphans = all_orphans_2 - all_orphans_1
    removed_orphans = all_orphans_1 - all_orphans_2

    return {
        'file1_total': len(all_orphans_1),
        'file2_total': len(all_orphans_2),
        'new': new_orphans,
        'removed': removed_orphans
    }


def main():
    parser = argparse.ArgumentParser(
        description='Check for orphan trace stubs in KiCad PCB files')
    parser.add_argument('input', help='Input PCB file')
    parser.add_argument('compare_file', nargs='?',
                       help='Second file to compare against (optional)')
    parser.add_argument('--net', help='Only check this net')
    parser.add_argument('--layer', help='Only check this layer')
    parser.add_argument('--compare', action='store_true',
                       help='Compare two files (requires two input files)')

    args = parser.parse_args()

    if args.compare_file or args.compare:
        if not args.compare_file:
            print("Error: --compare requires two input files")
            sys.exit(1)

        result = compare_orphans(args.input, args.compare_file, args.net, args.layer)

        print(f"\nOrphan Stub Comparison")
        print(f"=" * 50)
        print(f"File 1 ({args.input}): {result['file1_total']} orphans")
        print(f"File 2 ({args.compare_file}): {result['file2_total']} orphans")
        print(f"\nNew orphans in file 2: {len(result['new'])}")
        if result['new']:
            for net, lyr, pt in sorted(result['new'])[:20]:
                print(f"  {net} {lyr}: ({pt[0]:.2f}, {pt[1]:.2f})")
            if len(result['new']) > 20:
                print(f"  ... and {len(result['new']) - 20} more")

        print(f"\nRemoved orphans (fixed): {len(result['removed'])}")

        if result['new']:
            sys.exit(1)
    else:
        orphans = find_orphan_stubs(args.input, args.net, args.layer)

        total = 0
        for net, layers in orphans.items():
            for lyr, pts in layers.items():
                total += len(pts)

        print(f"\nOrphan Stub Analysis: {args.input}")
        print(f"=" * 50)

        if not orphans:
            print("No orphan stubs found!")
        else:
            print(f"Total orphan stubs: {total}\n")
            for net in sorted(orphans.keys()):
                layers = orphans[net]
                for lyr in sorted(layers.keys()):
                    pts = layers[lyr]
                    print(f"{net} on {lyr}: {len(pts)} orphans")
                    for pt in sorted(pts)[:5]:
                        print(f"  ({pt[0]:.2f}, {pt[1]:.2f})")
                    if len(pts) > 5:
                        print(f"  ... and {len(pts) - 5} more")

            sys.exit(1)


if __name__ == '__main__':
    main()

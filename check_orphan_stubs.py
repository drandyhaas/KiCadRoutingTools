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
import math
import sys
from collections import Counter, defaultdict
from typing import Set, Tuple, Dict, List, Optional

# Add current directory to path for imports
sys.path.insert(0, '.')


def load_pcb_data(filename: str):
    """Load PCB data using kicad_parser."""
    from kicad_parser import parse_kicad_pcb
    return parse_kicad_pcb(filename)


def _endpoint_connected(pt: Tuple[float, float], segments: List[Dict],
                        vias: List[Tuple[float, float, float]] = None,
                        ph_pads: List[Tuple[float, float, float]] = None,
                        layer_pads: List[Tuple[float, float, float]] = None,
                        tol: float = 0.05) -> bool:
    """True if a degree-1 endpoint actually lands on same-net copper.

    The naive checker treated an endpoint as connected only if within a fixed
    0.15 mm of a via/pad CENTRE and only matched exact shared segment endpoints,
    so it mis-flagged copper that is electrically connected: a stub ending inside
    a via/pad's copper but >0.15 mm from its centre, a tap landing on another
    trace's body (T-junction), or two traces meeting near-coincidentally. This
    tests against the actual copper extents -- via radius, pad half-extent, trace
    half-width -- plus a small overlap margin, matching the connectivity model.

    vias / ph_pads span all layers; layer_pads are SMD pads on this layer. Each
    entry is (x, y, size) where size is the full diameter / max pad dimension.
    The endpoint's own segment is skipped (an endpoint trivially touches itself).
    """
    px, py = pt
    for group in (vias, ph_pads, layer_pads):
        for cx, cy, csize in (group or ()):
            if math.hypot(px - cx, py - cy) < csize / 2 + tol:
                return True
    for s in segments:
        sx, sy = s['start']
        ex, ey = s['end']
        if (px == sx and py == sy) or (px == ex and py == ey):
            continue  # the endpoint's own segment
        dx, dy = ex - sx, ey - sy
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            continue
        # Nearest point on the whole segment (endpoints included): catches
        # T-junction taps, near-coincident endpoints, and collinear overlap.
        t = max(0.0, min(1.0, ((px - sx) * dx + (py - sy) * dy) / seg_len_sq))
        if math.hypot(px - (sx + t * dx), py - (sy + t * dy)) < s.get('width', 0.0) / 2 + tol:
            return True
    return False


def find_orphan_stubs(filename: str, net_name: Optional[str] = None,
                      layer: Optional[str] = None) -> Dict[str, Dict[str, Set[Tuple[float, float]]]]:
    """
    Find all orphan stubs in a PCB file.

    Returns dict of net_name -> {layer: set of orphan positions}
    """
    pcb_data = load_pcb_data(filename)

    # Build lookup structures once
    # Vias by net_id, with copper size (vias connect on all layers)
    vias_by_net: Dict[int, List[Tuple[float, float, float]]] = defaultdict(list)
    for via in pcb_data.vias:
        vias_by_net[via.net_id].append((via.x, via.y, getattr(via, 'size', 0.0)))

    # Segments by (net_id, layer)
    segments_by_net_layer: Dict[Tuple[int, str], List[Dict]] = defaultdict(list)
    for seg in pcb_data.segments:
        segments_by_net_layer[(seg.net_id, seg.layer)].append({
            'start': (seg.start_x, seg.start_y),
            'end': (seg.end_x, seg.end_y),
            'width': getattr(seg, 'width', 0.0)
        })

    # Determine which nets to check
    if net_name:
        # Find net_id for the given name
        nets_to_check = [(nid, net) for nid, net in pcb_data.nets.items() if net.name == net_name]
    else:
        # Check all nets with segments
        nets_with_segments = {key[0] for key in segments_by_net_layer.keys()}
        nets_to_check = [(nid, net) for nid, net in pcb_data.nets.items()
                         if net.name and nid in nets_with_segments]

    # Determine which layers to check
    layers_to_check = [layer] if layer else ['F.Cu', 'B.Cu', 'In1.Cu', 'In2.Cu']

    results = {}

    for net_id, net in nets_to_check:
        vias = vias_by_net[net_id]

        # Through-hole pads (drill > 0 or *.Cu) connect on all layers; carry size.
        through_hole_pads = []
        for pad in net.pads:
            if pad.drill > 0 or '*.Cu' in pad.layers:
                through_hole_pads.append((pad.global_x, pad.global_y,
                                          max(pad.size_x, pad.size_y)))

        net_results = {}
        for lyr in layers_to_check:
            segments = segments_by_net_layer.get((net_id, lyr), [])
            if not segments:
                continue

            # Find single endpoints (degree-1 nodes)
            endpoints = Counter()
            for seg in segments:
                endpoints[seg['start']] += 1
                endpoints[seg['end']] += 1
            single_endpoints = [pt for pt, count in endpoints.items() if count == 1]

            if not single_endpoints:
                continue

            # SMD pads on this layer (carry size).
            layer_pads = []
            for pad in net.pads:
                if lyr in pad.layers or '*.Cu' in pad.layers:
                    layer_pads.append((pad.global_x, pad.global_y,
                                       max(pad.size_x, pad.size_y)))

            # A degree-1 endpoint is an orphan only if it touches NO same-net
            # copper: not within a via/pad's copper extent, and not on another
            # same-net segment (T-junction / near-coincident / overlap). Matching
            # the real copper geometry instead of a fixed centre tolerance keeps
            # connected stubs from being reported as dead ends.
            orphans = {pt for pt in single_endpoints
                       if not _endpoint_connected(pt, segments, vias,
                                                  through_hole_pads, layer_pads)}

            if orphans:
                net_results[lyr] = orphans

        if net_results:
            results[net.name] = net_results

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
        print(f"=" * 60)
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
        print(f"Loading {args.input}...")
        orphans = find_orphan_stubs(args.input, args.net, args.layer)

        total = 0
        for net, layers in orphans.items():
            for lyr, pts in layers.items():
                total += len(pts)

        print(f"\nChecking for orphan trace stubs...")

        print("\n" + "=" * 60)
        if not orphans:
            print("NO ORPHAN STUBS FOUND!")
        else:
            print(f"FOUND {total} ORPHAN STUBS:\n")
            for net in sorted(orphans.keys()):
                layers = orphans[net]
                for lyr in sorted(layers.keys()):
                    pts = layers[lyr]
                    print(f"  {net} on {lyr}: {len(pts)} orphans")
                    for pt in sorted(pts)[:5]:
                        print(f"    ({pt[0]:.2f}, {pt[1]:.2f})")
                    if len(pts) > 5:
                        print(f"    ... and {len(pts) - 5} more")

        print("=" * 60)

        if orphans:
            sys.exit(1)


if __name__ == '__main__':
    main()

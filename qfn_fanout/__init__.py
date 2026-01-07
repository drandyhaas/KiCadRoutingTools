"""
QFN/QFP Fanout Strategy - Creates escape routing for QFN/QFP packages.

Generic module that analyzes actual pad geometry to determine:
- Which side each pad is on (based on position and pad orientation)
- Escape direction (perpendicular to pad's long axis)
- Stub length (based on chip size)
- Fan-out pattern (endpoints maximally separated)

Works with any QFN/QFP package regardless of pin count or size.
"""

import math
from typing import List, Dict, Tuple, Optional
from collections import defaultdict
import fnmatch

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout.types import QFNLayout, PadInfo, FanoutStub
from qfn_fanout.layout import analyze_qfn_layout, analyze_pad
from qfn_fanout.geometry import calculate_fanout_stub


# Public API
__all__ = [
    'generate_qfn_fanout',
    'main',
    # Types re-exported for external use
    'QFNLayout',
    'PadInfo',
    'FanoutStub',
]


def check_endpoint_spacing(stubs: List[FanoutStub], min_spacing: float) -> List[Tuple[int, int, float]]:
    """Check for endpoints that are too close together."""
    collisions = []
    for i, s1 in enumerate(stubs):
        for j, s2 in enumerate(stubs[i+1:], i+1):
            if s1.pad.net_id == s2.pad.net_id:
                continue
            dist = math.sqrt((s1.stub_end[0] - s2.stub_end[0])**2 +
                           (s1.stub_end[1] - s2.stub_end[1])**2)
            if dist < min_spacing:
                collisions.append((i, j, dist))
    return collisions


def generate_qfn_fanout(footprint: Footprint,
                        pcb_data: PCBData,
                        net_filter: Optional[List[str]] = None,
                        layer: str = "F.Cu",
                        track_width: float = 0.1,
                        clearance: float = 0.1,
                        fan_factor: float = 0.5) -> Tuple[List[Dict], List[Dict]]:
    """
    Generate QFN fanout tracks for a footprint.

    Creates two-segment stubs:
    1. Straight segment perpendicular to chip edge
    2. 45 degree segment fanning outward from center

    Edge pads get short straight (just past pad) + long 45 degree for maximum fan.
    Center pads get full straight (no 45 degree) since already separated.

    Args:
        footprint: The QFN/QFP footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        layer: Routing layer (default F.Cu)
        track_width: Width of fanout tracks
        clearance: Minimum clearance (used for track spacing and pad escape margin)
        fan_factor: Unused, kept for compatibility

    Returns:
        (tracks, vias) - tracks are the segments, vias is empty
    """
    layout = analyze_qfn_layout(footprint)
    if layout is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a QFN/QFP")
        return [], []

    print(f"QFN/QFP Layout Analysis for {footprint.reference}:")
    print(f"  Center: ({layout.center_x:.2f}, {layout.center_y:.2f})")
    print(f"  Bounding box: X[{layout.min_x:.2f}, {layout.max_x:.2f}], Y[{layout.min_y:.2f}, {layout.max_y:.2f}]")
    print(f"  Size: {layout.width:.2f} x {layout.height:.2f} mm")
    print(f"  Detected pad pitch: {layout.pad_pitch:.2f} mm")
    print(f"  Edge tolerance: {layout.edge_tolerance:.2f} mm")
    print(f"  Stub length: pad_width / 2 + clearance (clears pad before bend)")
    print(f"  Layer: {layer}")

    # Analyze all pads
    pad_infos: List[PadInfo] = []
    side_counts = defaultdict(int)

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        if net_filter:
            matched = any(fnmatch.fnmatch(pad.net_name, pattern) for pattern in net_filter)
            if not matched:
                continue

        pad_info = analyze_pad(pad, layout)
        if pad_info.side == 'center':
            continue  # Skip center/EP pads

        pad_infos.append(pad_info)
        side_counts[pad_info.side] += 1

    print(f"  Found {len(pad_infos)} pads to fanout:")
    for side, count in sorted(side_counts.items()):
        print(f"    {side}: {count} pads")

    # Show sample pad geometry
    if pad_infos:
        sample = pad_infos[0]
        print(f"  Sample pad geometry: {sample.pad_length:.2f} x {sample.pad_width:.2f} mm")

    if not pad_infos:
        return [], []

    # Build stubs
    stubs: List[FanoutStub] = []

    # Max diagonal length for corner pads = chip_width / 3
    max_diagonal_length = max(layout.width, layout.height) / 3

    for pad_info in pad_infos:
        # Straight stub length = pad_width / 2 + clearance (to clear the pad before bending)
        # pad_width is the dimension perpendicular to the chip edge (escape direction)
        straight_length = pad_info.pad_width / 2 + clearance
        corner_pos, stub_end = calculate_fanout_stub(
            pad_info, layout, straight_length, max_diagonal_length, fan_factor
        )

        stub = FanoutStub(
            pad=pad_info.pad,
            pad_pos=(pad_info.pad.global_x, pad_info.pad.global_y),
            corner_pos=corner_pos,
            stub_end=stub_end,
            side=pad_info.side,
            layer=layer
        )
        stubs.append(stub)

    # Generate tracks - two segments per stub
    tracks = []
    for stub in stubs:
        # Segment 1: Straight from pad to corner
        dx1 = abs(stub.corner_pos[0] - stub.pad_pos[0])
        dy1 = abs(stub.corner_pos[1] - stub.pad_pos[1])
        if dx1 > 0.001 or dy1 > 0.001:
            tracks.append({
                'start': stub.pad_pos,
                'end': stub.corner_pos,
                'width': track_width,
                'layer': stub.layer,
                'net_id': stub.net_id
            })

        # Segment 2: 45 degree from corner to end
        dx2 = abs(stub.stub_end[0] - stub.corner_pos[0])
        dy2 = abs(stub.stub_end[1] - stub.corner_pos[1])
        if dx2 > 0.001 or dy2 > 0.001:
            tracks.append({
                'start': stub.corner_pos,
                'end': stub.stub_end,
                'width': track_width,
                'layer': stub.layer,
                'net_id': stub.net_id
            })

    print(f"  Generated {len(tracks)} track segments ({len(stubs)} stubs x 2 segments)")

    # Validate endpoint spacing
    min_spacing = track_width + clearance
    collisions = check_endpoint_spacing(stubs, min_spacing)

    if collisions:
        print(f"  WARNING: {len(collisions)} endpoint pairs too close!")
        for i, j, dist in collisions[:5]:
            print(f"    {stubs[i].pad.net_name} <-> {stubs[j].pad.net_name}: {dist:.3f}mm")
        print(f"  Consider increasing clearance")
    else:
        print(f"  Validated: No endpoint collisions")

    return tracks, []


def main():
    """Run QFN fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate QFN/QFP fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='kicad_files/qfn_fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layer', '-l', default='F.Cu',
                        help='Routing layer')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Clearance in mm (track spacing and margin past pad before bend)')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')

    args = parser.parse_args()

    print(f"Parsing {args.pcb}...")
    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect QFN/QFP component if not specified
    if args.component is None:
        qfn_components = find_components_by_type(pcb_data, 'QFN')
        if not qfn_components:
            qfn_components = find_components_by_type(pcb_data, 'QFP')
        if qfn_components:
            args.component = qfn_components[0].reference
            print(f"Auto-detected QFN/QFP component: {args.component}")
            if len(qfn_components) > 1:
                print(f"  (Other QFN/QFPs found: {[fp.reference for fp in qfn_components[1:]]})")
        else:
            print("Error: No QFN/QFP components found in PCB")
            print(f"Available components: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
        return 1

    footprint = pcb_data.footprints[args.component]
    print(f"\nFound {args.component}: {footprint.footprint_name}")
    print(f"  Position: ({footprint.x:.2f}, {footprint.y:.2f})")
    print(f"  Rotation: {footprint.rotation}deg")
    print(f"  Pads: {len(footprint.pads)}")

    tracks, vias = generate_qfn_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        layer=args.layer,
        track_width=args.width,
        clearance=args.clearance
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")

    return 0


if __name__ == '__main__':
    exit(main())

"""
QFN/QFP Fanout Strategy - Creates escape routing for QFN/QFP packages.

Generic script that analyzes actual pad geometry to determine:
- Which side each pad is on (based on position and pad orientation)
- Escape direction (perpendicular to pad's long axis)
- Stub length (based on chip size)
- Fan-out pattern (endpoints maximally separated)

Works with any QFN/QFP package regardless of pin count or size.
"""

import math
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from collections import defaultdict
import fnmatch

from kicad_parser import parse_kicad_pcb, Pad, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb


@dataclass
class QFNLayout:
    """Represents the QFN package layout derived from pad analysis."""
    center_x: float
    center_y: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    width: float
    height: float
    pad_pitch: float
    edge_tolerance: float  # How close to edge to be considered "on edge"


@dataclass
class PadInfo:
    """Analyzed information about a pad."""
    pad: Pad
    side: str  # 'top', 'bottom', 'left', 'right', 'center'
    escape_direction: Tuple[float, float]  # Unit vector pointing outward
    pad_length: float  # Length of pad (along edge)
    pad_width: float   # Width of pad (perpendicular to edge)


@dataclass
class FanoutStub:
    """A fanout stub from a QFN pad - two segments: straight then 45°."""
    pad: Pad
    pad_pos: Tuple[float, float]
    corner_pos: Tuple[float, float]  # Where straight meets 45°
    stub_end: Tuple[float, float]    # Final fanned-out endpoint
    side: str
    layer: str = "F.Cu"

    @property
    def net_id(self) -> int:
        return self.pad.net_id


def analyze_qfn_layout(footprint: Footprint) -> Optional[QFNLayout]:
    """
    Analyze a footprint to extract QFN layout parameters.
    Derives everything from actual pad positions and sizes.
    """
    pads = footprint.pads
    if len(pads) < 4:
        return None

    # Get bounding box from pad positions
    x_positions = [p.global_x for p in pads]
    y_positions = [p.global_y for p in pads]
    min_x, max_x = min(x_positions), max(x_positions)
    min_y, max_y = min(y_positions), max(y_positions)

    width = max_x - min_x
    height = max_y - min_y

    # Find pads on each edge to determine pitch
    # Use a tolerance based on the package size
    edge_tolerance = min(width, height) * 0.1  # 10% of smaller dimension

    # Get pads on top edge
    top_pads = [p for p in pads if abs(p.global_y - min_y) < edge_tolerance]
    right_pads = [p for p in pads if abs(p.global_x - max_x) < edge_tolerance]
    bottom_pads = [p for p in pads if abs(p.global_y - max_y) < edge_tolerance]
    left_pads = [p for p in pads if abs(p.global_x - min_x) < edge_tolerance]

    # Calculate pitch from the edge with most pads
    all_edge_pads = [(top_pads, 'x'), (bottom_pads, 'x'), (left_pads, 'y'), (right_pads, 'y')]
    pad_pitch = None

    for edge_pads, axis in all_edge_pads:
        if len(edge_pads) > 1:
            if axis == 'x':
                positions = sorted(set(p.global_x for p in edge_pads))
            else:
                positions = sorted(set(p.global_y for p in edge_pads))

            if len(positions) > 1:
                pitches = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
                # Use the minimum non-zero pitch
                min_pitch = min(p for p in pitches if p > 0.1)
                if pad_pitch is None or min_pitch < pad_pitch:
                    pad_pitch = min_pitch

    if pad_pitch is None:
        pad_pitch = 0.5  # Fallback

    return QFNLayout(
        center_x=(min_x + max_x) / 2,
        center_y=(min_y + max_y) / 2,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        width=width,
        height=height,
        pad_pitch=pad_pitch,
        edge_tolerance=edge_tolerance
    )


def analyze_pad(pad: Pad, layout: QFNLayout) -> PadInfo:
    """
    Analyze a pad to determine its side and escape direction.
    Uses pad position AND geometry to determine orientation.
    """
    x, y = pad.global_x, pad.global_y

    # Distance to each edge
    dist_top = abs(y - layout.min_y)
    dist_bottom = abs(y - layout.max_y)
    dist_left = abs(x - layout.min_x)
    dist_right = abs(x - layout.max_x)

    min_dist = min(dist_top, dist_bottom, dist_left, dist_right)
    tol = layout.edge_tolerance

    # Determine side based on position
    if min_dist == dist_top and dist_top < tol:
        side = 'top'
    elif min_dist == dist_bottom and dist_bottom < tol:
        side = 'bottom'
    elif min_dist == dist_left and dist_left < tol:
        side = 'left'
    elif min_dist == dist_right and dist_right < tol:
        side = 'right'
    else:
        side = 'center'

    # Determine escape direction and pad dimensions based on side
    # For QFN/QFP, pads are typically elongated perpendicular to the chip edge
    if side == 'top':
        escape_direction = (0.0, -1.0)  # Escape upward (negative Y)
        pad_length = pad.size_x  # Along the edge
        pad_width = pad.size_y   # Perpendicular to edge
    elif side == 'bottom':
        escape_direction = (0.0, 1.0)   # Escape downward
        pad_length = pad.size_x
        pad_width = pad.size_y
    elif side == 'left':
        escape_direction = (-1.0, 0.0)  # Escape leftward
        pad_length = pad.size_y  # Along the edge (vertical)
        pad_width = pad.size_x   # Perpendicular
    elif side == 'right':
        escape_direction = (1.0, 0.0)   # Escape rightward
        pad_length = pad.size_y
        pad_width = pad.size_x
    else:
        escape_direction = (0.0, 0.0)
        pad_length = max(pad.size_x, pad.size_y)
        pad_width = min(pad.size_x, pad.size_y)

    return PadInfo(
        pad=pad,
        side=side,
        escape_direction=escape_direction,
        pad_length=pad_length,
        pad_width=pad_width
    )


def calculate_fanout_stub(pad_info: PadInfo, layout: QFNLayout,
                          straight_length: float, max_diagonal_length: float,
                          fan_factor: float) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    Calculate fanout stub with two segments: straight then 45°.

    Returns (corner_pos, end_pos):
    - corner_pos: where straight segment ends and 45° begins
    - end_pos: final fanned-out endpoint

    The straight segment extends perpendicular to the chip edge (uniform for all pads).
    The 45° diagonal length varies by position:
    - Center pads: diagonal length = 0
    - Corner pads: diagonal length = max_diagonal_length
    - Linear interpolation in between

    Args:
        straight_length: Length of straight stub (pad_length / 2, uniform for all)
        max_diagonal_length: Max diagonal length for corner pads (chip_width / 3)
    """
    pad = pad_info.pad
    pad_x, pad_y = pad.global_x, pad.global_y
    side = pad_info.side

    if side == 'center':
        return ((pad_x, pad_y), (pad_x, pad_y))

    # Escape direction perpendicular to edge
    esc_x, esc_y = pad_info.escape_direction

    # Corner point: end of straight segment
    corner_x = pad_x + esc_x * straight_length
    corner_y = pad_y + esc_y * straight_length

    # Calculate position along edge (0 = center, 1 = corner)
    if side in ('top', 'bottom'):
        half_width = layout.width / 2
        offset_from_center = abs(pad_x - layout.center_x)
        edge_position = offset_from_center / half_width if half_width > 0 else 0
    else:
        half_height = layout.height / 2
        offset_from_center = abs(pad_y - layout.center_y)
        edge_position = offset_from_center / half_height if half_height > 0 else 0

    # Diagonal length: 0 at center, max_diagonal_length at corners
    diagonal_length = edge_position * max_diagonal_length

    if diagonal_length < 0.01:
        # No 45° segment needed (center pads)
        return ((corner_x, corner_y), (corner_x, corner_y))

    # True 45° means equal movement in escape direction and fan direction
    # Distance along 45° line = diagonal_length, so each component = diagonal_length / sqrt(2)
    diag_component = diagonal_length / math.sqrt(2)

    if side in ('top', 'bottom'):
        # Horizontal edge - fan left/right based on position
        offset_from_center = pad_x - layout.center_x
        fan_dir = 1 if offset_from_center >= 0 else -1

        # True 45°: equal dx and dy
        end_x = corner_x + fan_dir * diag_component
        end_y = corner_y + esc_y * diag_component
    else:
        # Vertical edge - fan up/down based on position
        offset_from_center = pad_y - layout.center_y
        fan_dir = 1 if offset_from_center >= 0 else -1

        # True 45°: equal dx and dy
        end_x = corner_x + esc_x * diag_component
        end_y = corner_y + fan_dir * diag_component

    return ((corner_x, corner_y), (end_x, end_y))


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
                        stub_length: Optional[float] = None,
                        fan_factor: float = 0.5) -> Tuple[List[Dict], List[Dict]]:
    """
    Generate QFN fanout tracks for a footprint.

    Creates two-segment stubs:
    1. Straight segment perpendicular to chip edge
    2. 45° segment fanning outward from center

    Edge pads get short straight (just past pad) + long 45° for maximum fan.
    Center pads get full straight (no 45°) since already separated.

    Args:
        footprint: The QFN/QFP footprint
        pcb_data: Full PCB data
        net_filter: Optional list of net patterns to include
        layer: Routing layer (default F.Cu)
        track_width: Width of fanout tracks
        clearance: Minimum clearance between tracks
        stub_length: Total length of stubs (default: chip width / 2)
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
    print(f"  Stub length: pad_length / 2 (uniform for all pads)")
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
        # Straight stub length = pad_length / 2 (uniform for all pads)
        straight_length = pad_info.pad_length / 2
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

        # Segment 2: 45° from corner to end
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
        print(f"  Consider reducing fan_factor or increasing stub_length")
    else:
        print(f"  Validated: No endpoint collisions")

    return tracks, []


def main():
    """Run QFN fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate QFN/QFP fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='qfn_fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layer', '-l', default='F.Cu',
                        help='Routing layer')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Track clearance in mm')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--stub-length', '-s', type=float, default=None,
                        help='Stub length (default: chip width / 2)')

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
        clearance=args.clearance,
        stub_length=args.stub_length
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

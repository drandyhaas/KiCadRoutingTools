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

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import parse_kicad_pcb, Footprint, PCBData, find_components_by_type
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout.types import QFNLayout, PadInfo, FanoutStub
from bga_fanout.constants import POSITION_TOLERANCE
from net_queries import matches_net_filter
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
                        extension: float = 0.1,
                        clearance: float = 0.1,
                        grid_step: float = 0.0) -> Tuple[List[Dict], List[Dict], List[str]]:
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
        extension: Extension past pad edge before bend (mm)

    Returns:
        (tracks, vias, failed_nets) - tracks are the segments, vias is empty.
        failed_nets is the deduplicated list of net names whose stubs landed
        too close to another net's stub (endpoint spacing < track_width +
        extension); those tracks are still emitted but flagged as failing
        clearance so the GUI can surface them.
    """
    layout = analyze_qfn_layout(footprint)
    if layout is None:
        print(f"Warning: {footprint.reference} doesn't appear to be a QFN/QFP")
        return [], [], []

    # Sanity-check pad geometry before escaping: overlapping same-footprint pads
    # mean the pad rotation/size is modelled wrong, so the stubs would be placed
    # across neighbouring pads (issue: rotated-package fanout). Warn loudly.
    from check_pads import find_pad_overlaps
    _ov = find_pad_overlaps(pcb_data, component=footprint.reference)
    if _ov:
        print(f"  WARNING: {footprint.reference} has {len(_ov)} overlapping "
              f"different-net pad pair(s) - pad geometry looks wrong, fanout "
              f"stubs may cross pads. Run: python3 check_pads.py <board> "
              f"--component {footprint.reference}")

    print(f"QFN/QFP Layout Analysis for {footprint.reference}:")
    print(f"  Center: ({layout.center_x:.2f}, {layout.center_y:.2f})")
    print(f"  Bounding box: X[{layout.min_x:.2f}, {layout.max_x:.2f}], Y[{layout.min_y:.2f}, {layout.max_y:.2f}]")
    print(f"  Size: {layout.width:.2f} x {layout.height:.2f} mm")
    print(f"  Detected pad pitch: {layout.pad_pitch:.2f} mm")
    print(f"  Edge tolerance: {layout.edge_tolerance:.2f} mm")
    print(f"  Stub length: pad_width / 2 + extension (clears pad before bend)")
    print(f"  Layer: {layer}")

    # Analyze all pads
    pad_infos: List[PadInfo] = []
    side_counts = defaultdict(int)

    for pad in footprint.pads:
        if not pad.net_name or pad.net_id == 0:
            continue

        # Skip unconnected nets (KiCad pins not connected in schematic)
        if pad.net_name.lower().startswith('unconnected-'):
            continue

        if net_filter and not matches_net_filter(pad.net_name, net_filter):
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
        return [], [], []

    # Build stubs
    stubs: List[FanoutStub] = []

    # Max diagonal length for corner pads = chip_width / 3
    max_diagonal_length = max(layout.width, layout.height) / 3

    for pad_info in pad_infos:
        # Straight stub length = pad_width / 2 + extension (to clear the pad before bending)
        # pad_width is the dimension perpendicular to the chip edge (escape direction)
        straight_length = pad_info.pad_width / 2 + extension
        corner_pos, stub_end = calculate_fanout_stub(
            pad_info, layout, straight_length, max_diagonal_length, grid_step
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

    # Foreign-pad clearance (issue #123). The QFN fan emits each stub blind to
    # other components' pads, so a stub on a fine-pitch part routed out toward a
    # neighbouring passive grazes it within clearance (PAD-SEGMENT). Mirror the
    # bga_fanout escape clearing: shorten the 45 fan inward to clear the pad
    # (connectivity-neutral - the straight segment still escapes the chip), and
    # if even the straight escape itself grazes a pad, drop that stub and warn.
    from bga_fanout.reroute import _seg_hits_pad
    from obstacle_map import build_base_obstacle_map, build_layer_map, check_line_clearance
    from routing_config import GridRouteConfig
    margin = clearance + track_width / 2

    # Full obstacle map so a stub is checked against foreign TRACKS and VIAS too,
    # not just foreign pads (issue #149 part 2). The part's own nets are excluded
    # (nets_to_route) so same-net taps stay legal; extra_clearance = track half so
    # the cell test is a clearance test. Checked at stub-creation time (below), so
    # a stub/fan that would extend into a foreign obstacle is shortened or dropped.
    _obs_cfg = GridRouteConfig(layers=list(pcb_data.board_info.copper_layers or [layer]),
                               track_width=track_width, clearance=clearance)
    _obs_layer_map = build_layer_map(_obs_cfg.layers)
    _fanned_net_ids = [p.net_id for p in footprint.pads if p.net_id]
    _obstacles = build_base_obstacle_map(pcb_data, _obs_cfg, nets_to_route=_fanned_net_ids,
                                         extra_clearance=track_width / 2)
    _obs_layer_idx = _obs_layer_map.get(layer)
    # Global bbox of this part's pads (layout.min_* are in the footprint LOCAL
    # frame now, so they can't bound the global foreign-pad search window).
    _gxs = [p.global_x for p in footprint.pads]
    _gys = [p.global_y for p in footprint.pads]
    fp_lo_x, fp_hi_x = min(_gxs) - 3.0, max(_gxs) + 3.0
    fp_lo_y, fp_hi_y = min(_gys) - 3.0, max(_gys) + 3.0
    foreign_pads = [p for plist in pcb_data.pads_by_net.values() for p in plist
                    if p.component_ref != footprint.reference
                    and fp_lo_x <= p.global_x <= fp_hi_x and fp_lo_y <= p.global_y <= fp_hi_y]

    def _seg_grazes(p1, p2, net_id):
        # Foreign tracks / vias via the shared obstacle map (issue #149 part 2).
        if _obs_layer_idx is not None and not check_line_clearance(
                _obstacles, p1[0], p1[1], p2[0], p2[1], _obs_layer_idx, _obs_cfg):
            return True
        # Foreign pads, geometric + rotation-exact (issue #123).
        for pad in foreign_pads:
            if pad.net_id == net_id:
                continue
            if pad.drill <= 0 and layer not in (pad.layers or []):
                continue
            if _seg_hits_pad(p1[0], p1[1], p2[0], p2[1], pad, margin=margin):
                return True
        return False

    qfn_dropped: List[str] = []
    n_short = 0
    kept_stubs: List[FanoutStub] = []
    for stub in stubs:
        nid = stub.net_id
        if _seg_grazes(stub.pad_pos, stub.corner_pos, nid):
            # The perpendicular escape itself hits a foreign pad - cannot fan out.
            if stub.pad.net_name and stub.pad.net_name not in qfn_dropped:
                qfn_dropped.append(stub.pad.net_name)
            continue
        if _seg_grazes(stub.corner_pos, stub.stub_end, nid):
            # Shorten the 45 fan toward the corner until it clears (worst case
            # collapse it entirely - the straight escape is already clear).
            cx, cy = stub.corner_pos
            ex, ey = stub.stub_end
            new_end = stub.corner_pos
            for i in range(1, 9):
                t = 1.0 - i / 9.0  # walk inward from current tip toward corner
                cand = (cx + (ex - cx) * t, cy + (ey - cy) * t)
                if not _seg_grazes(stub.corner_pos, cand, nid):
                    new_end = cand
                    break
            stub.stub_end = new_end
            n_short += 1
        kept_stubs.append(stub)
    if n_short or qfn_dropped:
        print(f"  Pad-clearance: shortened {n_short} fan(s); dropped {len(qfn_dropped)} "
              f"stub(s) grazing a foreign pad (issue #123)")
    stubs = kept_stubs

    # Generate tracks - two segments per stub
    tracks = []
    for stub in stubs:
        # Segment 1: Straight from pad to corner
        dx1 = abs(stub.corner_pos[0] - stub.pad_pos[0])
        dy1 = abs(stub.corner_pos[1] - stub.pad_pos[1])
        if dx1 > POSITION_TOLERANCE or dy1 > POSITION_TOLERANCE:
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
        if dx2 > POSITION_TOLERANCE or dy2 > POSITION_TOLERANCE:
            tracks.append({
                'start': stub.corner_pos,
                'end': stub.stub_end,
                'width': track_width,
                'layer': stub.layer,
                'net_id': stub.net_id
            })

    print(f"  Generated {len(tracks)} track segments ({len(stubs)} stubs x 2 segments)")

    # Fine-pitch escape warning (issue #97): the 45-degree fan keeps adjacent
    # stubs parallel at pitch/sqrt(2) forever - fanning separates tips along
    # the diagonal, not laterally. At common defaults (clearance 0.25) the
    # router cannot launch from or pass between these stubs and every net
    # fails with 'no rippable blockers found'. Tell the user the workable
    # parameters up front instead.
    if layout.pad_pitch and layout.pad_pitch < 0.8:
        lateral = layout.pad_pitch / math.sqrt(2)
        # Escape at the stub's own width: route_track/2 + clearance +
        # stub_track/2 must fit in `lateral`.
        max_clear = lateral - track_width
        # One 0.05 grid step of margin for quantization, rounded down to 0.05,
        # capped at 0.15 (the combination verified to escape 0.5 mm LQFP fans).
        suggest_clear = min(max(0.05, int((max_clear - 0.05) / 0.05) * 0.05), 0.15)
        print(f"  NOTE: {layout.pad_pitch:.2f} mm pitch keeps adjacent fan stubs only "
              f"{lateral:.3f} mm apart (pitch/sqrt2).")
        print(f"  Routing these nets needs clearance below {max_clear:.2f} mm (with "
              f"{track_width:.2f} mm track) plus grid-quantization margin; the "
              f"default 0.25 clearance / 0.1 grid will fail to escape.")
        print(f"  Suggested: route.py --grid-step 0.05 --clearance {suggest_clear:.2f} "
              f"--track-width {track_width:.2f} for this component's nets.")

    # Validate endpoint spacing
    min_spacing = track_width + extension
    collisions = check_endpoint_spacing(stubs, min_spacing)

    failed_nets: List[str] = list(qfn_dropped)
    if collisions:
        print(f"  WARNING: {len(collisions)} endpoint pairs too close!")
        for i, j, dist in collisions[:5]:
            print(f"    {stubs[i].pad.net_name} <-> {stubs[j].pad.net_name}: {dist:.3f}mm")
        print(f"  Consider increasing extension")
        # Collect the deduplicated set of nets involved in any collision -
        # these are the "failed" nets the GUI surfaces.
        seen = set()
        for i, j, _dist in collisions:
            for name in (stubs[i].pad.net_name, stubs[j].pad.net_name):
                if name and name not in seen:
                    seen.add(name)
                    failed_nets.append(name)
    else:
        print(f"  Validated: No endpoint collisions")

    return tracks, [], failed_nets


def main():
    """Run QFN fanout generation."""
    import argparse

    parser = argparse.ArgumentParser(description='Generate QFN/QFP fanout routing')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--output', '-o', default='kicad_files/qfn_fanout_test.kicad_pcb',
                        help='Output PCB file')
    parser.add_argument('--component', '-c', default=None,
                        help='Component reference (auto-detected if not specified)')
    parser.add_argument('--layer', '-l', default=None,
                        help='Routing layer (default: the layer the component '
                             'is mounted on)')
    parser.add_argument('--width', '-w', type=float, default=0.1,
                        help='Track width in mm')
    parser.add_argument('--extension', type=float, default=0.1,
                        help='Extension past pad edge before bend (mm)')
    parser.add_argument('--clearance', type=float, default=0.1,
                        help='Min clearance to other-net pads (mm); stubs that '
                             'would graze a foreign pad are shortened or dropped')
    parser.add_argument('--nets', '-n', nargs='*',
                        help='Net patterns to include')
    parser.add_argument('--grid-step', type=float, default=0.1,
                        help='Routing grid step in mm (default: 0.1). Fanned stub ends are '
                             'snapped to this grid so the router gets on-grid terminals (issue '
                             '#149); MATCH the --grid-step you pass to route.py.')

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

    # Default the stub layer to the component's mounted layer (issue #96:
    # B.Cu-mounted parts silently got F.Cu stubs floating over their pads,
    # and route.py then reported their nets routed while electrically open).
    layer = args.layer
    if layer is None:
        layer = footprint.layer or 'F.Cu'
        print(f"  Layer: {layer} (from footprint)")
    elif footprint.layer and layer != footprint.layer:
        print(f"  WARNING: --layer {layer} differs from {args.component}'s "
              f"mounted layer {footprint.layer} - stubs will NOT touch the "
              f"SMD pads unless this is intentional")

    tracks, vias, _failed_nets = generate_qfn_fanout(
        footprint,
        pcb_data,
        net_filter=args.nets,
        layer=layer,
        track_width=args.width,
        extension=args.extension,
        clearance=args.clearance,
        grid_step=args.grid_step
    )

    if tracks:
        print(f"\nWriting {len(tracks)} tracks to {args.output}...")
        net_id_to_name = {nid: net.name for nid, net in pcb_data.nets.items()}
        add_tracks_and_vias_to_pcb(args.pcb, args.output, tracks, vias,
                                   net_id_to_name=net_id_to_name)
        print("Done!")
    else:
        print("\nNo fanout tracks generated")
        # Still produce the output file (board unchanged) so a multi-step
        # pipeline can continue - otherwise a fanout that finds nothing to do
        # (e.g. the component is already fanned on a retry) leaves the next step
        # with no input file.
        if getattr(args, 'output', None):
            import shutil
            shutil.copyfile(args.pcb, args.output)
            print(f"Wrote board through to {args.output} (unchanged)")

    return 0


if __name__ == '__main__':
    exit(main())

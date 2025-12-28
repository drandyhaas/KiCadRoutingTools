#!/usr/bin/env python3
"""
Switch all segments for matching nets to a specified layer.

Usage:
    python switch_to_layer.py "*rx*top*" --input input.kicad_pcb --output output.kicad_pcb --to-layer "In3.Cu"
"""

import argparse
import fnmatch
from kicad_parser import parse_kicad_pcb
from kicad_writer import modify_segment_layers, generate_via_sexpr


def main():
    parser = argparse.ArgumentParser(
        description='Switch all segments for matching nets to a specified layer'
    )
    parser.add_argument('net_patterns', nargs='+',
                        help='Net name patterns to match (supports * and ? wildcards)')
    parser.add_argument('--input', '-i', required=True,
                        help='Input PCB file')
    parser.add_argument('--output', '-o', required=True,
                        help='Output PCB file')
    parser.add_argument('--to-layer', '-l', required=True,
                        help='Target layer (e.g., "In3.Cu", "F.Cu", "B.Cu")')
    parser.add_argument('--via-size', type=float, default=0.3,
                        help='Via outer diameter in mm (default: 0.3)')
    parser.add_argument('--via-drill', type=float, default=0.2,
                        help='Via drill diameter in mm (default: 0.2)')
    parser.add_argument('--dry-run', '-n', action='store_true',
                        help='Show what would be changed without writing output')

    args = parser.parse_args()

    # Parse PCB
    print(f"Parsing {args.input}...")
    pcb_data = parse_kicad_pcb(args.input)

    # Build list of all net names
    all_net_names = {net.name: net.net_id for net in pcb_data.nets.values() if net.name}

    # Find matching nets
    matching_nets = {}
    for pattern in args.net_patterns:
        if '*' in pattern or '?' in pattern:
            matches = [(name, net_id) for name, net_id in all_net_names.items()
                       if fnmatch.fnmatch(name, pattern)]
            if not matches:
                print(f"Warning: Pattern '{pattern}' matched no nets")
            for name, net_id in matches:
                matching_nets[net_id] = name
        else:
            # Exact match
            if pattern in all_net_names:
                matching_nets[all_net_names[pattern]] = pattern
            else:
                print(f"Warning: Net '{pattern}' not found")

    if not matching_nets:
        print("Error: No nets matched the given patterns")
        return 1

    print(f"Found {len(matching_nets)} matching nets:")
    for net_id, name in sorted(matching_nets.items(), key=lambda x: x[1]):
        print(f"  {name} (id={net_id})")

    # Collect segments to modify
    segment_mods = []
    fcu_segments_by_net = {}  # net_id -> list of segments being switched from F.Cu
    for seg in pcb_data.segments:
        if seg.net_id in matching_nets:
            if seg.layer == args.to_layer:
                continue  # Already on target layer
            segment_mods.append({
                'start': (seg.start_x, seg.start_y),
                'end': (seg.end_x, seg.end_y),
                'net_id': seg.net_id,
                'old_layer': seg.layer,
                'new_layer': args.to_layer
            })
            # Track F.Cu segments for via insertion
            if seg.layer == 'F.Cu':
                if seg.net_id not in fcu_segments_by_net:
                    fcu_segments_by_net[seg.net_id] = []
                fcu_segments_by_net[seg.net_id].append(seg)

    if not segment_mods:
        print(f"No segments need to be changed (all matching segments already on {args.to_layer})")
        return 0

    # Find pads that need vias (segments on F.Cu connecting to pads)
    vias_to_add = []
    tolerance = 0.05
    for net_id, segments in fcu_segments_by_net.items():
        pads = pcb_data.pads_by_net.get(net_id, [])
        for pad in pads:
            # Check if any segment endpoint touches this pad
            pad_has_segment = False
            for seg in segments:
                if ((abs(seg.start_x - pad.global_x) < tolerance and
                     abs(seg.start_y - pad.global_y) < tolerance) or
                    (abs(seg.end_x - pad.global_x) < tolerance and
                     abs(seg.end_y - pad.global_y) < tolerance)):
                    pad_has_segment = True
                    break

            if not pad_has_segment:
                continue

            # Check if via already exists at this pad
            via_exists = False
            for via in pcb_data.vias:
                if (via.net_id == net_id and
                    abs(via.x - pad.global_x) < tolerance and
                    abs(via.y - pad.global_y) < tolerance):
                    via_exists = True
                    break

            if not via_exists:
                vias_to_add.append({
                    'x': pad.global_x,
                    'y': pad.global_y,
                    'size': args.via_size,
                    'drill': args.via_drill,
                    'layers': ['F.Cu', 'B.Cu'],
                    'net_id': net_id,
                    'net_name': matching_nets[net_id]
                })

    # Group by layer for summary
    layers_changed = {}
    for mod in segment_mods:
        old_layer = mod['old_layer']
        layers_changed[old_layer] = layers_changed.get(old_layer, 0) + 1

    print(f"\nSegments to change: {len(segment_mods)}")
    for layer, count in sorted(layers_changed.items()):
        print(f"  {layer} -> {args.to_layer}: {count} segments")

    if vias_to_add:
        print(f"\nVias to add at pad positions: {len(vias_to_add)}")
        for via in vias_to_add:
            print(f"  ({via['x']:.3f}, {via['y']:.3f}) for {via['net_name']}")

    if args.dry_run:
        print("\nDry run - no changes written")
        return 0

    # Read input file
    with open(args.input, 'r', encoding='utf-8') as f:
        content = f.read()

    # Modify segment layers
    modified_content, count = modify_segment_layers(content, segment_mods)

    if count != len(segment_mods):
        print(f"Warning: Expected to modify {len(segment_mods)} segments, but modified {count}")

    # Add vias if needed
    if vias_to_add:
        via_sexprs = []
        for via in vias_to_add:
            via_sexpr = generate_via_sexpr(
                via['x'], via['y'],
                via['size'], via['drill'],
                via['layers'], via['net_id']
            )
            via_sexprs.append(via_sexpr)

        # Insert vias before the final closing paren
        last_paren = modified_content.rfind(')')
        if last_paren != -1:
            via_text = '\n'.join(via_sexprs)
            modified_content = (modified_content[:last_paren] + '\n' +
                                via_text + '\n' + modified_content[last_paren:])

    # Write output file
    with open(args.output, 'w', encoding='utf-8') as f:
        f.write(modified_content)

    via_msg = f", {len(vias_to_add)} vias added" if vias_to_add else ""
    print(f"\nWrote {args.output} with {count} segments changed to {args.to_layer}{via_msg}")
    return 0


if __name__ == '__main__':
    exit(main())

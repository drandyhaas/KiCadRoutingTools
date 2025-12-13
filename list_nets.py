#!/usr/bin/env python3
"""
List all net names on a given component, sorted alphabetically.
"""

import argparse
from kicad_parser import parse_kicad_pcb, find_components_by_type


def main():
    parser = argparse.ArgumentParser(description='List net names on a component')
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--component', '-c', help='Component reference (auto-detected if not specified)')

    args = parser.parse_args()

    pcb_data = parse_kicad_pcb(args.pcb)

    # Auto-detect BGA component if not specified
    if args.component is None:
        bga_components = find_components_by_type(pcb_data, 'BGA')
        if bga_components:
            args.component = bga_components[0].reference
            print(f"Auto-detected BGA component: {args.component}")
        else:
            print("Error: No BGA components found. Please specify --component")
            print(f"Available: {list(pcb_data.footprints.keys())[:20]}...")
            return 1

    if args.component not in pcb_data.footprints:
        print(f"Error: Component {args.component} not found")
        print(f"Available: {list(pcb_data.footprints.keys())}")
        return 1

    footprint = pcb_data.footprints[args.component]

    # Collect unique net names
    nets = set()
    for pad in footprint.pads:
        if pad.net_name and pad.net_id > 0:
            nets.add(pad.net_name)

    # Print sorted
    print(f"\nNets on {args.component} ({len(nets)} total):\n")
    for net in sorted(nets):
        print(f"  {net}")

    return 0


if __name__ == '__main__':
    exit(main())

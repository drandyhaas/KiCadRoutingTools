#!/usr/bin/env python3
"""
List and analyze nets in a KiCad PCB file.

Examples:
    # List nets on a component
    python list_nets.py board.kicad_pcb --component U1

    # Show pad-to-net assignments
    python list_nets.py board.kicad_pcb --component U1 --pads

    # Detect differential pairs in the board
    python list_nets.py board.kicad_pcb --diff-pairs

    # Show power/ground nets with pad counts
    python list_nets.py board.kicad_pcb --power

    # Full board analysis
    python list_nets.py board.kicad_pcb --diff-pairs --power
"""

import argparse
from fnmatch import fnmatch
from kicad_parser import parse_kicad_pcb, find_components_by_type


def find_differential_pairs(pcb_data):
    """Find differential pairs based on common naming conventions."""
    net_names = [n.name for n in pcb_data.nets.values() if n.name]

    diff_patterns = [
        ('_P', '_N'),      # USB, PCIe, generic
        ('_p', '_n'),      # lowercase variant
        ('+', '-'),        # Some designs
        ('_DP', '_DN'),    # USB data
        ('_D+', '_D-'),    # USB alternate
        ('_TX+', '_TX-'),  # Ethernet TX
        ('_RX+', '_RX-'),  # Ethernet RX
        ('_TXP', '_TXN'),  # High-speed serial
        ('_RXP', '_RXN'),  # High-speed serial
        ('_t', '_c'),      # DDR DQS (true/complement)
        ('_T', '_C'),      # DDR DQS uppercase
    ]

    found_pairs = []
    used_nets = set()

    for name in sorted(net_names):
        if name in used_nets:
            continue
        for pos, neg in diff_patterns:
            if name.endswith(pos):
                base = name[:-len(pos)]
                pair_name = base + neg
                if pair_name in net_names and pair_name not in used_nets:
                    found_pairs.append((name, pair_name))
                    used_nets.add(name)
                    used_nets.add(pair_name)
                    break

    return found_pairs


def find_power_nets(pcb_data):
    """Find power and ground nets by name patterns and connection count."""
    gnd_patterns = ['GND', 'VSS', 'AGND', 'DGND', 'PGND', 'GNDA', 'GNDD']
    vcc_patterns = ['VCC', 'VDD', '+3.3', '+5', '+12', '+1.8', '+2.5', 'VBUS', 'VBAT', 'VIN']

    gnd_nets = []
    vcc_nets = []

    for net in pcb_data.nets.values():
        if not net.name:
            continue
        name_upper = net.name.upper()
        pad_count = len(net.pads)

        if any(g in name_upper for g in gnd_patterns):
            gnd_nets.append((net.name, pad_count))
        elif any(v in name_upper for v in vcc_patterns) or (net.name.startswith('+') and any(c.isdigit() for c in net.name)):
            vcc_nets.append((net.name, pad_count))

    # Sort by pad count descending
    gnd_nets.sort(key=lambda x: -x[1])
    vcc_nets.sort(key=lambda x: -x[1])

    return gnd_nets, vcc_nets


def find_high_connection_nets(pcb_data, top_n=10):
    """Find nets with the most connections (often power/ground)."""
    nets_by_count = []
    for net in pcb_data.nets.values():
        if net.name and not net.name.startswith('unconnected'):
            nets_by_count.append((net.name, len(net.pads)))

    nets_by_count.sort(key=lambda x: -x[1])
    return nets_by_count[:top_n]


def main():
    parser = argparse.ArgumentParser(
        description='List and analyze nets in a KiCad PCB file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('pcb', help='Input PCB file')
    parser.add_argument('--component', '-c', help='Component reference (e.g., U1)')
    parser.add_argument('--pads', action='store_true', help='Show pad-to-net assignments')
    parser.add_argument('--diff-pairs', '-d', action='store_true', help='Detect differential pairs')
    parser.add_argument('--power', '-p', action='store_true', help='Show power/ground nets')
    parser.add_argument('--top', '-t', type=int, default=10, help='Show top N most-connected nets (default: 10)')
    parser.add_argument('--pattern', help='Filter nets by glob pattern')

    args = parser.parse_args()

    pcb_data = parse_kicad_pcb(args.pcb)

    # If no specific analysis requested and no component, show help
    if not args.component and not args.diff_pairs and not args.power:
        # Default: show board summary and top connected nets
        print(f"\nBoard: {args.pcb}")
        print(f"Total nets: {len(pcb_data.nets)}")
        print(f"Total components: {len(pcb_data.footprints)}")
        print(f"\nTop {args.top} most-connected nets:")
        for name, count in find_high_connection_nets(pcb_data, args.top):
            print(f"  {name}: {count} pads")
        print(f"\nUse --component/-c to list nets on a component")
        print(f"Use --diff-pairs/-d to detect differential pairs")
        print(f"Use --power/-p to show power/ground nets")
        return 0

    # Differential pair detection
    if args.diff_pairs:
        pairs = find_differential_pairs(pcb_data)
        print(f"\nDifferential Pairs ({len(pairs)} found):")
        if pairs:
            for p, n in pairs:
                print(f"  {p}  /  {n}")
        else:
            print("  None detected")
        print()

    # Power net detection
    if args.power:
        gnd_nets, vcc_nets = find_power_nets(pcb_data)
        print(f"\nGround Nets ({len(gnd_nets)} found):")
        for name, count in gnd_nets:
            print(f"  {name}: {count} pads")
        print(f"\nPower Nets ({len(vcc_nets)} found):")
        for name, count in vcc_nets:
            print(f"  {name}: {count} pads")
        print()

    # Component-specific listing
    if args.component:
        # Auto-detect BGA component if 'auto' specified
        if args.component.lower() == 'auto':
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
            print(f"Available: {sorted(pcb_data.footprints.keys())}")
            return 1

        footprint = pcb_data.footprints[args.component]

        if args.pads:
            # Show pad-to-net assignments
            print(f"\nPads on {args.component} ({len(footprint.pads)} pads):\n")
            pads_sorted = sorted(footprint.pads, key=lambda p: (
                # Sort by pad number (handle alphanumeric like A1, B2)
                ''.join(c for c in p.pad_number if c.isalpha()),
                int(''.join(c for c in p.pad_number if c.isdigit()) or '0')
            ))
            for pad in pads_sorted:
                net_name = pad.net_name if pad.net_name else "(no net)"
                if args.pattern and not fnmatch(net_name, args.pattern):
                    continue
                print(f"  {pad.pad_number}: {net_name}")
        else:
            # Collect unique net names
            nets = set()
            for pad in footprint.pads:
                if pad.net_name and pad.net_id > 0:
                    if args.pattern and not fnmatch(pad.net_name, args.pattern):
                        continue
                    nets.add(pad.net_name)

            # Print sorted
            print(f"\nNets on {args.component} ({len(nets)} total):\n")
            for net in sorted(nets):
                print(f"  {net}")

    return 0


if __name__ == '__main__':
    exit(main())

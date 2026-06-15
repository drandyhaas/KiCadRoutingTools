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
import json
import os
import re
from fnmatch import fnmatch
from kicad_parser import parse_kicad_pcb, find_components_by_type


# KiCad netclass field -> the routing-CLI flag that consumes it.
_NETCLASS_FIELDS = [
    ('clearance', 'clearance', '--clearance'),
    ('track_width', 'track_width', '--track-width'),
    ('via_diameter', 'via_size', '--via-size'),
    ('via_drill', 'via_drill', '--via-drill'),
    ('diff_pair_gap', 'diff_pair_gap', '--diff-pair-gap (route_diff.py)'),
    ('diff_pair_width', 'diff_pair_width', '--track-width (route_diff.py)'),
]


def read_design_rules(pcb_path):
    """Read net-class design rules so a skill can pass them to the routing CLIs.

    Net classes live in the sibling .kicad_pro (KiCad 8+, JSON under
    net_settings.classes) or, for KiCad 6/7, in (net_class ...) blocks inside
    the .kicad_pcb. The router itself does not read these - it defaults to
    routing_defaults.CLEARANCE etc. - so the analysis step should read them
    here and pass --clearance/--track-width/--via-size/--via-drill (and the
    diff-pair gap) explicitly, instead of over-/under-clearancing the board.

    Returns {'classes': {name: {clearance, track_width, via_diameter,
    via_drill, diff_pair_gap, diff_pair_width}}, 'assignments': {net: class},
    'source': str}. Missing values are omitted per class.
    """
    classes = {}
    assignments = {}
    source = None

    pro_path = os.path.splitext(pcb_path)[0] + '.kicad_pro'
    if os.path.exists(pro_path):
        try:
            with open(pro_path, encoding='utf-8') as f:
                pro = json.load(f)
            ns = pro.get('net_settings', {}) or {}
            for c in ns.get('classes', []) or []:
                name = c.get('name', 'Default')
                classes[name] = {k: c[k] for k in
                                 ('clearance', 'track_width', 'via_diameter',
                                  'via_drill', 'diff_pair_gap', 'diff_pair_width')
                                 if k in c}
            # net -> class assignment (dict in newer files; list of pairs in some)
            na = ns.get('netclass_assignments') or {}
            if isinstance(na, dict):
                assignments = dict(na)
            if classes:
                source = pro_path
        except (json.JSONDecodeError, OSError):
            pass

    if not classes:
        # KiCad 6/7 fallback: (net_class "Name" (clearance X) (trace_width Y) ...)
        try:
            with open(pcb_path, encoding='utf-8') as f:
                content = f.read()
        except OSError:
            content = ''
        # KiCad 6/7: (net_class <name|"name"> "description" (clearance ..)
        # (trace_width ..) (via_dia ..) (via_drill ..) (add_net <name>) ...)
        for m in re.finditer(r'\(net_class\s+(?:"([^"]+)"|([^\s"]+))(.*?)\n\s*\)',
                             content, re.DOTALL):
            name = m.group(1) or m.group(2)
            body = m.group(3)
            cls = {}
            for src_key, key in (('clearance', 'clearance'), ('trace_width', 'track_width'),
                                 ('via_dia', 'via_diameter'), ('via_drill', 'via_drill'),
                                 ('diff_pair_gap', 'diff_pair_gap'),
                                 ('diff_pair_width', 'diff_pair_width')):
                vm = re.search(r'\(' + src_key + r'\s+([\d.]+)\)', body)
                if vm:
                    cls[key] = float(vm.group(1))
            classes[name] = cls
            for am in re.finditer(r'\(add_net\s+(?:"([^"]+)"|([^\s")]+))\)', body):
                assignments[am.group(1) or am.group(2)] = name
            source = pcb_path

    return {'classes': classes, 'assignments': assignments, 'source': source}


def print_design_rules(pcb_path):
    """Human + skill-friendly dump of the board's net-class design rules."""
    dr = read_design_rules(pcb_path)
    if not dr['classes']:
        print("No net classes found (no sibling .kicad_pro and no (net_class) "
              "in the PCB). Use routing defaults or pass values explicitly.")
        return
    print(f"\nNet-class design rules (from {os.path.basename(dr['source'])}):")
    for name, c in sorted(dr['classes'].items()):
        fields = "  ".join(f"{k}={c[k]}" for k in
                           ('clearance', 'track_width', 'via_diameter', 'via_drill',
                            'diff_pair_gap', 'diff_pair_width') if k in c)
        print(f"  [{name}] {fields}")
    non_default = {n: c for n, c in dr['assignments'].items() if c != 'Default'}
    if non_default:
        print(f"  ({len(non_default)} net(s) assigned to non-default classes — "
              f"route those separately with that class's flags)")
    # Ready-to-paste CLI flags from the Default class (router consumes these).
    d = dr['classes'].get('Default') or next(iter(dr['classes'].values()))
    flags = []
    if 'clearance' in d:        flags.append(f"--clearance {d['clearance']}")
    if 'track_width' in d:      flags.append(f"--track-width {d['track_width']}")
    if 'via_diameter' in d:     flags.append(f"--via-size {d['via_diameter']}")
    if 'via_drill' in d:        flags.append(f"--via-drill {d['via_drill']}")
    if flags:
        print("\nSUGGESTED route.py/qfn_fanout/bga_fanout/route_planes flags "
              "(Default class):\n  " + " ".join(flags))
    if 'diff_pair_gap' in d or 'diff_pair_width' in d:
        dp = []
        if 'diff_pair_width' in d: dp.append(f"--track-width {d['diff_pair_width']}")
        if 'diff_pair_gap' in d:   dp.append(f"--diff-pair-gap {d['diff_pair_gap']}")
        print("SUGGESTED route_diff.py flags (Default class):\n  " + " ".join(dp))


def find_differential_pairs(pcb_data):
    """Find differential pairs based on common naming conventions.

    Delegates to net_queries.extract_diff_pair_base so this report and the
    route_diff/fanout engines recognize the exact same conventions (issue #91:
    DDR _t/_c case + no-separator channels, USB DP/DM and DPLUS/DMINUS).
    """
    from net_queries import extract_diff_pair_base

    net_names = [n.name for n in pcb_data.nets.values() if n.name]

    # Key by (base, style) so a net only pairs within its own convention.
    halves = {}  # (base, style) -> {True: pos_name, False: neg_name}
    for name in net_names:
        result = extract_diff_pair_base(name)
        if result is None:
            continue
        base, is_pos, style = result
        halves.setdefault((base, style), {})[is_pos] = name

    found_pairs = []
    for (base, style), sides in halves.items():
        if True in sides and False in sides:
            found_pairs.append((sides[True], sides[False]))

    found_pairs.sort()
    return found_pairs


def find_power_nets(pcb_data):
    """Find power and ground nets by name patterns and connection count.

    Returns (gnd_nets, vcc_nets, candidate_nets) - candidates are unmatched
    nets whose pad count rivals the detected power rails (issue #91: rails
    like '-12V', '/5V', '/3V3', and bare 'VDC' defeat prefix patterns; pad
    count catches them).
    """
    import re
    gnd_patterns = ['GND', 'VSS', 'AGND', 'DGND', 'PGND', 'GNDA', 'GNDD']
    vcc_patterns = ['VCC', 'VDD', '+3.3', '+5', '+12', '+1.8', '+2.5', 'VBUS', 'VBAT', 'VIN']
    # Rail-shaped names, checked against the LAST hierarchical path component:
    # +5V, -12V, 5V, 3V3, 1V8, 12V0, 3.3V ...
    rail_re = re.compile(r'^[+-]?(\d+(\.\d+)?V\d*|\d+V\d+)$')

    gnd_nets = []
    vcc_nets = []
    matched = set()

    for net in pcb_data.nets.values():
        if not net.name:
            continue
        name_upper = net.name.upper()
        leaf = name_upper.rsplit('/', 1)[-1]
        pad_count = len(net.pads)

        if any(g in name_upper for g in gnd_patterns):
            gnd_nets.append((net.name, pad_count))
            matched.add(net.name)
        elif (any(v in name_upper for v in vcc_patterns)
              or (net.name[0] in '+-' and any(c.isdigit() for c in net.name))
              or rail_re.match(leaf)):
            vcc_nets.append((net.name, pad_count))
            matched.add(net.name)

    # Sort by pad count descending
    gnd_nets.sort(key=lambda x: -x[1])
    vcc_nets.sort(key=lambda x: -x[1])

    # High pad-count nets the patterns missed: anything rivaling the known
    # POWER rails is probably a power net with an unconventional name (e.g.
    # VDC). Threshold from the largest vcc rail, not GND - GND's pad count
    # dwarfs everything and would hide real rails.
    vcc_counts = [c for _, c in vcc_nets]
    threshold = max(10, max(vcc_counts) // 2) if vcc_counts else 10
    candidates = sorted(
        ((net.name, len(net.pads)) for net in pcb_data.nets.values()
         if net.name and net.name not in matched
         and not net.name.lower().startswith('unconnected')
         and len(net.pads) >= threshold),
        key=lambda x: -x[1])[:8]

    return gnd_nets, vcc_nets, candidates


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
    parser.add_argument('--design-rules', '-r', action='store_true',
                        help="Show net-class design rules (clearance/track/via/diff-pair) "
                             "from the .kicad_pro (or .kicad_pcb net_class) and the CLI "
                             "flags to pass them to the routing tools")
    parser.add_argument('--top', '-t', type=int, default=10, help='Show top N most-connected nets (default: 10)')
    parser.add_argument('--pattern', help='Filter nets by glob pattern')

    args = parser.parse_args()

    # Design rules read from project metadata, not the parsed PCB object.
    if args.design_rules:
        print_design_rules(args.pcb)
        if not (args.component or args.diff_pairs or args.power):
            return

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
        gnd_nets, vcc_nets, candidates = find_power_nets(pcb_data)
        print(f"\nGround Nets ({len(gnd_nets)} found):")
        for name, count in gnd_nets:
            print(f"  {name}: {count} pads")
        print(f"\nPower Nets ({len(vcc_nets)} found):")
        for name, count in vcc_nets:
            print(f"  {name}: {count} pads")
        if candidates:
            print(f"\nHigh pad-count nets NOT matched by power patterns "
                  f"(possible power rails - verify):")
            for name, count in candidates:
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

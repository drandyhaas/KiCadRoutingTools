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

# Board Constraints (KiCad: Board Setup -> Constraints) live in the .kicad_pro
# under board.design_settings.rules. Unlike the net-class *defaults*, THESE are
# what DRC actually enforces for the geometric rules: a net-class track_width /
# via_diameter is only the size new objects are drawn at, never a DRC minimum --
# only these min_* values are. So the router may emit a via/track SMALLER than
# the net-class nominal (down to these) and the board still passes DRC. (#111/#115)
_CONSTRAINT_FIELDS = ('min_clearance', 'min_track_width', 'min_via_diameter',
                      'min_via_annular_width', 'min_hole_to_hole',
                      'min_through_hole_diameter')

# JLCPCB standard (no-extra-charge) manufacturing floors, by copper-layer count.
# Used as the BACKSTOP when a board leaves a Constraint at 0/unset (common --
# e.g. min_clearance is frequently 0). The router never goes below these, and
# DRC/connectivity are graded at them. Source: jlcpcb.com/capabilities (2026-06).
# 'fine_via_*' is the smaller JLC "advanced" (small extra-cost) via, available
# on 4+ layer boards, for FINE-PITCH ESCAPE only (sub-~0.5mm BGA/QFN balls the
# 0.45mm standard via can't via-in-pad / dog-bone between). Not the default --
# general routing uses the standard via. On 2-layer it equals the standard via
# (advanced small vias are a multilayer option).
# 'track_width' is the fab PHYSICAL minimum trace width, and it is LAYER-DEPENDENT:
# JLCPCB allows 3.5 mil (0.0889 mm) only on 4+ layer boards; 2-layer is limited to
# 5 mil (0.127 mm). It is deliberately BELOW the board's own min_track_width
# constraint, which is usually conservative (human originals route dense channels
# at the fab minimum, e.g. ottercast 0.127 mm under its 0.2 mm board rule). Dense/
# congested signals route down to this (see list_nets --design-rules "fine-pitch
# signal track floor"); power/impedance nets keep their net-class width.
_FAB_FLOORS = {
    2: {'clearance': 0.127, 'track_width': 0.127, 'via_diameter': 0.45,
        'via_drill': 0.20, 'hole_to_hole': 0.25, 'annular': 0.13,
        'fine_via_diameter': 0.45, 'fine_via_drill': 0.20},
    4: {'clearance': 0.10, 'track_width': 0.0889, 'via_diameter': 0.45,
        'via_drill': 0.20, 'hole_to_hole': 0.25, 'annular': 0.13,
        'fine_via_diameter': 0.30, 'fine_via_drill': 0.15},
}


def fab_floors(copper_layer_count):
    """JLCPCB manufacturing floors for the given copper-layer count (2 vs 4+)."""
    return _FAB_FLOORS[2] if (copper_layer_count or 2) <= 2 else _FAB_FLOORS[4]


def _count_copper_layers(content):
    """Count copper layers from a .kicad_pcb's (layers ...) definitions.

    Matches layer-def lines like `(0 "F.Cu" signal)` / `(4 "In1(GND).Cu" ...)`
    but not pad `(layers "F.Cu" ...)` lists (those have no leading layer index).
    """
    return len(re.findall(r'\(\d+\s+"[^"]*\.Cu"', content)) or 0


def effective_floors(constraints, copper_layers):
    """Combine the board's own Constraints with the JLC fab floor.

    Returns the values the tools should actually use:
      - working_via_diameter / working_via_drill: the SMALL escape/working via to
        emit for fanout & routing, instead of the (larger) net-class nominal (#115).
        These are GEOMETRY constraints (the board's min via, reliably the size the
        human's vias actually are), floored at the fab minimum.
      - drc_clearance / drc_hole_to_hole: the manufacturability floor to grade DRC
        and connectivity against, instead of the inflated net-class clearance (#111).
        These are the fab SPACING floor directly: fine-pitch escapes are routed down
        to it, so DRC must check at it to pass them; and the board's own
        `min_clearance` is an unreliable edit-floor (often 0, sometimes a stale large
        value that would re-flood the DRC), so it is deliberately NOT used here.
    """
    fab = fab_floors(copper_layers)

    def floor(constraint_val, fab_val):
        return max(constraint_val or 0.0, fab_val)

    return {
        'working_via_diameter': floor(constraints.get('min_via_diameter'), fab['via_diameter']),
        'working_via_drill':    floor(constraints.get('min_through_hole_diameter'), fab['via_drill']),
        # Advanced small via for fine-pitch escape only (4+ layer). The fab's
        # smallest, not floored by the board's nominal min via.
        'fine_via_diameter':    fab['fine_via_diameter'],
        'fine_via_drill':       fab['fine_via_drill'],
        # DRC-enforced track floor: the board's own min_track_width if set, else the
        # fab minimum. min_track_width IS a DRC rule, so this is what DRC checks.
        'min_track_width':      floor(constraints.get('min_track_width'), fab['track_width']),
        # Fab PHYSICAL track minimum, NOT clamped to the board constraint. Dense/
        # congested signals route down to this (below the board's min_track_width,
        # like the human originals); it's the routing-capability floor, distinct
        # from the DRC floor above.
        'fab_track_width':      fab['track_width'],
        'drc_clearance':        fab['clearance'],
        'drc_hole_to_hole':     fab['hole_to_hole'],
        'fab': fab,
    }


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
    'constraints': {min_clearance, min_track_width, min_via_diameter, ...},
    'copper_layers': int, 'effective': {...}, 'source': str}. Missing values
    are omitted per class. 'constraints' are the DRC-enforced Board Setup minima;
    'effective' combines them with the JLC fab floor (see effective_floors()).
    """
    classes = {}
    assignments = {}
    constraints = {}
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
            # DRC-enforced Board Constraints (what humans actually route against).
            rules = ((pro.get('board', {}) or {}).get('design_settings', {}) or {}).get('rules', {}) or {}
            constraints = {k: rules[k] for k in _CONSTRAINT_FIELDS if k in rules}
            if classes or constraints:
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

    # Copper-layer count (for the fab-floor backstop) from the .kicad_pcb.
    copper_layers = 0
    try:
        with open(pcb_path, encoding='utf-8') as f:
            copper_layers = _count_copper_layers(f.read())
    except OSError:
        pass

    return {'classes': classes, 'assignments': assignments,
            'constraints': constraints, 'copper_layers': copper_layers,
            'effective': effective_floors(constraints, copper_layers),
            'source': source}


def print_design_rules(pcb_path):
    """Human + skill-friendly dump of the board's net-class design rules."""
    dr = read_design_rules(pcb_path)
    eff = dr['effective']
    if not dr['classes'] and not dr['constraints']:
        print("No net classes or Board Constraints found (no sibling .kicad_pro "
              "and no (net_class) in the PCB).")
        print(f"Falling back to JLCPCB fab floors for {dr['copper_layers'] or '?'}-"
              "layer board — route/DRC at these:")
        print(f"  working via {eff['working_via_diameter']}/{eff['working_via_drill']}  "
              f"clearance {eff['drc_clearance']}  hole-to-hole {eff['drc_hole_to_hole']}")
        return
    src = os.path.basename(dr['source']) if dr['source'] else '?'
    print(f"\nNet-class design rules (from {src}):")
    for name, c in sorted(dr['classes'].items()):
        fields = "  ".join(f"{k}={c[k]}" for k in
                           ('clearance', 'track_width', 'via_diameter', 'via_drill',
                            'diff_pair_gap', 'diff_pair_width') if k in c)
        print(f"  [{name}] {fields}")
    non_default = {n: c for n, c in dr['assignments'].items() if c != 'Default'}
    if non_default:
        print(f"  ({len(non_default)} net(s) assigned to non-default classes — "
              f"route those separately with that class's flags)")

    # The DRC-enforced Board Constraints. Net-class track_width/via_diameter are
    # only drawing DEFAULTS; ONLY these min_* are DRC floors (#111/#115).
    if dr['constraints']:
        cons = "  ".join(f"{k}={v}" for k, v in dr['constraints'].items())
        print(f"\nBoard Constraints (DRC-enforced minima, {dr['copper_layers']}-layer): {cons}")
    print(f"Manufacturing floor (Constraint or JLC fab min, whichever is larger): "
          f"via {eff['working_via_diameter']}/{eff['working_via_drill']}  "
          f"clearance {eff['drc_clearance']}  hole-to-hole {eff['drc_hole_to_hole']}  "
          f"track {eff['min_track_width']} (DRC track floor = board min_track_width)")
    # The router must honour these as DISTINCT rules (issue #125):
    print(f"  - hole-to-hole {eff['drc_hole_to_hole']} = drill-to-drill minimum, "
          "net-INDEPENDENT: applies to via/via, via/pad-drill and pad-drill/pad-drill "
          "on ALL nets, INCLUDING same-net.")
    print(f"  - copper clearance {eff['drc_clearance']} = via/pad and via/via copper, "
          "between DIFFERENT nets. Same-net via-pad copper clearance is 0 "
          "(via-in-pad) where the fab allows it; hole-to-hole still applies.")
    # Advanced small via for fine-pitch escape (4+ layer). Standard via can't
    # dog-bone / via-in-pad sub-0.5mm BGA/QFN balls (issues #99/#122).
    if eff['fine_via_diameter'] < eff['fab']['via_diameter']:
        print(f"  - fine-pitch escape via {eff['fine_via_diameter']}/{eff['fine_via_drill']} "
              "(JLC advanced, small extra cost): use ONLY for sub-~0.5mm-pitch "
              "BGA/QFN escape (bga_fanout/qfn_fanout/route_diff for those parts); "
              "keep the standard via for general routing.")
    # Fab PHYSICAL track floor, distinct from the DRC track floor above. Surfaced
    # only when the fab can go thinner than the board's min_track_width — that gap
    # is where dense-board completion is won (the human originals route there).
    if eff['fab_track_width'] < eff['min_track_width']:
        print(f"  - fine-pitch signal track floor {eff['fab_track_width']} "
              f"({dr['copper_layers']}-layer fab min; the board's min_track_width is "
              f"{eff['min_track_width']}): on DENSE/congested boards route ordinary "
              "signals at this width (route.py --track-width), BELOW the board's own "
              "rule, like the human originals — thinner is more complete AND faster. "
              "Keep power nets on --power-nets and impedance nets at net-class width.")

    # Routing flags: track_width is a per-class MINIMUM (keep, for current/
    # impedance); clearance is the per-class default. But the VIA uses the small
    # working floor, NOT the net-class via_diameter (which is just a max-like
    # default) -- emitting the nominal everywhere is #115.
    d = dr['classes'].get('Default') or (next(iter(dr['classes'].values())) if dr['classes'] else {})
    flags = []
    if 'clearance' in d:   flags.append(f"--clearance {d['clearance']}")
    if 'track_width' in d: flags.append(f"--track-width {d['track_width']}")
    flags.append(f"--via-size {eff['working_via_diameter']}")
    flags.append(f"--via-drill {eff['working_via_drill']}")
    print("\nSUGGESTED route.py/qfn_fanout/bga_fanout/route_planes flags "
          "(Default class; small working via):\n  " + " ".join(flags))
    print("  Fine-pitch escape may drop --clearance toward the manufacturing floor "
          f"{eff['drc_clearance']} (never below); route non-Default-class nets separately.")
    if eff['fab_track_width'] < eff['min_track_width']:
        print(f"  On dense/congested boards, drop --track-width to the fab floor "
              f"{eff['fab_track_width']} for ordinary signals (keep --power-nets and "
              "impedance nets wide); thinner completes more nets and routes faster.")

    if 'diff_pair_gap' in d or 'diff_pair_width' in d:
        dp = []
        if 'diff_pair_width' in d: dp.append(f"--track-width {d['diff_pair_width']}")
        if 'diff_pair_gap' in d:   dp.append(f"--diff-pair-gap {d['diff_pair_gap']}")
        print("SUGGESTED route_diff.py flags (Default class):\n  " + " ".join(dp))

    # Verification: DRC/connectivity at the manufacturability floor -- the rule
    # the human original passes -- NOT the inflated net-class clearance (#111).
    print(f"\nSUGGESTED check_drc.py flags (grade at the manufacturing floor):\n  "
          f"--clearance {eff['drc_clearance']} "
          f"--hole-to-hole-clearance {eff['drc_hole_to_hole']}")


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

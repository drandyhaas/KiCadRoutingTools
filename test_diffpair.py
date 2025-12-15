#!/usr/bin/env python3
"""
Test script for differential pair routing.

Builds the Rust router, routes test differential pairs,
and runs DRC to check for violations.

Supports wildcards for diff pair selection:
  - ? matches any single character
  - * matches any sequence of characters

Examples:
  python test_diffpair.py lvds_rx4_1           # Route single diff pair
  python test_diffpair.py lvds_rx1_?           # Route lvds_rx1_0 through lvds_rx1_9
  python test_diffpair.py lvds_rx*_9           # Route all rx channel 9s
  python test_diffpair.py 'lvds_rx1_*' 'lvds_rx2_*'  # Route multiple patterns
  python test_diffpair.py lvds_rx2_? --stub-proximity-radius 2.0
"""

import subprocess
import sys
import os
import argparse
import fnmatch
import re
from typing import List, Set, Tuple, Optional


def run_command(cmd, description, capture_output=False):
    """Run a command and print its output."""
    print(f"\n{'='*60}")
    print(f"{description}")
    print(f"{'='*60}")
    print(f"Running: {' '.join(cmd)}\n")

    result = subprocess.run(cmd, capture_output=capture_output, text=True)
    if capture_output and result.stdout:
        print(result.stdout)
    if capture_output and result.stderr:
        print(result.stderr, file=sys.stderr)
    return result


def extract_diff_pair_base(net_name: str) -> Optional[Tuple[str, bool]]:
    """
    Extract differential pair base name and polarity from net name.
    Returns (base_name, is_positive) or None if not a diff pair.
    """
    if not net_name:
        return None

    # Try _P/_N suffix (most common for LVDS)
    if net_name.endswith('_P'):
        return (net_name[:-2], True)
    if net_name.endswith('_N'):
        return (net_name[:-2], False)

    # Try P/N suffix without underscore
    if net_name.endswith('P') and len(net_name) > 1:
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], True)
    if net_name.endswith('N') and len(net_name) > 1:
        if net_name[-2] in '0123456789_':
            return (net_name[:-1], False)

    # Try +/- suffix
    if net_name.endswith('+'):
        return (net_name[:-1], True)
    if net_name.endswith('-'):
        return (net_name[:-1], False)

    return None


def find_all_diff_pairs_in_pcb(pcb_file: str) -> Set[str]:
    """
    Parse PCB file to find all differential pair base names.
    Returns set of base names (e.g., {'lvds_rx4_1', 'lvds_rx3_10', ...})
    """
    diff_pairs = set()
    p_nets = set()
    n_nets = set()

    # Parse net definitions from PCB file
    net_pattern = re.compile(r'\(net\s+(\d+)\s+"([^"]+)"\)')

    with open(pcb_file, 'r', encoding='utf-8') as f:
        content = f.read()

    for match in net_pattern.finditer(content):
        net_name = match.group(2)
        result = extract_diff_pair_base(net_name)
        if result:
            base_name, is_p = result
            if is_p:
                p_nets.add(base_name)
            else:
                n_nets.add(base_name)

    # Only include complete pairs (have both P and N)
    diff_pairs = p_nets & n_nets
    return diff_pairs


def match_diff_pairs(patterns: List[str], available_pairs: Set[str]) -> List[str]:
    """
    Match user patterns against available diff pairs using fnmatch.
    Returns sorted list of matching diff pair base names.
    """
    matched = set()
    for pattern in patterns:
        for pair in available_pairs:
            if fnmatch.fnmatch(pair, pattern):
                matched.add(pair)
    return sorted(matched)


def main():
    parser = argparse.ArgumentParser(
        description='Test differential pair routing',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  %(prog)s lvds_rx4_1                    Route single diff pair
  %(prog)s lvds_rx1_?                    Route lvds_rx1_0 through lvds_rx1_9
  %(prog)s lvds_rx*_9                    Route all rx channel 9s
  %(prog)s 'lvds_rx1_*' 'lvds_rx2_*'     Route multiple patterns
  %(prog)s --list                        List all available diff pairs
  %(prog)s lvds_rx2_? --stub-proximity-radius 2.0
'''
    )
    # Test script options
    parser.add_argument('diff_pairs', nargs='*', default=['lvds_rx4_1'],
                        help='Diff pair patterns to route (supports * and ? wildcards). '
                             'Default: lvds_rx4_1')
    parser.add_argument('--build', action='store_true',
                        help='Build the Rust router before running')
    parser.add_argument('--list', action='store_true',
                        help='List all available diff pairs in the PCB and exit')
    parser.add_argument('--dry-run', action='store_true',
                        help='Show which diff pairs would be routed without actually routing')
    parser.add_argument('--skip-drc', action='store_true',
                        help='Skip DRC checks after routing')
    parser.add_argument('--input', default='routed_output.kicad_pcb',
                        help='Input PCB file (default: routed_output.kicad_pcb)')
    parser.add_argument('--output', default='test_batch_diffpair.kicad_pcb',
                        help='Output PCB file (default: test_batch_diffpair.kicad_pcb)')

    # Router options (pass-through to batch_grid_router.py)
    router_group = parser.add_argument_group('Router options (passed to batch_grid_router.py)')
    router_group.add_argument('--ordering', '-o', choices=['inside_out', 'mps', 'original'],
                              help='Net ordering strategy (default: mps)')
    router_group.add_argument('--direction', '-d', choices=['forward', 'backwards', 'random'],
                              help='Direction search order')
    router_group.add_argument('--no-bga-zones', action='store_true', default=True,
                              help='Disable BGA exclusion zones (default: enabled)')
    router_group.add_argument('--bga-zones', action='store_true',
                              help='Enable BGA exclusion zones')
    router_group.add_argument('--layers', '-l', nargs='+',
                              help='Routing layers (default: F.Cu In1.Cu In2.Cu B.Cu)')
    router_group.add_argument('--track-width', type=float,
                              help='Track width in mm (default: 0.1)')
    router_group.add_argument('--clearance', type=float,
                              help='Clearance in mm (default: 0.1)')
    router_group.add_argument('--via-size', type=float,
                              help='Via outer diameter in mm (default: 0.3)')
    router_group.add_argument('--via-drill', type=float,
                              help='Via drill size in mm (default: 0.2)')
    router_group.add_argument('--grid-step', type=float,
                              help='Grid resolution in mm (default: 0.1)')
    router_group.add_argument('--via-cost', type=float,
                              help='Via cost penalty in grid steps (default: 25)')
    router_group.add_argument('--max-iterations', type=int,
                              help='Max A* iterations (default: 100000)')
    router_group.add_argument('--heuristic-weight', type=float,
                              help='A* heuristic weight (default: 1.5)')
    router_group.add_argument('--stub-proximity-radius', type=float,
                              help='Radius around stubs to penalize in mm (default: 1.5)')
    router_group.add_argument('--stub-proximity-cost', type=float,
                              help='Cost penalty near stubs in mm equivalent (default: 2.0)')
    router_group.add_argument('--diff-pair-gap', type=float,
                              help='Gap between P/N traces in mm (default: 0.1)')
    router_group.add_argument('--min-diff-pair-centerline-setback', type=float, default=1.5,
                              help='Min distance in front of stubs to start route in mm (default: 1.5)')
    router_group.add_argument('--max-diff-pair-centerline-setback', type=float, default=3.0,
                              help='Max distance in front of stubs to start route in mm (default: 3.0)')

    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Configuration
    input_pcb = args.input
    output_pcb = args.output
    diff_pair_pattern = "*lvds*"

    # Find all available diff pairs in the PCB
    if not os.path.exists(input_pcb):
        print(f"ERROR: Input PCB file not found: {input_pcb}")
        return 1

    available_pairs = find_all_diff_pairs_in_pcb(input_pcb)

    if args.list:
        print(f"Available differential pairs in {input_pcb}:")
        print(f"{'='*60}")
        for pair in sorted(available_pairs):
            print(f"  {pair}")
        print(f"\nTotal: {len(available_pairs)} diff pairs")
        return 0

    # Match patterns against available pairs
    matched_pairs = match_diff_pairs(args.diff_pairs, available_pairs)

    if not matched_pairs:
        print(f"ERROR: No diff pairs matched the pattern(s): {', '.join(args.diff_pairs)}")
        print(f"\nAvailable diff pairs (showing first 10):")
        for pair in sorted(available_pairs)[:10]:
            print(f"  {pair}")
        if len(available_pairs) > 10:
            print(f"  ... and {len(available_pairs) - 10} more (use --list to see all)")
        return 1

    print(f"Matched {len(matched_pairs)} diff pair(s):")
    for pair in matched_pairs:
        print(f"  {pair}")

    if args.dry_run:
        print("\nDry run - no routing performed")
        return 0

    # Step 1: Build the Rust router (optional)
    if args.build:
        result = run_command(
            [sys.executable, "build_router.py"],
            "Step 1: Building Rust router"
        )
        if result.returncode != 0:
            print("\nERROR: Build failed!")
            return 1

    # Step 2: Route all differential pairs in a single pass
    # Build net patterns for all matched pairs
    net_patterns = [f"*{diff_pair}_*" for diff_pair in matched_pairs]

    # Build router command with pass-through options
    router_cmd = [
        sys.executable, "batch_grid_router.py",
        input_pcb, output_pcb
    ] + net_patterns

    # Add pass-through options
    if args.no_bga_zones and not args.bga_zones:
        router_cmd.append("--no-bga-zones")
    if args.ordering:
        router_cmd.extend(["--ordering", args.ordering])
    if args.direction:
        router_cmd.extend(["--direction", args.direction])
    if args.layers:
        router_cmd.extend(["--layers"] + args.layers)
    if args.track_width is not None:
        router_cmd.extend(["--track-width", str(args.track_width)])
    if args.clearance is not None:
        router_cmd.extend(["--clearance", str(args.clearance)])
    if args.via_size is not None:
        router_cmd.extend(["--via-size", str(args.via_size)])
    if args.via_drill is not None:
        router_cmd.extend(["--via-drill", str(args.via_drill)])
    if args.grid_step is not None:
        router_cmd.extend(["--grid-step", str(args.grid_step)])
    if args.via_cost is not None:
        router_cmd.extend(["--via-cost", str(args.via_cost)])
    if args.max_iterations is not None:
        router_cmd.extend(["--max-iterations", str(args.max_iterations)])
    if args.heuristic_weight is not None:
        router_cmd.extend(["--heuristic-weight", str(args.heuristic_weight)])
    if args.stub_proximity_radius is not None:
        router_cmd.extend(["--stub-proximity-radius", str(args.stub_proximity_radius)])
    if args.stub_proximity_cost is not None:
        router_cmd.extend(["--stub-proximity-cost", str(args.stub_proximity_cost)])
    if args.diff_pair_gap is not None:
        router_cmd.extend(["--diff-pair-gap", str(args.diff_pair_gap)])
    router_cmd.extend(["--min-diff-pair-centerline-setback", str(args.min_diff_pair_centerline_setback)])
    router_cmd.extend(["--max-diff-pair-centerline-setback", str(args.max_diff_pair_centerline_setback)])
    router_cmd.extend(["--diff-pairs", diff_pair_pattern])

    result = run_command(
        router_cmd,
        f"Step 2: Routing {len(matched_pairs)} differential pair(s)",
        capture_output=True
    )
    router_output = result.stdout if result.stdout else ""

    # Parse router output to determine which pairs succeeded/failed routing
    # Router format:
    #   [1/10] Routing diff pair lvds_rx2_0
    #   ...
    #   SUCCESS: X segments, Y vias, Z iterations (T.TTs)
    # or:
    #   FAILED: Could not find route (T.TTs)

    routing_results = {}  # diff_pair -> True (routed) or False (failed)

    for diff_pair in matched_pairs:
        # Find the section for this diff pair
        # Pattern: "Routing diff pair {name}" followed by content until next "Routing diff pair" or "Routing complete"
        section_pattern = rf'Routing diff pair\s+{re.escape(diff_pair)}\b([\s\S]*?)(?=\[\d+/\d+\] Routing|Routing complete|$)'
        section_match = re.search(section_pattern, router_output)

        if section_match:
            section = section_match.group(1)
            # Check for SUCCESS or FAILED in this section
            if re.search(r'^\s*SUCCESS:', section, re.MULTILINE):
                routing_results[diff_pair] = True
            elif re.search(r'^\s*FAILED:', section, re.MULTILINE):
                routing_results[diff_pair] = False
            else:
                # No explicit SUCCESS/FAILED found, check for other indicators
                if 'No route found' in section or 'No valid source' in section or 'No valid target' in section:
                    routing_results[diff_pair] = False
                else:
                    routing_results[diff_pair] = True  # Assume success
        else:
            # Pair not found in output - assume success (DRC will catch issues)
            routing_results[diff_pair] = True

    routed_pairs = [p for p in matched_pairs if routing_results.get(p, True)]
    failed_routing_pairs = [p for p in matched_pairs if not routing_results.get(p, True)]

    # Step 3: Run DRC check only on successfully routed pairs (unless skipped)
    drc_results = {}  # diff_pair -> (passed, error_count)

    if args.skip_drc:
        print("\nSkipping DRC checks (--skip-drc)")
    else:
        for diff_pair in routed_pairs:
            net_pattern = f"*{diff_pair}_*"
            result = run_command(
                [sys.executable, "check_drc.py", output_pcb, "--nets", net_pattern],
                f"Step 3: DRC check for {diff_pair}",
                capture_output=True
            )

            # Parse DRC output to count errors
            output = result.stdout if result.stdout else ""
            error_count = 0

            # Look for violation counts in output
            # Format: "FOUND 8 DRC VIOLATIONS" or "X violations" or "X violation"
            patterns = [
                r'FOUND\s+(\d+)\s+DRC\s+VIOLATION',  # "FOUND 8 DRC VIOLATIONS"
                r'(\d+)\s+DRC\s+violation',           # "8 DRC violations"
                r'(\d+)\s+violation',                 # "8 violations"
            ]

            for pattern in patterns:
                match = re.search(pattern, output, re.IGNORECASE)
                if match:
                    error_count = int(match.group(1))
                    break

            # If no pattern matched but "violation" mentioned (and not "no violations" or "0")
            if error_count == 0 and "violation" in output.lower():
                # Check for explicit "no violations" messages (check_drc.py outputs "NO DRC VIOLATIONS FOUND!")
                no_violations = ("no drc violations" in output.lower() or
                                 "no violation" in output.lower() or
                                 "0 violation" in output.lower())
                if not no_violations:
                    # Count individual violation lines as fallback
                    violation_lines = re.findall(r'violation|error', output, re.IGNORECASE)
                    error_count = len(violation_lines) if violation_lines else 1

            passed = result.returncode == 0 and error_count == 0
            drc_results[diff_pair] = (passed, error_count)

    # Print summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")

    # Routing summary
    print(f"\nRouting:")
    print(f"  Succeeded: {len(routed_pairs)}/{len(matched_pairs)}")

    if routed_pairs:
        print(f"  Routed:")
        for pair in routed_pairs:
            print(f"    {pair}")

    if failed_routing_pairs:
        print(f"  Failed to route:")
        for pair in failed_routing_pairs:
            print(f"    {pair}")

    # DRC summary (only for routed pairs, and only if DRC was run)
    if routed_pairs and not args.skip_drc:
        drc_passed = [p for p, (ok, _) in drc_results.items() if ok]
        drc_failed = [p for p, (ok, _) in drc_results.items() if not ok]

        print(f"\nDRC (of {len(routed_pairs)} routed):")
        print(f"  Passed: {len(drc_passed)}/{len(routed_pairs)}")

        if drc_passed:
            print(f"  OK:")
            for pair in drc_passed:
                print(f"    {pair}")

        if drc_failed:
            print(f"  FAILED:")
            for pair in drc_failed:
                _, error_count = drc_results[pair]
                print(f"    {pair} ({error_count} violation{'s' if error_count != 1 else ''})")
    elif args.skip_drc:
        print(f"\nDRC: skipped")

    print(f"\nOutput file: {output_pcb}")
    print(f"{'='*60}")

    # Return error if any routing failures, or DRC failures (if DRC was run)
    has_failures = bool(failed_routing_pairs)
    if not args.skip_drc:
        has_failures = has_failures or any(not ok for ok, _ in drc_results.values())
    return 1 if has_failures else 0


if __name__ == "__main__":
    sys.exit(main())

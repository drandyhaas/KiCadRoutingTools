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
    """Run a command and print its output.

    When capture_output=True, streams output line-by-line as it's produced
    while also capturing it for later use.
    """
    print(f"\n{'='*60}")
    print(f"{description}")
    print(f"{'='*60}")
    print(f"Running: {' '.join(cmd)}")
    sys.stdout.flush()

    if capture_output:
        # Use Popen to stream output in real-time while capturing
        # Set PYTHONUNBUFFERED to force the child process to flush output immediately
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Merge stderr into stdout for unified streaming
            text=True,
            bufsize=1,  # Line buffered
            env=env
        )

        stdout_lines = []
        while True:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            if line:
                print(line, end='', flush=True)
                stdout_lines.append(line)

        process.wait()

        # Create a result-like object
        class Result:
            def __init__(self, returncode, stdout):
                self.returncode = returncode
                self.stdout = stdout
                self.stderr = None

        return Result(process.returncode, ''.join(stdout_lines))
    else:
        return subprocess.run(cmd, text=True)


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
    parser.add_argument('--skip-connectivity', action='store_true',
                        help='Skip connectivity checks after routing')
    parser.add_argument('--input', default='routed_output.kicad_pcb',
                        help='Input PCB file (default: routed_output.kicad_pcb)')
    parser.add_argument('--output', default='test_diffpair.kicad_pcb',
                        help='Output PCB file (default: test_diffpair.kicad_pcb)')

    # Router options (pass-through to route.py)
    router_group = parser.add_argument_group('Router options (passed to route.py)')
    router_group.add_argument('--ordering', '-o', choices=['inside_out', 'mps', 'original'],
                              help='Net ordering strategy (default: mps)')
    router_group.add_argument('--direction', '-d', choices=['forward', 'backward', 'random'],
                              help='Direction search order')
    router_group.add_argument('--no-bga-zones', action='store_true',
                              help='Disable BGA exclusion zones')
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
    router_group.add_argument('--via-cost', type=int,
                              help='Via cost penalty in grid steps (default: 25)')
    router_group.add_argument('--max-iterations', type=int,
                              help='Max A* iterations (default: 200000)')
    router_group.add_argument('--max-probe-iterations', type=int,
                              help='Max iterations for quick probe per direction (default: 5000)')
    router_group.add_argument('--heuristic-weight', type=float,
                              help='A* heuristic weight (default: 1.9)')
    router_group.add_argument('--stub-proximity-radius', type=float,
                              help='Radius around stubs to penalize in mm (default: 2.0)')
    router_group.add_argument('--stub-proximity-cost', type=float,
                              help='Cost penalty near stubs in mm equivalent (default: 0.2)')
    router_group.add_argument('--via-proximity-cost', type=float,
                              help='Multiplier on stub-proximity-cost for vias near stubs (0=block, default: 20.0)')
    router_group.add_argument('--bga-proximity-radius', type=float,
                              help='Radius around BGA edges to penalize in mm (default: 10.0)')
    router_group.add_argument('--bga-proximity-cost', type=float,
                              help='Cost penalty near BGA edges in mm equivalent (default: 0.2)')
    router_group.add_argument('--track-proximity-distance', type=float,
                              help='Distance around routed tracks to penalize on same layer in mm (default: 1.0)')
    router_group.add_argument('--track-proximity-cost', type=float,
                              help='Cost penalty near routed tracks in mm equivalent (default: 0.2)')
    router_group.add_argument('--diff-pair-gap', type=float,
                              help='Gap between P/N traces in mm (default: 0.101)')
    router_group.add_argument('--diff-pair-centerline-setback', type=float,
                              help='Distance in front of stubs to start route in mm (default: 2x P-N spacing)')
    router_group.add_argument('--min-turning-radius', type=float,
                              help='Minimum turning radius for pose-based routing in mm (default: 0.2)')
    router_group.add_argument('--debug-lines', action='store_true',
                              help='Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8/9 (centerline)')
    router_group.add_argument('--no-fix-polarity', action='store_true',
                              help="Don't swap target pad nets if polarity swap is needed (default: fix polarity)")
    router_group.add_argument('--no-stub-layer-swap', action='store_true',
                              help='Disable stub layer switching optimization (enabled by default)')
    router_group.add_argument('--can-swap-to-top-layer', action='store_true',
                              help='Allow swapping stubs to F.Cu (top layer). Off by default due to via clearance issues.')
    router_group.add_argument('--max-ripup', type=int,
                              help='Maximum blockers to rip up at once during rip-up and retry (default: 3)')
    router_group.add_argument('--max-setback-angle', type=float,
                              help='Maximum angle (degrees) for setback position search (default: 45.0)')
    router_group.add_argument('--swappable-nets', nargs='+',
                              help='Glob patterns for diff pairs that can have targets swapped (e.g., rx1_*)')
    router_group.add_argument('--crossing-penalty', type=float,
                              help='Penalty for crossing assignments in target swap optimization (default: 100.0)')
    router_group.add_argument('--mps-unroll', action='store_true',
                              help='Use chip boundary unrolling for MPS ordering and target swap crossing detection')

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
        sys.executable, "route.py",
        input_pcb, output_pcb
    ] + net_patterns

    # Add pass-through options
    if args.no_bga_zones:
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
    if args.max_probe_iterations is not None:
        router_cmd.extend(["--max-probe-iterations", str(args.max_probe_iterations)])
    if args.heuristic_weight is not None:
        router_cmd.extend(["--heuristic-weight", str(args.heuristic_weight)])
    if args.stub_proximity_radius is not None:
        router_cmd.extend(["--stub-proximity-radius", str(args.stub_proximity_radius)])
    if args.stub_proximity_cost is not None:
        router_cmd.extend(["--stub-proximity-cost", str(args.stub_proximity_cost)])
    if args.via_proximity_cost is not None:
        router_cmd.extend(["--via-proximity-cost", str(args.via_proximity_cost)])
    if args.bga_proximity_radius is not None:
        router_cmd.extend(["--bga-proximity-radius", str(args.bga_proximity_radius)])
    if args.bga_proximity_cost is not None:
        router_cmd.extend(["--bga-proximity-cost", str(args.bga_proximity_cost)])
    if args.track_proximity_distance is not None:
        router_cmd.extend(["--track-proximity-distance", str(args.track_proximity_distance)])
    if args.track_proximity_cost is not None:
        router_cmd.extend(["--track-proximity-cost", str(args.track_proximity_cost)])
    if args.diff_pair_gap is not None:
        router_cmd.extend(["--diff-pair-gap", str(args.diff_pair_gap)])
    if args.diff_pair_centerline_setback is not None:
        router_cmd.extend(["--diff-pair-centerline-setback", str(args.diff_pair_centerline_setback)])
    if args.min_turning_radius is not None:
        router_cmd.extend(["--min-turning-radius", str(args.min_turning_radius)])
    if args.debug_lines:
        router_cmd.append("--debug-lines")
    if args.no_fix_polarity:
        router_cmd.append("--no-fix-polarity")
    if args.no_stub_layer_swap:
        router_cmd.append("--no-stub-layer-swap")
    if args.can_swap_to_top_layer:
        router_cmd.append("--can-swap-to-top-layer")
    if args.max_ripup is not None:
        router_cmd.extend(["--max-ripup", str(args.max_ripup)])
    if args.max_setback_angle is not None:
        router_cmd.extend(["--max-setback-angle", str(args.max_setback_angle)])
    if args.swappable_nets:
        router_cmd.extend(["--swappable-nets"] + args.swappable_nets)
    if args.crossing_penalty is not None:
        router_cmd.extend(["--crossing-penalty", str(args.crossing_penalty)])
    if args.mps_unroll:
        router_cmd.append("--mps-unroll")
    router_cmd.extend(["--diff-pairs", diff_pair_pattern])

    result = run_command(
        router_cmd,
        f"Step 2: Routing {len(matched_pairs)} differential pair(s)",
        capture_output=True
    )
    router_output = result.stdout if result.stdout else ""

    # Parse JSON summary from router output
    import json
    json_summary = None
    for line in router_output.split('\n'):
        if line.startswith('JSON_SUMMARY: '):
            try:
                json_summary = json.loads(line[14:])
            except json.JSONDecodeError:
                pass
            break

    if not json_summary:
        print("ERROR: Could not parse JSON_SUMMARY from router output")
        return 1

    routed_pairs = json_summary.get('routed_diff_pairs', [])
    failed_routing_pairs = json_summary.get('failed_diff_pairs', [])

    # Step 3: Run DRC check only on successfully routed pairs (unless skipped)
    drc_results = {}  # diff_pair -> (passed, error_count)

    skip_drc = args.skip_drc
    if skip_drc:
        print("\nSkipping DRC checks (--skip-drc)")
    else:
        print(f"\nStep 3: DRC checks for {len(routed_pairs)} routed pair(s)")
        for diff_pair in routed_pairs:
            net_pattern = f"*{diff_pair}_*"
            drc_cmd = [sys.executable, "check_drc.py", output_pcb, "--nets", net_pattern, "--quiet"]
            if args.debug_lines:
                drc_cmd.append("--debug-lines")
            result = subprocess.run(drc_cmd, capture_output=True, text=True)

            # Print the quiet-mode output from the script
            if result.stdout:
                print(result.stdout, end="")
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)

            # Parse DRC output to count errors
            output = result.stdout if result.stdout else ""
            error_count = 0

            # Look for violation counts in output
            # Format: "FOUND 8 DRC VIOLATIONS" or "X violations" or "X violation" or "FAILED (N violations)"
            patterns = [
                r'FAILED\s+\((\d+)\s+violation',      # "FAILED (8 violations)"
                r'FOUND\s+(\d+)\s+DRC\s+VIOLATION',   # "FOUND 8 DRC VIOLATIONS"
                r'(\d+)\s+DRC\s+violation',           # "8 DRC violations"
                r'(\d+)\s+violation',                 # "8 violations"
            ]

            for pattern in patterns:
                match = re.search(pattern, output, re.IGNORECASE)
                if match:
                    error_count = int(match.group(1))
                    break

            passed = result.returncode == 0 and error_count == 0
            drc_results[diff_pair] = (passed, error_count)

    # Step 4: Run connectivity check only on successfully routed pairs (unless skipped)
    connectivity_results = {}  # diff_pair -> (passed, disconnected_count)
    skip_connectivity = args.skip_connectivity or args.debug_lines

    if skip_connectivity:
        print("\nSkipping connectivity checks" + (" (--debug-lines)" if args.debug_lines else " (--skip-connectivity)"))
    else:
        print(f"\nStep 4: Connectivity checks for {len(routed_pairs)} routed pair(s)")
        for diff_pair in routed_pairs:
            net_pattern = f"*{diff_pair}_*"
            result = subprocess.run(
                [sys.executable, "check_connected.py", output_pcb, "--nets", net_pattern, "--quiet"],
                capture_output=True, text=True
            )

            # Print the quiet-mode output from the script
            if result.stdout:
                print(result.stdout, end="")
            if result.stderr:
                print(result.stderr, end="", file=sys.stderr)

            # Parse connectivity output to count issues
            output = result.stdout if result.stdout else ""
            issue_count = 0

            # Look for issue counts in output
            # Format: "FOUND X CONNECTIVITY ISSUES" or "ALL NETS FULLY CONNECTED!" or "FAILED (N issues)"
            patterns = [
                r'FAILED\s+\((\d+)\s+issue',              # "FAILED (2 issues)"
                r'FOUND\s+(\d+)\s+CONNECTIVITY\s+ISSUE',  # "FOUND 2 CONNECTIVITY ISSUES"
                r'(\d+)\s+connectivity\s+issue',          # "2 connectivity issues"
            ]

            for pattern in patterns:
                match = re.search(pattern, output, re.IGNORECASE)
                if match:
                    issue_count = int(match.group(1))
                    break

            passed = result.returncode == 0 and issue_count == 0
            connectivity_results[diff_pair] = (passed, issue_count)

    # Print summary
    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")

    # All values from JSON summary
    total_iterations = json_summary.get('total_iterations', 0)
    total_time = json_summary.get('total_time', 0)
    total_vias = json_summary.get('total_vias', 0)
    ripup_success_pairs = json_summary.get('ripup_success_pairs', [])
    rerouted_pairs_list = json_summary.get('rerouted_pairs', [])
    polarity_swapped_pairs = json_summary.get('polarity_swapped_pairs', [])
    target_swaps = json_summary.get('target_swaps', [])
    layer_swaps = json_summary.get('layer_swaps', 0)

    # Routing summary
    print(f"\nRouting:")
    print(f"  Succeeded: {len(routed_pairs)}/{len(matched_pairs)}")
    print(f"  Total iterations: {total_iterations:,}")
    print(f"  Total via pairs:  {total_vias // 2}")
    print(f"  Total time:       {total_time:.2f}s")
    print(f"  Polarity swaps:   {len(polarity_swapped_pairs)}")
    print(f"  Target swaps:     {len(target_swaps)}")
    print(f"  Layer swaps:      {layer_swaps}")
    if ripup_success_pairs or rerouted_pairs_list:
        print(f"  Rip-up/reroute:   {len(ripup_success_pairs)} rip-up success, {len(rerouted_pairs_list)} rerouted")

    if routed_pairs:
        print(f"  Routed:")
        for pair in routed_pairs:
            suffix = ""
            if pair in ripup_success_pairs:
                suffix = " (rip-up success)"
            elif pair in rerouted_pairs_list:
                suffix = " (rerouted)"
            print(f"    {pair}{suffix}")

    if failed_routing_pairs:
        print(f"  Failed to route:")
        for pair in failed_routing_pairs:
            print(f"    {pair}")

    # DRC summary (only for routed pairs, and only if DRC was run)
    if routed_pairs and not skip_drc:
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
    elif skip_drc:
        print(f"\nDRC: skipped")

    # Connectivity summary (only for routed pairs, and only if connectivity check was run)
    if routed_pairs and not skip_connectivity:
        conn_passed = [p for p, (ok, _) in connectivity_results.items() if ok]
        conn_failed = [p for p, (ok, _) in connectivity_results.items() if not ok]

        print(f"\nConnectivity (of {len(routed_pairs)} routed):")
        print(f"  Passed: {len(conn_passed)}/{len(routed_pairs)}")

        if conn_passed:
            print(f"  OK:")
            for pair in conn_passed:
                print(f"    {pair}")

        if conn_failed:
            print(f"  FAILED:")
            for pair in conn_failed:
                _, issue_count = connectivity_results[pair]
                print(f"    {pair} ({issue_count} disconnected net{'s' if issue_count != 1 else ''})")
    elif skip_connectivity:
        print(f"\nConnectivity: skipped")

    if polarity_swapped_pairs:
        print(f"\nNets with P<->N polarity swap:")
        for pair in polarity_swapped_pairs:
            print(f"  - {pair}")

    if target_swaps:
        print(f"\nNets with target swap:")
        for swap in target_swaps:
            pair1 = swap.get('pair1', '')
            pair2 = swap.get('pair2', '')
            print(f"  - {pair1} <-> {pair2}")

    print(f"\nOutput file: {output_pcb}")
    print(f"{'='*60}")

    # Return error if any routing failures, DRC failures (if DRC was run), or connectivity failures
    has_failures = bool(failed_routing_pairs)
    if not skip_drc:
        has_failures = has_failures or any(not ok for ok, _ in drc_results.values())
    if not skip_connectivity:
        has_failures = has_failures or any(not ok for ok, _ in connectivity_results.values())
    return 1 if has_failures else 0


if __name__ == "__main__":
    sys.exit(main())

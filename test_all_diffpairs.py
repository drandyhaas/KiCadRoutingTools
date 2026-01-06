#!/usr/bin/env python3
"""
Test script that routes all differential pairs and reports results.

Iterates through all diff pairs in the PCB, routes each one,
runs DRC, and provides a summary of successes and failures.
"""

import subprocess
import sys
import os
import argparse
import re
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from kicad_parser import parse_kicad_pcb
from route import find_differential_pairs


def run_test(diff_pair_name, args, verbose=False):
    """Run test_diffpair.py for a single diff pair and return results."""
    # Create a unique temp output file for this test
    temp_fd, temp_output = tempfile.mkstemp(suffix='.kicad_pcb', prefix=f'test_{diff_pair_name}_')
    os.close(temp_fd)

    try:
        # Use wildcard prefix to match full path (e.g., /fpga_adc/lvds_rx4_1)
        cmd = [sys.executable, "test_diffpair.py", f"*{diff_pair_name}"]
        cmd.extend(["--output", temp_output])

        # Add pass-through arguments
        if args.build:
            cmd.append("--build")
        if args.skip_drc:
            cmd.append("--skip-drc")
        if args.skip_connectivity:
            cmd.append("--skip-connectivity")
        if args.input != 'routed_output.kicad_pcb':
            cmd.extend(["--input", args.input])
        if args.no_bga_zones and not args.bga_zones:
            cmd.append("--no-bga-zones")
        if args.ordering:
            cmd.extend(["--ordering", args.ordering])
        if args.direction:
            cmd.extend(["--direction", args.direction])
        if args.layers:
            cmd.extend(["--layers"] + args.layers)
        if args.track_width is not None:
            cmd.extend(["--track-width", str(args.track_width)])
        if args.clearance is not None:
            cmd.extend(["--clearance", str(args.clearance)])
        if args.via_size is not None:
            cmd.extend(["--via-size", str(args.via_size)])
        if args.via_drill is not None:
            cmd.extend(["--via-drill", str(args.via_drill)])
        if args.grid_step is not None:
            cmd.extend(["--grid-step", str(args.grid_step)])
        if args.via_cost is not None:
            cmd.extend(["--via-cost", str(args.via_cost)])
        if args.max_iterations is not None:
            cmd.extend(["--max-iterations", str(args.max_iterations)])
        if args.heuristic_weight is not None:
            cmd.extend(["--heuristic-weight", str(args.heuristic_weight)])
        if args.stub_proximity_radius is not None:
            cmd.extend(["--stub-proximity-radius", str(args.stub_proximity_radius)])
        if args.stub_proximity_cost is not None:
            cmd.extend(["--stub-proximity-cost", str(args.stub_proximity_cost)])
        if args.diff_pair_gap is not None:
            cmd.extend(["--diff-pair-gap", str(args.diff_pair_gap)])
        if args.diff_pair_centerline_setback is not None:
            cmd.extend(["--diff-pair-centerline-setback", str(args.diff_pair_centerline_setback)])
        if args.min_turning_radius is not None:
            cmd.extend(["--min-turning-radius", str(args.min_turning_radius)])
        if args.max_probe_iterations is not None:
            cmd.extend(["--max-probe-iterations", str(args.max_probe_iterations)])
        if args.bga_proximity_radius is not None:
            cmd.extend(["--bga-proximity-radius", str(args.bga_proximity_radius)])
        if args.bga_proximity_cost is not None:
            cmd.extend(["--bga-proximity-cost", str(args.bga_proximity_cost)])
        if args.debug_lines:
            cmd.append("--debug-lines")
        if args.no_fix_polarity:
            cmd.append("--no-fix-polarity")
        if args.stub_layer_swap:
            cmd.append("--stub-layer-swap")
        if args.can_swap_to_top_layer:
            cmd.append("--can-swap-to-top-layer")
        if args.max_ripup is not None:
            cmd.extend(["--max-ripup", str(args.max_ripup)])
        if args.max_setback_angle is not None:
            cmd.extend(["--max-setback-angle", str(args.max_setback_angle)])

        result = subprocess.run(cmd, capture_output=True, text=True)

        # Parse output for success/failure
        output = result.stdout + result.stderr

        drc_passed = "NO DRC VIOLATIONS FOUND!" in output or "DRC... OK" in output
        routing_success = "SUCCESS:" in output

        # Check for polarity swap without vias warning (known limitation)
        polarity_no_vias = "WARNING: Polarity swap needed but no vias" in output

        # Extract any DRC violations
        violations = []
        if "DRC VIOLATIONS" in output:
            # Find violation details - count lines with '<->' which are the actual violation headers
            lines = output.split('\n')
            in_violations = False
            for line in lines:
                if "DRC VIOLATIONS" in line:
                    in_violations = True
                elif in_violations and '<->' in line:
                    violations.append(line.strip())
                elif in_violations and "=" * 20 in line:
                    in_violations = False

        # Check for routing failure
        # Note: "No route found after N iterations, trying backwards..." is not a failure
        # Only "No route found after N iterations (both directions)" is a failure
        routing_failed = "Routing failed" in output or "(both directions)" in output

        # Parse SUCCESS line for iterations, vias, time
        # Format: "SUCCESS: X segments, Y vias, Z iterations (T.TTs)"
        iterations = 0
        vias = 0
        route_time = 0.0
        success_match = re.search(r'SUCCESS:\s*(\d+)\s*segments?,\s*(\d+)\s*vias?,\s*(\d+)\s*iterations?\s*\(([0-9.]+)s\)', output)
        if success_match:
            vias = int(success_match.group(2))
            iterations = int(success_match.group(3))
            route_time = float(success_match.group(4))

        # Check for polarity fix (N<->P swap)
        polarity_fixed = "Polarity fixed:" in output or "Polarity swap needed - will swap" in output

        return {
            'name': diff_pair_name,
            'routing_success': routing_success and not routing_failed,
            'drc_success': drc_passed or len(violations) == 0,
            'polarity_no_vias': polarity_no_vias,
            'polarity_fixed': polarity_fixed,
            'violations': violations,
            'iterations': iterations,
            'vias': vias,
            'route_time': route_time,
            'output': output if verbose else None
        }
    finally:
        # Clean up temp file
        if os.path.exists(temp_output):
            os.remove(temp_output)


def main():
    parser = argparse.ArgumentParser(description='Test all differential pairs')
    parser.add_argument('--pattern', '-p', default='*lvds*',
                        help='Pattern to filter diff pairs (default: *lvds*)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed output for each test')
    parser.add_argument('--stop-on-error', '-s', action='store_true',
                        help='Stop on first error')
    parser.add_argument('--threads', '-t', type=int, default=14,
                        help='Number of parallel threads (default: 14)')

    # Test script options (pass-through to test_diffpair.py)
    parser.add_argument('--build', action='store_true',
                        help='Build the Rust router before running')
    parser.add_argument('--skip-drc', action='store_true',
                        help='Skip DRC checks after routing')
    parser.add_argument('--skip-connectivity', action='store_true',
                        help='Skip connectivity checks after routing')
    parser.add_argument('--input', default='kicad_files/routed_output.kicad_pcb',
                        help='Input PCB file (default: kicad_files/routed_output.kicad_pcb)')

    # Router options (pass-through to route.py)
    router_group = parser.add_argument_group('Router options (passed to route.py)')
    router_group.add_argument('--ordering', '-o', choices=['inside_out', 'mps', 'original'],
                              help='Net ordering strategy (default: mps)')
    router_group.add_argument('--direction', '-d', choices=['forward', 'backward', 'random'],
                              help='Direction search order')
    router_group.add_argument('--no-bga-zones', action='store_true',
                              help='Disable BGA exclusion zones')
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
                              help='Max A* iterations (default: 200000)')
    router_group.add_argument('--heuristic-weight', type=float,
                              help='A* heuristic weight (default: 2.0)')
    router_group.add_argument('--stub-proximity-radius', type=float,
                              help='Radius around stubs to penalize in mm (default: 5.0)')
    router_group.add_argument('--stub-proximity-cost', type=float,
                              help='Cost penalty near stubs in mm equivalent (default: 0.2)')
    router_group.add_argument('--diff-pair-gap', type=float,
                              help='Gap between P/N traces in mm (default: 0.101)')
    router_group.add_argument('--diff-pair-centerline-setback', type=float,
                              help='Distance in front of stubs to start route in mm (default: 2x P-N spacing)')
    router_group.add_argument('--min-turning-radius', type=float,
                              help='Minimum turning radius for pose-based routing in mm (default: 0.2)')
    router_group.add_argument('--max-probe-iterations', type=int,
                              help='Max iterations for quick probe per direction (default: 5000)')
    router_group.add_argument('--bga-proximity-radius', type=float,
                              help='Radius around BGA edges to penalize in mm (default: 10.0)')
    router_group.add_argument('--bga-proximity-cost', type=float,
                              help='Cost penalty near BGA edges in mm equivalent (default: 0.2)')
    router_group.add_argument('--debug-lines', action='store_true',
                              help='Output debug geometry on User.3 (connectors), User.4 (stub dirs), User.8/9 (centerline)')
    router_group.add_argument('--no-fix-polarity', action='store_true',
                              help="Don't swap target pad nets when polarity swap needed (default: fix polarity)")
    router_group.add_argument('--stub-layer-swap', action='store_true',
                              help='Enable stub layer switching optimization (experimental)')
    router_group.add_argument('--can-swap-to-top-layer', action='store_true',
                              help='Allow swapping stubs to F.Cu (top layer). Off by default.')
    router_group.add_argument('--max-ripup', type=int,
                              help='Maximum blockers to rip up at once during rip-up and retry (default: 3)')
    router_group.add_argument('--max-setback-angle', type=float,
                              help='Maximum angle (degrees) for setback position search (default: 22.5)')

    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Load PCB and find diff pairs
    input_pcb = args.input
    print(f"Loading {input_pcb} to find differential pairs...")
    pcb_data = parse_kicad_pcb(input_pcb)

    diff_pairs = find_differential_pairs(pcb_data, [args.pattern])

    if not diff_pairs:
        print(f"No differential pairs found matching pattern '{args.pattern}'")
        return 1

    # Extract base names (without _P/_N suffix)
    pair_names = sorted(diff_pairs.keys())

    # Convert full path names to short names for test_diffpair.py
    # e.g., "/fpga_adc/lvds_rx4_1" -> "lvds_rx4_1"
    short_names = []
    for name in pair_names:
        # Extract the last part after the last /
        short = name.split('/')[-1]
        short_names.append(short)

    print(f"Found {len(short_names)} differential pairs to test")
    print(f"Running with {args.threads} threads")
    print("=" * 60)

    # Run tests in parallel
    results = []
    completed = 0
    stop_flag = False

    with ThreadPoolExecutor(max_workers=args.threads) as executor:
        # Submit all tasks
        future_to_name = {
            executor.submit(run_test, name, args, args.verbose): name
            for name in short_names
        }

        for future in as_completed(future_to_name):
            if stop_flag:
                continue

            name = future_to_name[future]
            completed += 1

            try:
                result = future.result()
                results.append(result)

                if result['routing_success'] and result['drc_success']:
                    print(f"[{completed}/{len(short_names)}] {name}: PASS")
                elif not result['routing_success']:
                    print(f"[{completed}/{len(short_names)}] {name}: FAIL - Routing failed")
                    if args.stop_on_error:
                        stop_flag = True
                elif result['polarity_no_vias']:
                    print(f"[{completed}/{len(short_names)}] {name}: KNOWN LIMITATION - Polarity swap without vias")
                else:
                    print(f"[{completed}/{len(short_names)}] {name}: FAIL - DRC violations: {len(result['violations'])}")
                    for v in result['violations'][:3]:
                        print(f"    {v}")
                    if len(result['violations']) > 3:
                        print(f"    ... and {len(result['violations']) - 3} more")
                    if args.stop_on_error:
                        stop_flag = True

                if args.verbose and result['output']:
                    print(result['output'])

            except Exception as e:
                print(f"[{completed}/{len(short_names)}] {name}: ERROR - {e}")
                if args.stop_on_error:
                    stop_flag = True

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    passed = [r for r in results if r['routing_success'] and r['drc_success']]
    routing_failed = [r for r in results if not r['routing_success']]
    polarity_no_vias = [r for r in results if r['routing_success'] and not r['drc_success'] and r['polarity_no_vias']]
    drc_failed = [r for r in results if r['routing_success'] and not r['drc_success'] and not r['polarity_no_vias']]
    polarity_fixed = [r for r in results if r.get('polarity_fixed', False)]

    # Aggregate statistics
    total_iterations = sum(r.get('iterations', 0) for r in results)
    total_vias = sum(r.get('vias', 0) for r in results)
    total_time = sum(r.get('route_time', 0) for r in results)

    print(f"Total:              {len(results)}")
    print(f"Passed:             {len(passed)}")
    print(f"Known limitations:  {len(polarity_no_vias)}")
    print(f"Routing failed:     {len(routing_failed)}")
    print(f"DRC failed:         {len(drc_failed)}")
    print()
    print(f"Total iterations:   {total_iterations:,}")
    print(f"Total via pairs:    {total_vias // 2}")
    print(f"Total routing time: {total_time:.2f}s")
    print(f"Polarity swaps:     {len(polarity_fixed)}")

    if polarity_fixed:
        print(f"\nNets with P<->N polarity swap:")
        for r in sorted(polarity_fixed, key=lambda x: x['name']):
            print(f"  - {r['name']}")

    if polarity_no_vias:
        print(f"\nKnown limitations (polarity swap without vias):")
        for r in polarity_no_vias:
            print(f"  - {r['name']}")

    if routing_failed:
        print(f"\nRouting failures:")
        for r in routing_failed:
            print(f"  - {r['name']}")

    if drc_failed:
        print(f"\nDRC failures:")
        for r in drc_failed:
            print(f"  - {r['name']}: {len(r['violations'])} violations")

    # Success if no routing failures and no unexpected DRC failures
    if not routing_failed and not drc_failed:
        if polarity_no_vias:
            print(f"\nAll tests passed ({len(polarity_no_vias)} with known limitations)!")
        else:
            print("\nAll tests passed!")
        return 0
    else:
        return 1


if __name__ == "__main__":
    sys.exit(main())

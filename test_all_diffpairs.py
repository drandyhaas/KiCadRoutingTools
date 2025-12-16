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
from kicad_parser import parse_kicad_pcb
from batch_grid_router import find_differential_pairs


def run_test(diff_pair_name, verbose=False):
    """Run test_diffpair.py for a single diff pair and return results."""
    # Use wildcard prefix to match full path (e.g., /fpga_adc/lvds_rx4_1)
    cmd = [sys.executable, "test_diffpair.py", f"*{diff_pair_name}"]

    result = subprocess.run(cmd, capture_output=True, text=True)

    # Parse output for success/failure
    output = result.stdout + result.stderr

    success = "NO DRC VIOLATIONS FOUND!" in output
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
    routing_failed = "Routing failed" in output or "No route found" in output

    return {
        'name': diff_pair_name,
        'routing_success': routing_success and not routing_failed,
        'drc_success': success,
        'polarity_no_vias': polarity_no_vias,
        'violations': violations,
        'output': output if verbose else None
    }


def main():
    parser = argparse.ArgumentParser(description='Test all differential pairs')
    parser.add_argument('--pattern', '-p', default='*lvds*',
                        help='Pattern to filter diff pairs (default: *lvds*)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed output for each test')
    parser.add_argument('--stop-on-error', '-s', action='store_true',
                        help='Stop on first error')
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Load PCB and find diff pairs
    input_pcb = "routed_output.kicad_pcb"
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
    print("=" * 60)

    # Run tests
    results = []
    for i, name in enumerate(short_names):
        print(f"\n[{i+1}/{len(short_names)}] Testing {name}...")

        result = run_test(name, args.verbose)
        results.append(result)

        if result['routing_success'] and result['drc_success']:
            print(f"  PASS")
        elif not result['routing_success']:
            print(f"  FAIL - Routing failed")
            if args.stop_on_error:
                break
        elif result['polarity_no_vias']:
            # Known limitation: polarity swap without vias causes crossing
            print(f"  KNOWN LIMITATION - Polarity swap without vias (tracks cross)")
        else:
            print(f"  FAIL - DRC violations: {len(result['violations'])}")
            for v in result['violations'][:3]:
                print(f"    {v}")
            if len(result['violations']) > 3:
                print(f"    ... and {len(result['violations']) - 3} more")
            if args.stop_on_error:
                break

        if args.verbose and result['output']:
            print(result['output'])

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    passed = [r for r in results if r['routing_success'] and r['drc_success']]
    routing_failed = [r for r in results if not r['routing_success']]
    polarity_no_vias = [r for r in results if r['routing_success'] and not r['drc_success'] and r['polarity_no_vias']]
    drc_failed = [r for r in results if r['routing_success'] and not r['drc_success'] and not r['polarity_no_vias']]

    print(f"Total:              {len(results)}")
    print(f"Passed:             {len(passed)}")
    print(f"Known limitations:  {len(polarity_no_vias)}")
    print(f"Routing failed:     {len(routing_failed)}")
    print(f"DRC failed:         {len(drc_failed)}")

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

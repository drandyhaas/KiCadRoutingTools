#!/usr/bin/env python3
"""
Test script for differential pair routing.

Builds the Rust router, routes a test differential pair,
and runs DRC to check for violations.
"""

import subprocess
import sys
import os
import argparse

def run_command(cmd, description):
    """Run a command and print its output."""
    print(f"\n{'='*60}")
    print(f"{description}")
    print(f"{'='*60}")
    print(f"Running: {' '.join(cmd)}\n")

    result = subprocess.run(cmd, capture_output=False)
    return result.returncode

def main():
    parser = argparse.ArgumentParser(description='Test differential pair routing')
    parser.add_argument('diff_pair', nargs='?', default='lvds_rx4_1',
                        help='Name of the diff pair to route (default: lvds_rx4_1). '
                             'Examples: lvds_rx3_10 (no layer/polarity change), '
                             'lvds_rx3_7 (layer change, no polarity change), '
                             'lvds_rx4_1 (layer change and polarity change)')
    parser.add_argument('--build', action='store_true',
                        help='Build the Rust router before running')
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    # Configuration
    input_pcb = "routed_output.kicad_pcb"
    output_pcb = "test_batch_diffpair.kicad_pcb"
    net_pattern = f"*{args.diff_pair}_*"
    diff_pair_pattern = "*lvds*"

    # Step 1: Build the Rust router (optional)
    if args.build:
        ret = run_command(
        [sys.executable, "build_router.py"],
        "Step 1: Building Rust router"
        )
        if ret != 0:
            print("\nERROR: Build failed!")
            return 1

    # Step 2: Route the differential pair
    ret = run_command(
        [sys.executable, "batch_grid_router.py",
         input_pcb, output_pcb, net_pattern,
         "--no-bga-zones",
         "--diff-pair-centerline-setback", "1.5",
         "--diff-pairs", diff_pair_pattern],
        "Step 2: Routing differential pair"
    )
    if ret != 0:
        print("\nERROR: Routing failed!")
        return 1

    # Step 3: Run DRC check
    ret = run_command(
        [sys.executable, "check_drc.py", output_pcb, "--nets", net_pattern],
        "Step 3: Running DRC check"
    )

    print(f"\n{'='*60}")
    print("Test complete!")
    print(f"Output file: {output_pcb}")
    print(f"{'='*60}")

    return 0

if __name__ == "__main__":
    sys.exit(main())

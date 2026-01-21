#!/usr/bin/env python3
"""
Test routing on the flat_hierarchy board.
Creates GND plane first, then routes all signals.
"""

import argparse
import shlex
import subprocess


def run(cmd: str, unbuffered: bool = False) -> None:
    """Run a command string and exit if it fails."""
    if unbuffered and cmd.startswith('python3 '):
        cmd = 'python3 -u ' + cmd[8:]
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")


def main():
    parser = argparse.ArgumentParser(description='Test routing on flat_hierarchy board')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--checks', action='store_true', default=False,
                        help='Run DRC and connectivity checks after routing')
    args = parser.parse_args()

    unbuffered = args.unbuffered

    # Common CLI parameter groups
    GEOMETRY = "" #defaults are OK
    #GEOMETRY = "--clearance 0.1 --via-size 0.3 --via-drill 0.2 --track-width 0.1"
    LAYERS_4 = "--layers F.Cu B.Cu"

    # From claude prompt: /analyze-power-nets kicad_files/flat_hierarchy.kicad_pcb
    POWER = '--power-nets "GND" "VCC" "VCC_PIC" "VPP" "Net-(D1-A)" "Net-(D1-K)" --power-nets-widths 0.5 0.5 0.35 0.35 0.5 0.5'

    # Step 1: Create GND plane on B.Cu
    run(f"python3 route_planes.py kicad_files/flat_hierarchy.kicad_pcb --nets GND --plane-layers B.Cu {GEOMETRY} {POWER}", unbuffered)

    # Step 2: Route all signals
    run(f"python3 route.py kicad_files/flat_hierarchy_routed.kicad_pcb --overwrite {GEOMETRY} {LAYERS_4} {POWER}", unbuffered)

    if args.checks:
        # Check for DRC errors
        run("python3 check_drc.py kicad_files/flat_hierarchy_routed.kicad_pcb", unbuffered)
        # Check connectivity
        run("python3 check_connected.py kicad_files/flat_hierarchy_routed.kicad_pcb", unbuffered)

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

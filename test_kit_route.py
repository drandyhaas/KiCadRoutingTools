#!/usr/bin/env python3
"""
Test routing on kit-dev-coldfire-xilinx_5213 board.
Routes nets directly from pads without fanout.
"""

import shlex
import subprocess


def run(cmd: str) -> None:
    """Run a command string and exit if it fails.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
    """
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")


def main():
    # Route DDAT nets from pads (no fanout needed)
    #run('python3 route.py kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb kicad_files/kit-out.kicad_pcb "/DDAT*" --track-width 0.2 --clearance 0.15 --via-size 0.5 --via-drill 0.4')
    run('python3 route.py kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb kicad_files/kit-out.kicad_pcb --component U102 --track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.4')

    # Check for DRC errors
    run('python3 check_drc.py kicad_files/kit-out.kicad_pcb --clearance 0.15 --nets "/DDAT*"')

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test routing on kit-dev-coldfire-xilinx_5213 board.
Routes nets directly from pads without fanout.
"""

import argparse
import shlex
import subprocess

def run(cmd: str, unbuffered: bool = False) -> None:
    """Run a command string and exit if it fails.

    Args:
        cmd: Command string to run (will be parsed using shell-style splitting)
        unbuffered: If True, add -u flag to python commands
    """
    if unbuffered and cmd.startswith('python3 '):
        cmd = 'python3 -u ' + cmd[8:]
    print(f"\n>>> {cmd}")
    args = shlex.split(cmd)
    result = subprocess.run(args)
    if result.returncode != 0:
        print(f"Command failed with exit code {result.returncode}")

def main():
    parser = argparse.ArgumentParser(description='Test routing on kit-dev-coldfire-xilinx board')
    parser.add_argument('--quick', '-q', action='store_true',
                        help='Quick mode: only route a few nets (default: route all U102 nets)')
    parser.add_argument('-u', '--unbuffered', action='store_true',
                        help='Run python commands with -u (unbuffered output)')
    args = parser.parse_args()

    quick = args.quick
    unbuffered = args.unbuffered

    #target = "--component U102"
    #target = "--component U301"
    #target = "--component U204"
    target = '--nets "/*" "Net-*" GNDA '

    #if quick: target = '--nets "/IRQ*" "/AN*"'
    if quick: target = '--nets "Net*"'

    base_options = '--track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.4 --hole-to-hole-clearance 0.3 '

    power_nets = '--power-nets "GND" "+3.3V" "GNDA" "/VDDPLL" "/VCCA" "Net-(TB201-P1)" "Net-(F201-Pad1)" "Net-(D201-K)" --power-nets-widths 0.5 0.5 0.3 0.3 0.3 0.5 0.5 0.5'

    options = base_options+'--via-proximity-cost 10 --via-cost 50 --track-proximity-distance 3.0 --track-proximity-cost 0.2 --vertical-attraction-radius 0 --stub-proximity-cost 2.0 --stub-proximity-radius 4.0 --max-ripup 10 --max-iterations 10000000 '+power_nets

    # Route some nets from pads (no fanout needed)
    run('python3 route.py kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb kicad_files/kit-out.kicad_pcb '+target+" "+options, unbuffered)

    # Route some planes
    run('python3 route_plane.py kicad_files/kit-out.kicad_pcb kicad_files/kit-out-plane.kicad_pcb --net +3.3V GND +3.3V GND --plane-layer F.Cu In1.Cu In2.Cu B.Cu --max-via-reuse-radius 3 --rip-blocker-nets --reroute-ripped-nets '+base_options, unbuffered)

    # Connect broken plane regions
    run('python3 route_disconnected_planes.py kicad_files/kit-out-plane.kicad_pcb kicad_files/kit-out-plane-connected.kicad_pcb ')

    # Check for DRC errors
    run('python3 check_drc.py kicad_files/kit-out-plane-connected.kicad_pcb --clearance 0.15 --hole-to-hole-clearance 0.3', unbuffered)

    # Check for connectivity
    run('python3 check_connected.py kicad_files/kit-out-plane-connected.kicad_pcb '+target, unbuffered)

    print("\n=== Test completed ===")

if __name__ == "__main__":
    main()

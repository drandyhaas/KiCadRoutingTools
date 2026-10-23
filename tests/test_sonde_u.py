#!/usr/bin/env python3
"""
Test routing on the sonde_u board at the router defaults, then a GND plane
on B.Cu with return vias; grades DRC and connectivity of the planed board.
"""

import argparse
from run_utils import run


def main():
    parser = argparse.ArgumentParser(description='Test routing on sonde_u board')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--no-checks', action='store_true', default=False,
                        help='Skip DRC and connectivity checks after routing')
    args = parser.parse_args()

    unbuffered = args.unbuffered

    OPTIONS='' #--track-width 1.2 --clearance 0.3 --board-edge-clearance 0.6'

    # Route (the wide-track options above are commented out)
    run(f"python3 py_router/route.py kicad_files/sonde_u.kicad_pcb {OPTIONS}", unbuffered)

    # Add some gnd vias and the gnd plane
    run('python3 py_router/route_planes.py kicad_files/sonde_u_routed.kicad_pcb kicad_files/sonde_u_planes.kicad_pcb '
        '--nets GND --plane-layers B.Cu --add-gnd-vias', unbuffered)

    if not args.no_checks:
        # Check for DRC errors
        run("python3 py_router/check_drc.py kicad_files/sonde_u_planes.kicad_pcb --clearance 0.2", unbuffered)
        # Check connectivity
        run("python3 py_router/check_connected.py kicad_files/sonde_u_planes.kicad_pcb", unbuffered)

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test routing on the interf_u board.
Creates VCC/GND planes first, fans out U9 (PGA120) nets, routes all signals,
then connects disconnected plane regions.
"""

import argparse
from run_utils import run


def main():
    parser = argparse.ArgumentParser(description='Test routing on interf_u board')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--checks', action='store_true', default=True,
                        help='Run DRC and connectivity checks after routing')
    args = parser.parse_args()

    unbuffered = args.unbuffered

    # Step 1: Create VCC and GND planes on both layers
    run("python3 route_planes.py kicad_files/interf_u_unrouted.kicad_pcb kicad_files/interf_u_plane.kicad_pcb --nets VCC GND --plane-layers F.Cu B.Cu", unbuffered)

    # Step 2: Fan out U9 (PGA120) nets
    run('python3 bga_fanout.py kicad_files/interf_u_plane.kicad_pcb --component U9 --output kicad_files/interf_u_fanout.kicad_pcb --nets "/*" ', unbuffered)

    # Step 3: Route all signals
    run("python3 route.py kicad_files/interf_u_fanout.kicad_pcb kicad_files/interf_u_routed.kicad_pcb --stats --no-bga-zone --add-teardrops --layer-costs 1 1 --max-ripup 30 --stub-proximity-radius 10 --stub-proximity-cost 3.0 --track-proximity-cost 1.0 --max-iterations 10000000 --board-edge-clearance 0.55", unbuffered)

    # Step 4: Connect disconnected plane regions
    run("python3 route_disconnected_planes.py kicad_files/interf_u_routed.kicad_pcb kicad_files/interf_u_connected.kicad_pcb --board-edge-clearance 0.6", unbuffered)

    if args.checks:
        # Check for DRC errors
        run("python3 check_drc.py kicad_files/interf_u_connected.kicad_pcb", unbuffered)
        # Check connectivity
        run("python3 check_connected.py kicad_files/interf_u_connected.kicad_pcb", unbuffered)

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test routing on the interf_u board.
Creates VCC/GND planes first, fans out U9 (PGA120) nets, routes all signals,
then connects disconnected plane regions.
"""

import argparse
from run_utils import run

#: A four-step chain on interf_u (25 footprints, 174 nets, 379 pads): planes on
#: both layers, a PGA120 fan-out, a full signal route at --max-iterations
#: 1000000 --max-ripup 10, then plane repair and two checkers. It carried NO
#: budget, so `run_all` gave it the 600 s default and killed it there (#930) --
#: and an undeclared budget cannot tell "this chain is long" from "this chain
#: wedged", so every full-suite run ended on a non-pass a reader had to spend
#: another ten minutes re-running by hand to dismiss.
#:
#: MEASURED: 111.5 s wall, alone, on a fast 2026 arm64 macOS machine
#: (`/usr/bin/time -p`, 2026-09-09). 1200 s is ~10x that -- the suite runs four
#: tests in parallel, and the reporting machine that hit the 600 s wall is a
#: slower Windows box. The figure is a claim about the CHAIN, not the machine:
#: if this file ever times out at 1200 s, something wedged.
RUN_ALL_TIMEOUT = 1200


def main():
    parser = argparse.ArgumentParser(description='Test routing on interf_u board')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--checks', action='store_true', default=True,
                        help='Run DRC and connectivity checks after routing')
    args = parser.parse_args()

    unbuffered = args.unbuffered

    # Step 1: Create VCC and GND planes on both layers
    run("python3 py_router/route_planes.py kicad_files/interf_u_unrouted.kicad_pcb kicad_files/interf_u_plane.kicad_pcb --nets VCC GND --plane-layers F.Cu B.Cu", unbuffered)

    # Step 2: Fan out U9 (PGA120) nets
    run('python3 py_router/bga_fanout.py kicad_files/interf_u_plane.kicad_pcb --component U9 --output kicad_files/interf_u_fanout.kicad_pcb --nets "/*" ', unbuffered)

    # Step 3: Route all signals
    run("python3 py_router/route.py kicad_files/interf_u_fanout.kicad_pcb kicad_files/interf_u_routed.kicad_pcb --no-bga-zone --add-teardrops --layer-costs 1 1 --max-ripup 10 --stub-proximity-radius 10 --stub-proximity-cost 3.0 --max-iterations 1000000 --board-edge-clearance 0.55", unbuffered)

    # Step 4: Connect disconnected plane regions
    run("python3 py_router/repair_planes.py kicad_files/interf_u_routed.kicad_pcb kicad_files/interf_u_connected.kicad_pcb --board-edge-clearance 0.6", unbuffered)

    if args.checks:
        # Check for DRC errors
        run("python3 py_router/check_drc.py kicad_files/interf_u_connected.kicad_pcb", unbuffered)
        # Check connectivity
        run("python3 py_router/check_connected.py kicad_files/interf_u_connected.kicad_pcb", unbuffered)

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

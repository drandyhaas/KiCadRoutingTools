#!/usr/bin/env python3
"""
Test routing on kit-dev-coldfire-xilinx_5213 board.
Routes nets directly from pads without fanout.
"""

import argparse
from run_utils import run


def main():
    parser = argparse.ArgumentParser(description='Test routing on kit-dev-coldfire-xilinx board')
    parser.add_argument('--quick', '-q', action='store_true',
                        help='Quick mode: only route a few nets (default: route all U102 nets)')
    parser.add_argument('-u', '--unbuffered', action='store_true',
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--planes-only', action='store_true',
                        help='Skip the routing and just redo the planes')
    args = parser.parse_args()

    quick = args.quick
    unbuffered = args.unbuffered

    #target = "--component U102"
    #target = "--component U301"
    #target = "--component U204"
    target = '--nets "/*" "Net-*" GNDA '

    if quick: target = '--nets "/AN*"'
    #if quick: target = '--nets "Net*"'

    base_options = '--track-width 0.2 --clearance 0.2 --via-size 0.5 --via-drill 0.4 --hole-to-hole-clearance 0.3 --layers F.Cu In1.Cu In2.Cu B.Cu '

    power_nets = '--power-nets "GND" "+3.3V" "GNDA" "/VDDPLL" "/VCCA" "Net-(TB201-P1)" "Net-(F201-Pad1)" "Net-(D201-K)" --power-nets-widths 0.5 0.5 0.3 0.3 0.3 0.5 0.5 0.5'

    options = base_options+'--proximity-heuristic-factor 0.02 --direction-preference-cost 50 --ripped-route-avoidance-radius 1.0 --ripped-route-avoidance-cost 10.0 \
    --via-proximity-cost 10 --via-cost 50 --track-proximity-distance 3.0 --track-proximity-cost 0.0 --vertical-attraction-cost 0.0 \
    --stub-proximity-cost 2.0 --stub-proximity-radius 4.0 --max-ripup 10 --max-iterations 10000000 \
    --bus --bus-detection-radius 5 --bus-attraction-bonus 5000 --bus-attraction-radius 3 '+power_nets

    # Route some nets from pads (no fanout needed)
    if not args.planes_only:
        run('python3 route.py kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb kicad_files/kit-out.kicad_pcb '+target+" "+options, unbuffered)

    # Route some power nets with vias to planes
    run('python3 route_planes.py kicad_files/kit-out.kicad_pcb kicad_files/kit-out-plane.kicad_pcb --nets +3.3V GND +3.3V GND --plane-layers F.Cu In1.Cu In2.Cu B.Cu \
    --max-via-reuse-radius 3 --rip-blocker-nets --reroute-ripped-nets '+base_options, unbuffered)

    # Connect broken plane regions
    run('python3 route_disconnected_planes.py kicad_files/kit-out-plane.kicad_pcb kicad_files/kit-out-plane-connected.kicad_pcb --analysis-grid-step 0.1 '+base_options)

    # Check for DRC errors
    run('python3 check_drc.py kicad_files/kit-out-plane-connected.kicad_pcb --clearance 0.2 --hole-to-hole-clearance 0.3', unbuffered)

    # Check for connectivity
    run('python3 check_connected.py kicad_files/kit-out-plane-connected.kicad_pcb '+target, unbuffered)

    # Check for orphan stub segments
    run('python3 check_orphan_stubs.py kicad_files/kit-out-plane-connected.kicad_pcb ')

    print("\n=== Test completed ===")

if __name__ == "__main__":
    main()

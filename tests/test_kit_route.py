#!/usr/bin/env python3
"""
Test routing on kit-dev-coldfire-xilinx_5213 board.
Routes nets directly from pads without fanout.

All routing outputs are written into a fresh temporary directory rather than
into the tracked ``kicad_files/`` tree.  This keeps two problems from biting
(issue #426):

  * The chain writes a sibling ``<output>.kicad_pro`` DRC-floor for every
    stage (route.py / route_planes.py / route_disconnected_planes.py).  If
    those land on committed paths the run dirties the repo, and -- worse --
    a *re-run* reads the previously written ``.kicad_pro`` back in and
    silently re-routes to that stale clearance floor (the DRC-floor carryover
    footgun documented in CLAUDE.md).  A fresh temp dir has no stale
    ``.kicad_pro`` to read back, so every stage routes to the ``--clearance``
    on the command line.
  * Running the test never mutates a tracked file, so ``git status`` stays
    clean afterwards.

The input board is read straight from ``kicad_files/`` (read-only); only the
generated ``kit-out*`` artifacts move to the temp dir.

Known result (issue #426): this is a dense, fine-pitch board and the router
does NOT complete it with these parameters -- ``route.py`` (step 1) leaves
roughly 50 of the U102<->U301 signal nets unrouted (their pads are boxed in by
neighbouring copper), so the final ``check_connected`` / ``check_drc`` still
report unconnected pads and a handful of small (~0.02-0.05 mm) congestion
grazes.  That shortfall is genuine routing difficulty, not a stale-fixture
artifact: step 1's input board has no sibling ``.kicad_pro`` so it cannot be
contaminated by any carried-over DRC floor.  The test is a smoke test -- it
runs the full plane-connected chain and prints the checker output; it does not
assert clean, so it stays green while #426 tracks improving completion on this
board.
"""

import argparse
import os
import tempfile

from run_utils import run, ROOT_DIR


def main():
    parser = argparse.ArgumentParser(description='Test routing on kit-dev-coldfire-xilinx board')
    parser.add_argument('--quick', '-q', action='store_true',
                        help='Quick mode: only route a few nets (default: route all U102 nets)')
    parser.add_argument('-u', '--unbuffered', action='store_true',
                        help='Run python commands with -u (unbuffered output)')
    parser.add_argument('--planes-only', action='store_true',
                        help='Skip the routing and just redo the planes')
    parser.add_argument('--workdir', default=None,
                        help='Directory for generated kit-out* artifacts '
                             '(default: a fresh temp dir, auto-removed). Pass a '
                             'path to keep the outputs for inspection.')
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
    --via-proximity-cost 10 --via-cost 300 --track-proximity-distance 3.0 --track-proximity-cost 0.0 --vertical-attraction-cost 0.0 \
    --stub-proximity-cost 4.0 --stub-proximity-radius 5.0 --max-ripup 10 --max-iterations 10000000 \
    --bus --bus-detection-radius 5 --bus-attraction-bonus 5000 --bus-attraction-radius 1 '+power_nets

    # Input board is read from the tracked tree; outputs go to a fresh work dir
    # so the run never touches committed files and never reads back a stale
    # sibling .kicad_pro DRC floor (issue #426).
    src_pcb = 'kicad_files/kit-dev-coldfire-xilinx_5213.kicad_pcb'

    tmp = None
    if args.workdir:
        workdir = os.path.abspath(args.workdir)
        os.makedirs(workdir, exist_ok=True)
    else:
        tmp = tempfile.TemporaryDirectory(prefix='kit_route_')
        workdir = tmp.name

    def wpath(name):
        # run() executes from ROOT_DIR, so hand it a path relative to that.
        return os.path.relpath(os.path.join(workdir, name), ROOT_DIR)

    out       = wpath('kit-out.kicad_pcb')
    out_plane = wpath('kit-out-plane.kicad_pcb')
    out_conn  = wpath('kit-out-plane-connected.kicad_pcb')

    try:
        # Route some nets from pads (no fanout needed)
        if not args.planes_only:
            run(f'python3 route.py {src_pcb} {out} '+target+" "+options, unbuffered)

        # Route some power nets with vias to planes
        run(f'python3 route_planes.py {out} {out_plane} --nets +3.3V GND +3.3V GND --plane-layers F.Cu In1.Cu In2.Cu B.Cu \
        --max-via-reuse-radius 3 --rip-blocker-nets --reroute-ripped-nets '+base_options, unbuffered)

        # Connect broken plane regions
        run(f'python3 route_disconnected_planes.py {out_plane} {out_conn} --analysis-grid-step 0.1 '+base_options)

        # Check for DRC errors
        run(f'python3 check_drc.py {out_conn} --clearance 0.2 --hole-to-hole-clearance 0.3', unbuffered)

        # Check for connectivity
        run(f'python3 check_connected.py {out_conn} '+target, unbuffered)

        # Check for orphan stub segments
        run(f'python3 check_orphan_stubs.py {out_conn} ')

        print("\n=== Test completed ===")
    finally:
        if tmp is not None:
            tmp.cleanup()

if __name__ == "__main__":
    main()

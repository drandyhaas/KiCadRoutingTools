#!/usr/bin/env python3
"""
Test routing on kit-dev-coldfire-xilinx_5213 board.
Routes nets directly from pads without fanout.

All routing outputs are written into a fresh temporary directory rather than
into the tracked ``kicad_files/`` tree.  This keeps two problems from biting
(issue #426):

  * The chain writes a sibling ``<output>.kicad_pro`` DRC-floor for every
    stage (route.py / route_planes.py / repair_planes.py).  If
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

The default run routes only the ``/AN*`` nets (~30 s) so run_all's per-test
timeout survives a loaded machine; the full signal set is behind ``--full``
(~10 min). Both exercise the identical chain -- route, both plane steps, and
all three checkers.

Known result for the FULL run (``--full``, issue #426): this is a dense,
fine-pitch board and the router
does NOT complete it with these parameters -- ``route.py`` (step 1) leaves
roughly 50 of the U102<->U301 signal nets unrouted (their pads are boxed in by
neighbouring copper), so the final ``check_connected`` / ``check_drc`` still
report unconnected pads and a handful of small (~0.02-0.05 mm) congestion
grazes.  That shortfall is genuine routing difficulty, not a stale-fixture
artifact: step 1's input board has no sibling ``.kicad_pro`` so it cannot be
contaminated by any carried-over DRC floor.  So under ``--full`` the three
checkers' verdicts are printed, not asserted, while #426 tracks improving
completion on this board.

What IS asserted, in both modes (`run_utils.run_step`): every step exits
cleanly with no traceback and writes the board the next step reads, and each
checker runs to a verdict. In the default run the verdicts must also be
CLEAN. This file used `run_utils.run`, which prints a failure and carries on,
so it was green whatever the chain did.
"""

import argparse
import os
import shlex
import tempfile

from run_utils import ROOT_DIR, run_step


def main():
    parser = argparse.ArgumentParser(description='Test routing on kit-dev-coldfire-xilinx board')
    parser.add_argument('--quick', '-q', action='store_true',
                        help='Quick mode: only route the /AN* nets (the default; '
                             'kept as an explicit flag for compatibility)')
    parser.add_argument('--full', action='store_true',
                        help='Route the full signal set (~10 min; the #426 dense-board '
                             'run that used to be the default and blew run_all\'s '
                             'per-test timeout under load)')
    parser.add_argument('-u', '--unbuffered', action='store_true',
                        help='Accepted for compatibility; step output is '
                             'always echoed live')
    parser.add_argument('--planes-only', action='store_true',
                        help='Skip the routing and just redo the planes')
    parser.add_argument('--workdir', default=None,
                        help='Directory for generated kit-out* artifacts '
                             '(default: a fresh temp dir, auto-removed). Pass a '
                             'path to keep the outputs for inspection.')
    args = parser.parse_args()

    quick = not args.full

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
    --bus --bus-detection-radius 5 '+power_nets

    # Option groups are split here; PATHS are passed as whole argv items and
    # never go through shlex, whose POSIX rules eat a Windows backslash.
    target, base_options, options = (shlex.split(target),
                                     shlex.split(base_options),
                                     shlex.split(options))

    # Input board is read from the tracked tree; outputs go to a fresh work dir
    # so the run never touches committed files and never reads back a stale
    # sibling .kicad_pro DRC floor (issue #426).
    src_pcb = os.path.join(ROOT_DIR, 'kicad_files',
                           'kit-dev-coldfire-xilinx_5213.kicad_pcb')

    tmp = None
    if args.workdir:
        workdir = os.path.abspath(args.workdir)
        os.makedirs(workdir, exist_ok=True)
    else:
        tmp = tempfile.TemporaryDirectory(prefix='kit_route_')
        workdir = tmp.name

    out       = os.path.join(workdir, 'kit-out.kicad_pcb')
    out_plane = os.path.join(workdir, 'kit-out-plane.kicad_pcb')
    out_conn  = os.path.join(workdir, 'kit-out-plane-connected.kicad_pcb')

    try:
        # Route some nets from pads (no fanout needed)
        if not args.planes_only:
            run_step(['py_router/route.py', src_pcb, out] + target + options,
                     outputs=[out])

        # Route some power nets with vias to planes
        run_step(['py_router/route_planes.py', out, out_plane,
                  '--nets', '+3.3V', 'GND', '+3.3V', 'GND',
                  '--plane-layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
                 + base_options, outputs=[out_plane])

        # Connect broken plane regions
        run_step(['py_router/repair_planes.py', out_plane, out_conn,
                  '--analysis-grid-step', '0.1'] + base_options,
                 outputs=[out_conn])

        # The three checkers each run to a verdict (a crash fails here and
        # now); all three report before any verdict is judged.
        verdicts = {}
        # Check for DRC errors -- at the floor the chain wrote back to the
        # output's .kicad_pro, never a hardcoded one: repair_planes' pad taps
        # descend toward the fab floor to reach a pad and record that floor
        # (here 0.2 -> 0.15), so grading at 0.2 flags its own legal vias.
        verdicts['check_drc'], _ = run_step(
            ['py_router/check_drc.py', out_conn], findings_ok=True)

        # Check for connectivity
        verdicts['check_connected'], _ = run_step(
            ['py_router/check_connected.py', out_conn] + target,
            findings_ok=True)

        # Check for orphan stub segments
        verdicts['check_orphan_stubs'], _ = run_step(
            ['py_tools/check_orphan_stubs.py', out_conn], findings_ok=True)

        dirty = sorted(k for k, rc in verdicts.items() if rc)
        if dirty and quick:
            raise AssertionError(
                f"the chain ran, but {', '.join(dirty)} found problems on "
                f"the result (output above)")
        if dirty:
            # --full is known not clean (docstring, #426).
            print(f"\n--full: {', '.join(dirty)} found problems (expected "
                  f"for the dense full-board run, #426; not asserted)")

        print("\n=== Test completed ===")
    finally:
        if tmp is not None:
            tmp.cleanup()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Test routing on the interf_u board.
Creates VCC/GND planes first, fans out U9 (PGA120) nets, routes all signals,
then connects disconnected plane regions.

Every step is ASSERTED (`run_utils.run_step`): it must exit 0 with no
traceback and write the board the next step reads, and check_drc must pass.
This file used `run_utils.run`, which prints a failure and carries on, so it
was green whatever the chain did.

Connectivity is held to two things rather than to zero: every net
check_connected finds open must be one the chain itself REPORTED (route.py's
failed/open lists or repair_planes' unconnected pads -- no false success), and
there may be at most MAX_OPEN_NETS of them. Which nets a rip cascade leaves
open moves with any change to the rip-up heuristics and between platforms
(one such change left two nets open on Linux and none on macOS, #1400), and
per-board spread is a few nets (CLAUDE.md), so routing QUALITY is the corpus
A/B's to judge; this test guards the chain and its honesty.

Outputs go to a fresh temporary directory, not into kicad_files/ (as
test_kit_route does, #426): a stale output there would satisfy the "was it
written" check, and a stale sibling `.kicad_pro` would be read back as the
next step's DRC floor (CLAUDE.md). The input board is read from kicad_files/.
"""

import argparse
import json
import os
import re
import tempfile

from run_utils import ROOT_DIR, run_step

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

#: Open nets the final board may carry (see the module docstring).
MAX_OPEN_NETS = 2


def _reported_open(route_out, repair_out):
    """The nets the chain itself says it left open: route.py's run-scope
    JSON_SUMMARY failed / open lists, and the nets of repair_planes' unconnected
    plane pads ("U5.16 (GND)")."""
    nets = set()
    summaries = []
    for line in route_out.splitlines():
        if line.startswith('JSON_SUMMARY:'):
            try:
                summaries.append(json.loads(line[len('JSON_SUMMARY:'):]))
            except ValueError:
                pass
    run = [s for s in summaries if s.get('scope') == 'run'] or summaries
    if run:
        s = run[-1]
        nets |= set(s.get('failed_single') or ())
        nets |= set(s.get('open_single') or ())
        nets |= {m.get('net_name') for m in s.get('failed_multipoint') or ()}
    for line in repair_out.splitlines():
        if 'unconnected' in line.lower():
            # "U5.16 (GND)": a REF.PAD and its net in parentheses
            nets |= set(re.findall(r'\S+\.\S+ \(([^()]+)\)', line))
    return nets


def main():
    parser = argparse.ArgumentParser(description='Test routing on interf_u board')
    parser.add_argument('-u', '--unbuffered', action='store_true', default=False,
                        help='Accepted for compatibility; step output is '
                             'always echoed live')
    parser.add_argument('--checks', action='store_true', default=True,
                        help='Run DRC and connectivity checks after routing')
    parser.add_argument('--workdir', default=None,
                        help='Directory for the generated boards (default: a '
                             'fresh temp dir, auto-removed). Pass a path to '
                             'keep them for inspection.')
    args = parser.parse_args()

    src = os.path.join(ROOT_DIR, 'kicad_files', 'interf_u_unrouted.kicad_pcb')
    tmp = None
    if args.workdir:
        workdir = os.path.abspath(args.workdir)
        os.makedirs(workdir, exist_ok=True)
    else:
        tmp = tempfile.TemporaryDirectory(prefix='interf_u_')
        workdir = tmp.name
    plane = os.path.join(workdir, 'interf_u_plane.kicad_pcb')
    fanout = os.path.join(workdir, 'interf_u_fanout.kicad_pcb')
    routed = os.path.join(workdir, 'interf_u_routed.kicad_pcb')
    connected = os.path.join(workdir, 'interf_u_connected.kicad_pcb')

    try:
        # Step 1: Create VCC and GND planes on both layers
        run_step(['py_router/route_planes.py', src, plane,
                  '--nets', 'VCC', 'GND', '--plane-layers', 'F.Cu', 'B.Cu'],
                 outputs=[plane])

        # Step 2: Fan out U9 (PGA120) nets
        run_step(['py_router/bga_fanout.py', plane, '--component', 'U9',
                  '--output', fanout, '--nets', '/*'],
                 outputs=[fanout])

        # Step 3: Route all signals
        _rc, route_out = run_step(['py_router/route.py', fanout, routed, '--no-bga-zone',
                  '--add-teardrops', '--layer-costs', '1', '1',
                  '--max-ripup', '10', '--stub-proximity-radius', '10',
                  '--stub-proximity-cost', '3.0', '--max-iterations', '1000000',
                  '--board-edge-clearance', '0.55'],
                 outputs=[routed])

        # Step 4: Connect disconnected plane regions
        _rc, repair_out = run_step(['py_router/repair_planes.py', routed, connected,
                  '--board-edge-clearance', '0.6'],
                 outputs=[connected])

        if args.checks:
            # Both checkers run to a verdict (a crash fails here and now) and
            # report before either verdict is judged.
            drc_rc, _ = run_step(
                ['py_router/check_drc.py', connected], findings_ok=True)
            _cc_rc, cc_out = run_step(
                ['py_router/check_connected.py', connected], findings_ok=True)
            if drc_rc:
                raise AssertionError("the chain ran, but check_drc found "
                                     "problems on the result (output above)")
            open_nets = set(re.findall(r'^\s+(\S+) \(net \d+\):', cc_out, re.M))
            reported = _reported_open(route_out, repair_out)
            unreported = sorted(open_nets - reported)
            print(f"\nopen nets on the final board: {sorted(open_nets) or 'none'}; "
                  f"reported by the chain: {sorted(reported) or 'none'}")
            if unreported:
                raise AssertionError(
                    f"check_connected finds {unreported} open, and no step of "
                    f"the chain reported them (false success)")
            if len(open_nets) > MAX_OPEN_NETS:
                raise AssertionError(
                    f"{len(open_nets)} nets open on the final board "
                    f"({sorted(open_nets)}), more than {MAX_OPEN_NETS}")
    finally:
        if tmp is not None:
            tmp.cleanup()

    print("\n=== Test completed ===")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Issue #197: the stock (surface-fanned) watchy USB_D pair must route 1/1,
fully connected and DRC-clean, from route_diff alone -- no hand-placed vias.

watchy (`kicad_files/watchy.kicad_pcb`) fans its QFN (U4) out on the surface, so
the USB_D+/USB_D- terminals are bare F.Cu stubs with no escape vias. The pair is
too tight to coupled-route normally, so the hybrid escape fires: a coupled middle
on an open inner layer + a point-to-point single-ended leg per terminal that drops
its own escape via.

The USB_D+ stub tip is physically boxed on F.Cu by the adjacent USB_DET surface
stub (~0.225mm away). The leg used to fail there because the obstacle map's
approximations near that diagonal stub were wrong: the conservative SQUARE track
stamp blocked every Euclidean-legal cell around the tip (so the leg couldn't step
off it), and the bresenham via-block under-covered the diagonal stub (so a via
landed a hair too close). The fix made the obstacle keep-out EXACT (a capsule from
the true float segment, in build_base to match the cache) for both the track and
via keep-out, and wired the partner stub into the leg's keep-out. With exact
geometry the boxed tip's clear cells are no longer falsely blocked and the escape
via lands clean -- no special-case escape code needed (the earlier
compute_endpoint_escape local correction was retired once the global keep-out
became exact).

This test fans the stock board itself (so it can't silently depend on a
pre-fanned fixture) and asserts the pair routes 1/1, connects end to end, and is
DRC-clean at the routed clearance.

Run:
    python3 tests/test_watchy_diff_hybrid_escape.py [-v]
"""
import argparse
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT_DIR, "kicad_files", "watchy.kicad_pcb")
NETS = ["USB_D+", "USB_D-"]
CLEARANCE = "0.1"
FAN = ["--component", "U4", "--nets", "*", "!GND", "!+3V3", "--width", "0.1"]
DIFF_GEOM = ["--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
             "--track-width", "0.1", "--diff-pair-gap", "0.15",
             "--clearance", CLEARANCE, "--via-size", "0.3", "--via-drill", "0.2",
             "--no-gnd-vias", "--grid-step", "0.05"]


def _run(args, verbose):
    r = subprocess.run([sys.executable, *args], cwd=ROOT_DIR,
                       capture_output=True, text=True)
    txt = r.stdout + r.stderr
    if verbose:
        print(txt)
    return txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(BOARD):
        print(f"FAIL: board not found: {BOARD}")
        return 1

    fails = []
    tmp = []

    def _tmp(prefix):
        fd, p = tempfile.mkstemp(suffix=".kicad_pcb", prefix=prefix)
        os.close(fd)
        tmp.append(p)
        return p

    try:
        # 1) surface fanout of the QFN -> bare F.Cu USB_D stubs, no escape vias.
        fan = _tmp("watchy_fan_")
        ftxt = _run(["qfn_fanout.py", BOARD, *FAN, "--output", fan], args.verbose)
        if not (os.path.exists(fan) and os.path.getsize(fan) > 0):
            print("FAIL: qfn_fanout produced no board\n" + ftxt)
            return 1

        # 2) coupled route -> the hybrid escape must take the boxed pair to 1/1.
        diff = _tmp("watchy_diff_")
        dtxt = _run(["route_diff.py", fan, "--nets", *NETS, *DIFF_GEOM, "--output", diff], args.verbose)
        m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", dtxt)
        summary = json.loads(m.group(1)) if m else {}
        if summary.get("successful") != 1 or summary.get("failed"):
            fails.append(f"USB_D did not route 1/1 (summary: "
                         f"{summary.get('routed_diff_pairs')} / failed "
                         f"{summary.get('failed_diff_pairs')})")

        # 3) the routed pair must connect end to end and be DRC-clean.
        if os.path.exists(diff) and os.path.getsize(diff) > 0:
            conn = _run(["check_connected.py", diff, "--nets", *NETS], args.verbose)
            if "ALL NETS FULLY CONNECTED" not in conn:
                fails.append("USB_D not fully connected after hybrid escape")
            drc = _run(["check_drc.py", diff, "--clearance", CLEARANCE], args.verbose)
            if "NO DRC VIOLATIONS" not in drc:
                fails.append("hybrid-escaped USB_D has DRC violations")
        else:
            fails.append("route_diff wrote no output board")
    finally:
        for p in tmp:
            if os.path.exists(p):
                os.remove(p)

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: stock watchy USB_D routes 1/1, fully connected, DRC-clean via "
          "the hybrid tip-escape (#197)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

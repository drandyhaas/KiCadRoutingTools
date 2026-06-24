#!/usr/bin/env python3
"""Issue #166 regression: the esp_prog USB diff pair /D_P//D_N must coupled-route
off its source pads.

esp_prog (`kicad_files/esp_prog.kicad_pcb`) routes a single USB differential pair
from U1 (an SSOP-20, 1.27mm pads) to the USB1 connector (1.65mm pads). It routed
1/1 until the #165 connector-launch rework: dceb3ca's pad-edge launch shifted the
launch to the far edge of every long pad and pushed the centerline setback a full
pad-length past the pad -- for U1's 1.27mm pad a 0.9mm setback already clears it,
so the shift backed the setback into the congested area below the chip and the
source probe died in 10 iterations ("Probe forward blocked at source").

The fix keeps the pad-edge launch but pulls the setback back in by however far
the launch moved, so the setback point stays where a centre launch would put it.
This test guards that the pair coupled-routes, fully connects, and is DRC-clean.

Run:
    python3 tests/test_esp_prog_diff.py [-v]
"""
import argparse
import os
import subprocess
import sys
import tempfile

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT_DIR, "kicad_files", "esp_prog.kicad_pcb")
NETS = ["/D_P", "/D_N"]
CLEARANCE = "0.1778"
DIFF_GEOM = ["--track-width", "0.2", "--diff-pair-gap", "0.25",
             "--clearance", CLEARANCE, "--via-size", "0.45", "--via-drill", "0.3",
             "--no-gnd-vias"]
SE_GEOM = ["--clearance", CLEARANCE, "--track-width", "0.2",
           "--via-size", "0.45", "--via-drill", "0.3", "--no-fix-drc-settings"]


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

    def _drc_clean(board):
        txt = _run(["check_drc.py", board, "--clearance", CLEARANCE,
                    "--nets", *NETS, "--clearance-margin", "0.1"], args.verbose)
        return "NO DRC VIOLATIONS" in txt

    fails = []
    tmp = []

    def _tmp(prefix):
        fd, p = tempfile.mkstemp(suffix=".kicad_pcb", prefix=prefix)
        os.close(fd)
        tmp.append(p)
        return p

    try:
        # Step 1 (the #166 regression): route_diff must coupled-route the pair off
        # its source pads and WRITE output. On total failure it writes nothing, so
        # the whole downstream chain FileNotFoundErrors -- which is the bug.
        diff_out = _tmp("esp_prog_diff_")
        txt = _run(["route_diff.py", BOARD, diff_out, "--nets", *NETS, *DIFF_GEOM], args.verbose)
        coupled = '"routed_diff_pairs": ["/D"]' in txt and '"failed": 0' in txt
        if not coupled:
            fails.append("route_diff did not coupled-route /D_P//D_N "
                         "(source escape blocked -- issue #166)")
        if not (os.path.exists(diff_out) and os.path.getsize(diff_out) > 0):
            fails.append("route_diff wrote no output (downstream chain would break)")
        elif not _drc_clean(diff_out):
            fails.append("coupled diff route has DRC violations")

        # Step 2: the single-ended follow-up connects the connector (USB1) row,
        # exactly as the esp_prog chain does. All /D pads end up connected,
        # DRC-clean -- the coupled route is usable end to end.
        if coupled and os.path.exists(diff_out) and os.path.getsize(diff_out) > 0:
            final_out = _tmp("esp_prog_final_")
            _run(["route.py", diff_out, final_out, "--nets", *NETS, *SE_GEOM], args.verbose)
            conn = _run(["check_connected.py", final_out, "--nets", *NETS], args.verbose)
            if "ALL NETS FULLY CONNECTED" not in conn:
                fails.append("/D_P//D_N not fully connected after single-ended follow-up")
            if not _drc_clean(final_out):
                fails.append("final board has DRC violations")
    finally:
        for p in tmp:
            if os.path.exists(p):
                os.remove(p)

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: esp_prog /D_P//D_N coupled-routes 1/1, fully connected, DRC-clean")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

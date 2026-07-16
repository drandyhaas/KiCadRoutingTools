#!/usr/bin/env python3
"""Issue #167: route_diff must never leave the pipeline without an output board.

On set-3 board watchy (`kicad_files/watchy.kicad_pcb`) the USB_D+/USB_D- pair sits
on a dense fine-pitch connector and genuinely cannot be coupled-routed -- the
terminal/launch escape exhausts its whole setback/direction ladder for both
source and target. That is acceptable (the downstream single-ended pass picks the
pair up), but route_diff used to write NO output file in that case, so the rest of
the chain (signal -> planes -> repair) FileNotFoundError'd and produced no board.

This guards that route_diff still writes the board THROUGH (unchanged) when it
can't coupled-route anything, so the pipeline continues. It does NOT require the
pair to route -- only that an output board is always produced.

Run:
    python3 tests/test_watchy_diff_passthrough.py [-v]
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
GEOM = ["--layers", "F.Cu", "In1.Cu", "In2.Cu", "B.Cu",
        "--track-width", "0.2", "--diff-pair-gap", "0.25", "--clearance", "0.1",
        "--via-size", "0.5", "--via-drill", "0.3", "--no-gnd-vias"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(BOARD):
        print(f"FAIL: board not found: {BOARD}")
        return 1

    fails = []
    fd, out = tempfile.mkstemp(suffix=".kicad_pcb", prefix="watchy_diff_")
    os.close(fd)
    os.remove(out)  # route_diff must (re)create it
    try:
        r = subprocess.run(
            [sys.executable, "route_diff.py", BOARD, "--nets", *NETS, *GEOM, "--output", out],
            cwd=ROOT_DIR, capture_output=True, text=True)
        txt = r.stdout + r.stderr
        if args.verbose:
            print(txt)

        # route_diff must exit cleanly (a single unroutable pair is not a crash)
        # and ALWAYS leave an output board for the downstream chain.
        if r.returncode != 0:
            fails.append(f"route_diff exited {r.returncode} (should be 0)")
        if not (os.path.exists(out) and os.path.getsize(out) > 0):
            fails.append("route_diff wrote NO output board (pipeline would break -- issue #167)")

        # This board genuinely can't be coupled-routed; the point is the board is
        # still written through, not that the pair routes.
        m = re.search(r"JSON_SUMMARY:\s*(\{.*\})", txt)
        if m:
            summary = json.loads(m.group(1))
            se_fb = summary.get("single_ended_fallback") or {}
            if summary.get("routed_diff_pairs"):
                # If a future engine CAN route it, that's fine too -- only assert
                # output exists (already checked above).
                pass
            elif se_fb.get("routed") or se_fb.get("partial"):
                # The single-ended member fallback (Class 2) connected members
                # in-run: real copper was written, so no passthrough is needed.
                pass
            elif "wrote board through" not in txt.lower() and "board through" not in txt.lower():
                fails.append("pair not routed yet no board-through message was printed")

        # route.py is the other step in these chains; it too must leave a board
        # when it can't route a thing (issue #167). Force a total failure with an
        # impossible clearance and assert an output board is still produced.
        rp = out + ".rp.kicad_pcb"
        if os.path.exists(rp):
            os.remove(rp)
        r2 = subprocess.run(
            [sys.executable, "route.py", BOARD, rp, "--nets", "Net-(U4-LNA_IN{slash}RF)",
             "--clearance", "5.0", "--track-width", "0.1", "--via-size", "0.5",
             "--via-drill", "0.3", "--no-bga-zone", "--no-fix-drc-settings"],
            cwd=ROOT_DIR, capture_output=True, text=True)
        if args.verbose:
            print(r2.stdout + r2.stderr)
        if not (os.path.exists(rp) and os.path.getsize(rp) > 0):
            fails.append("route.py wrote NO output board when it routed nothing "
                         "(pipeline would break -- issue #167)")
        if os.path.exists(rp):
            os.remove(rp)
    finally:
        if os.path.exists(out):
            os.remove(out)

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: route_diff always writes an output board (board-through on "
          "unroutable pair), so the pipeline never FileNotFoundErrors")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

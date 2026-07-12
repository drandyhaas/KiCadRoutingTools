#!/usr/bin/env python3
"""Issue #223: fanout escape stubs must never be laid below the fab minimum
track width.

usb_sniffer carried 45 TRACK-WIDTH violations -- a whole /T_USB_* bus emitted at
0.100mm against the board's 0.127mm 2-layer fab floor. Unlike clearance grazes,
the stub width is a parameter, not a search outcome, so it is trivially clamped
to max(fab_min_track, requested) at the fanout step -- the same fab-floor clamp
route.py / check_drc.py already apply.

This test covers both shared escape engines (called by CLI *and* GUI):

  1. generate_bga_fanout on a 2-layer board with --track-width below the floor
     -> every emitted stub is at the floor, none below.
  2. generate_qfn_fanout, same.
  3. A 4-layer board whose requested width is ABOVE the floor is left untouched
     (the clamp only ever raises, never lowers).

Run:  python3 tests/test_fanout_track_width_floor.py [-v]
"""
import argparse
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import parse_kicad_pcb
from list_nets import fab_floors
from bga_fanout import generate_bga_fanout
from qfn_fanout import generate_qfn_fanout

BGA_BOARD = os.path.join(ROOT, "kicad_files", "interf_u_plane.kicad_pcb")        # 2-layer, PGA120 grid
QFN_BOARD = os.path.join(ROOT, "kicad_files", "qfn_diffpair_escape.kicad_pcb")    # 2-layer QFN
BGA_4L = os.path.join(ROOT, "kicad_files", "ulx3s.kicad_pcb")                     # 4-layer BGA


def _run(fn):
    """Run a fanout entry point with its noisy stdout captured."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = fn()
    return out, buf.getvalue()


def _widths(tracks):
    return sorted({round(t['width'], 4) for t in tracks})


def test_bga_clamps_below_floor(verbose):
    fails = []
    if not os.path.exists(BGA_BOARD):
        print(f"  [SKIP] BGA board absent: {BGA_BOARD}")
        return fails
    pcb = parse_kicad_pcb(BGA_BOARD)
    floor = fab_floors(len(pcb.board_info.copper_layers))['track_width']  # 0.127 (2-layer)
    fp = pcb.footprints["U9"]
    (tracks, _va, _vr, _f), log = _run(lambda: generate_bga_fanout(
        fp, pcb, layers=["F.Cu", "B.Cu"], track_width=0.1, clearance=0.25,
        via_size=0.45, via_drill=0.2))
    if not tracks:
        fails.append("bga: produced no tracks (test exercises nothing)")
        return fails
    thin = [w for w in _widths(tracks) if w < floor - 1e-9]
    if thin:
        fails.append(f"bga: stub widths below floor {floor}: {thin}")
    if "issue #223" not in log:
        fails.append("bga: clamp did not announce itself")
    if verbose and not fails:
        print(f"  bga U9: requested 0.1 -> stubs at {_widths(tracks)} (floor {floor})  OK")
    return fails


def test_qfn_clamps_below_floor(verbose):
    fails = []
    if not os.path.exists(QFN_BOARD):
        print(f"  [SKIP] QFN board absent: {QFN_BOARD}")
        return fails
    pcb = parse_kicad_pcb(QFN_BOARD)
    floor = fab_floors(len(pcb.board_info.copper_layers))['track_width']  # 0.127 (2-layer)
    fp = pcb.footprints["U1"]
    (tracks, _v, _f), log = _run(lambda: generate_qfn_fanout(
        fp, pcb, layer="F.Cu", track_width=0.1, clearance=0.15, grid_step=0.05))
    if not tracks:
        fails.append("qfn: produced no tracks (test exercises nothing)")
        return fails
    thin = [w for w in _widths(tracks) if w < floor - 1e-9]
    if thin:
        fails.append(f"qfn: stub widths below floor {floor}: {thin}")
    if "issue #223" not in log:
        fails.append("qfn: clamp did not announce itself")
    if verbose and not fails:
        print(f"  qfn U1: requested 0.1 -> stubs at {_widths(tracks)} (floor {floor})  OK")
    return fails


def test_above_floor_untouched(verbose):
    """The clamp only ever raises -- a width above the floor is left as-is."""
    fails = []
    if not os.path.exists(BGA_4L):
        print(f"  [SKIP] 4-layer board absent: {BGA_4L}")
        return fails
    pcb = parse_kicad_pcb(BGA_4L)
    floor = fab_floors(len(pcb.board_info.copper_layers))['track_width']  # 0.0889 (4-layer)
    req = 0.12  # comfortably above the 4-layer floor
    fp = pcb.footprints["U1"]
    (tracks, _va, _vr, _f), log = _run(lambda: generate_bga_fanout(
        fp, pcb, layers=["F.Cu", "In1.Cu", "In2.Cu", "B.Cu"], track_width=req,
        clearance=0.1, via_size=0.5, via_drill=0.3))
    if "issue #223" in log:
        fails.append(f"4L: clamp fired on a width ({req}) already above floor {floor}")
    stub_ws = [w for w in _widths(tracks) if abs(w - req) < 1e-6]
    if tracks and not stub_ws:
        fails.append(f"4L: no stub at the requested {req} (widths {_widths(tracks)})")
    if verbose and not fails:
        print(f"  4L U1: requested {req} >= floor {floor}, left untouched  OK")
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    print("=== fanout clamps escape stubs to the fab track-width floor ===")
    fails = test_bga_clamps_below_floor(args.verbose)
    fails += test_qfn_clamps_below_floor(args.verbose)
    fails += test_above_floor_untouched(args.verbose)

    if fails:
        print("\nFAIL:\n  " + "\n  ".join(fails))
        return 1
    print("\nPASS: BGA and QFN fanout clamp sub-floor escape stubs up to the fab "
          "minimum; above-floor widths left untouched")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

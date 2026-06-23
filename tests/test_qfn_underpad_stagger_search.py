#!/usr/bin/env python3
"""Issue #161 follow-up: the under-pad escape's alternative-stagger search lifts
the escape fraction on a crowded edge by re-staggering legs that the simple
nearest-first greedy drops.

Board (kicad_files/qfn_underpad_coupling.kicad_pcb): a stock HVQFN-32 with all
eight right-edge pads on their own nets (SIG0..SIG7, a byte-wide bus) and a
FOREIGN neighbour track crowding the OUTER lane of the lower-right pads. The
nearest-first stagger paints one lower leg into a corner; trying a different
stagger configuration (reversed order / per-leg in-vs-out bias) frees it. The
board is a synthetic, DRC-clean reduction built once with KiCad's pcbnew; it is
checked in, so this test needs no pcbnew.

Asserts:
  - with the default config ONLY (env flag) the side drops >= 1 leg, and
  - the full search escapes ALL eight legs, DRC-clean at the routed clearance.
So the search, not luck, is what rescues the leg.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kicad_parser import parse_kicad_pcb
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout import generate_qfn_fanout
from check_drc import run_drc

BOARD = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "kicad_files", "qfn_underpad_coupling.kicad_pcb")
VIA_SIZE, VIA_DRILL, CLEARANCE, GRID = 0.45, 0.2, 0.12, 0.05
N_LEGS = 8


def _fanout(out_path):
    pcb = parse_kicad_pcb(BOARD)
    tracks, vias, dropped = generate_qfn_fanout(
        pcb.footprints["U1"], pcb, net_filter=["SIG*"], layer="F.Cu",
        track_width=0.1, clearance=CLEARANCE, grid_step=GRID,
        escape_method="underpad", via_size=VIA_SIZE, via_drill=VIA_DRILL,
        allow_via_in_pad=True)
    net_id_to_name = {nid: net.name for nid, net in pcb.nets.items()}
    add_tracks_and_vias_to_pcb(BOARD, out_path, tracks, vias,
                               net_id_to_name=net_id_to_name)
    return vias, dropped


def main():
    if not os.path.exists(BOARD):
        print(f"FAIL: coupling board missing ({BOARD})")
        return 1

    import tempfile
    fails = []
    with tempfile.TemporaryDirectory() as tmp:
        # check_sizes=False: this fanout intentionally routes 0.1mm escapes, below
        # the 2-layer fab floor -- the test asserts clearance/connectivity, not the
        # fab-width floor that issue #176 added.
        if run_drc(BOARD, clearance=CLEARANCE, quiet=True, check_sizes=False):
            fails.append("input board is not DRC-clean")

        # Default config only: the simple nearest-first stagger drops a leg.
        os.environ["QFN_UNDERPAD_NO_ALT_STAGGER"] = "1"
        try:
            _v_single, d_single = _fanout(os.path.join(tmp, "single.kicad_pcb"))
        finally:
            del os.environ["QFN_UNDERPAD_NO_ALT_STAGGER"]
        if not d_single:
            fails.append("expected the default-only config to drop a leg "
                         "(board no longer exercises the stagger search)")

        # Full search: every leg escapes, DRC-clean.
        full_out = os.path.join(tmp, "full.kicad_pcb")
        v_full, d_full = _fanout(full_out)
        if d_full:
            fails.append(f"alternative-stagger search should escape every leg, "
                         f"dropped={d_full}")
        if len(v_full) != N_LEGS:
            fails.append(f"expected {N_LEGS} escape vias, got {len(v_full)}")
        if len(d_full) >= len(d_single):
            fails.append(f"search did not improve escape fraction "
                         f"(single dropped {len(d_single)}, full dropped {len(d_full)})")
        if run_drc(full_out, clearance=CLEARANCE, quiet=True, check_sizes=False):
            fails.append("full-search output is not DRC-clean")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print(f"PASS: default stagger drops {len(d_single)} leg(s); the "
          f"alternative-stagger search escapes all {N_LEGS}, DRC-clean")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

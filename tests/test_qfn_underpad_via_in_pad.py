#!/usr/bin/env python3
"""Issue #161 (reopen): qfn_fanout under-pad escape must not land a via on a
neighbour's track, and --allow-via-in-pad must escape every fanned pad.

Uses the checked-in synthetic repro board (kicad_files/qfn_diffpair_escape.kicad_pcb):
a stock HVQFN-32 with the pair to escape (DP1) on the outer right edge, a routed
neighbour pair (DP2) whose diagonal escape sweeps into DP1's escape lane, and a
FOREIGN track crossing that lane. The neighbour copper is on the chip's OWN other
nets, which is exactly what let the old via-drop place a via on a neighbour's
track and short it.

The board is a synthetic, DRC-clean reduction of the reporter's setup (built
once with KiCad's pcbnew + the HVQFN-32-1EP_5x5mm_P0.5mm_EP3.1x3.1mm footprint);
it is checked in, so this test needs no pcbnew. To regenerate/tweak it, see the
build recipe in issue #161 (edgehero's repro) -- DP1 outer pair, DP2 neighbour
pair routed, FOREIGN track in DP1's escape lane but clear of all pads.

Two scenarios, both must be DRC-clean at the routed clearance:
  A) plain under-pad escape: the outward via is boxed in by the neighbour
     track/pad, so it is DROPPED (fail cleanly) rather than shorted.
  B) --allow-via-in-pad: the outer leg staggers INWARD onto its own pad instead
     of outward into the neighbour, so BOTH halves escape, DRC-clean.

The board ships a via at (14.65, 9.75) shorting DP2_N and FOREIGN under the old
engine, so this is a real regression guard.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from kicad_parser import parse_kicad_pcb
from kicad_writer import add_tracks_and_vias_to_pcb
from qfn_fanout import generate_qfn_fanout
from check_drc import run_drc

BOARD = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "kicad_files", "qfn_diffpair_escape.kicad_pcb")
VIA_SIZE, VIA_DRILL, CLEARANCE, GRID = 0.45, 0.2, 0.15, 0.05


def _fanout(out_path, allow_via_in_pad):
    pcb = parse_kicad_pcb(BOARD)
    u1 = pcb.footprints["U1"]
    tracks, vias, dropped = generate_qfn_fanout(
        u1, pcb, net_filter=["DP1*"], layer="F.Cu",
        track_width=0.1, clearance=CLEARANCE, grid_step=GRID,
        escape_method="underpad", via_size=VIA_SIZE, via_drill=VIA_DRILL,
        allow_via_in_pad=allow_via_in_pad)
    net_id_to_name = {nid: net.name for nid, net in pcb.nets.items()}
    add_tracks_and_vias_to_pcb(BOARD, out_path, tracks, vias,
                               net_id_to_name=net_id_to_name)
    return tracks, vias, dropped


def _drc_clean(out_path):
    # check_sizes=False: the fanout intentionally routes 0.1mm escapes (below the
    # 2-layer fab floor); this test asserts clearance, not the issue #176 fab floor.
    return not run_drc(out_path, clearance=CLEARANCE, quiet=True, check_sizes=False)


def main():
    if not os.path.exists(BOARD):
        print(f"FAIL: example board missing ({BOARD})")
        return 1

    import tempfile
    fails = []
    with tempfile.TemporaryDirectory() as tmp:
        # The input board itself must be DRC-clean so any violation we find is
        # one the fanout introduced.
        if not _drc_clean(BOARD):
            fails.append("input board is not DRC-clean at 0.15")

        # Scenario A: plain under-pad escape fails cleanly (no shorting via).
        a_out = os.path.join(tmp, "plain.kicad_pcb")
        _ta, va, da = _fanout(a_out, allow_via_in_pad=False)
        if not _drc_clean(a_out):
            fails.append("plain under-pad escape shipped a DRC violation "
                         "(via landed on a neighbour's track -- issue #161)")
        if "DP1_P" not in (da or []):
            fails.append(f"plain escape should drop the boxed-in DP1_P, got dropped={da}")

        # Scenario B: via-in-pad escapes BOTH halves, DRC-clean.
        b_out = os.path.join(tmp, "vip.kicad_pcb")
        _tb, vb, db = _fanout(b_out, allow_via_in_pad=True)
        if db:
            fails.append(f"via-in-pad should escape every pad, dropped={db}")
        if len(vb) != 2:
            fails.append(f"via-in-pad: expected 2 escape vias, got {len(vb)}")
        if {v["net_id"] for v in vb} != {p.net_id for p in
                parse_kicad_pcb(BOARD).footprints["U1"].pads
                if p.net_name and "DP1" in p.net_name}:
            fails.append("via-in-pad: both DP1 halves should escape")
        for v in vb:
            if v["layers"] != ["F.Cu", "B.Cu"]:
                fails.append(f"escape via not through-hole: {v['layers']}")
        if not _drc_clean(b_out):
            fails.append("via-in-pad under-pad escape shipped a DRC violation")

    if fails:
        print("FAIL: " + "; ".join(fails))
        return 1
    print("PASS: plain escape drops the boxed-in via (DRC-clean); via-in-pad "
          "escapes both DP1 halves staggered inward (DRC-clean)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

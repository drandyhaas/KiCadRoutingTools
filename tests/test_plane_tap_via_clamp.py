#!/usr/bin/env python3
"""Fine-pitch plane taps shrink the via to fit the pad (issue #360).

A plane-repair tap that keeps the caller's full via cannot connect a fine-pitch
pad the via is too big for: a 0.6 mm via dropped in a 0.4 mm ball bulges past the
copper and, ringed by neighbours, finds no in-pad site at all. The fine-pitch
escalation now clamps the tap via to the pad (borrowing the underpad-fanout
`clamp_via_to_pad`, down to the fab via floor, escalating standard->advanced with
a warning) so a via-in-pad tap fits where a full via never did -- no manual
`--via-size` needed.

This isolates the CLAMP WIRING in tap_pad_with_escalation: a 0.4 mm pad ringed by
four foreign vias at 0.5 mm has NO in-pad site for a 0.6 mm via (its keep-out ring
grazes every neighbour) and no room to search a via out and trace back, so the
full-via tap fails; the clamped 0.4 mm via clears the same neighbours at the pad
centre and taps in-pad. (clamp_via_to_pad's own geometry is covered by
test_fanout_via_in_pad_clamp / test_qfn_underpad_via_in_pad.)

Run:  python3 tests/test_plane_tap_via_clamp.py [-v]
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import Pad, Via, PCBData, BoardInfo
from routing_config import GridRouteConfig
from plane_pad_tap import tap_pad_with_escalation, try_tap_pad


def _board():
    """A single 0.4 mm plane pad at (10,10) ringed by four foreign vias at 0.5 mm."""
    pad = Pad(component_ref="U1", pad_number="B2", global_x=10.0, global_y=10.0,
              local_x=0.0, local_y=0.0, size_x=0.4, size_y=0.4, shape="circle",
              layers=["F.Cu"], net_id=1, net_name="+3V3")
    foreign = [
        Via(x=10.5, y=10.0, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
        Via(x=9.5, y=10.0, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
        Via(x=10.0, y=10.5, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
        Via(x=10.0, y=9.5, size=0.3, drill=0.2, layers=["F.Cu", "B.Cu"], net_id=2),
    ]
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = (0.0, 0.0, 20.0, 20.0)
    pcb = PCBData(board_info=bi, nets={}, footprints={},
                  vias=foreign, segments=[], pads_by_net={1: [pad], 2: []})
    return pcb, pad


def _config():
    return GridRouteConfig(track_width=0.127, clearance=0.1, via_size=0.6,
                           via_drill=0.36, grid_step=0.05, board_edge_clearance=0.2,
                           hole_to_hole_clearance=0.2)


def run(verbose=False):
    pcb, pad = _board()
    cfg = _config()
    # A tight search radius so the only way to connect is a via INSIDE the pad --
    # there is no room to place a via outside the foreign ring and trace back.
    R = 0.25

    # 1. The full 0.6 mm via cannot tap the 0.4 mm pad at all (bulges + ringed in).
    full = try_tap_pad(pad, "F.Cu", 1, pcb, cfg, max_search_radius=R,
                       via_size=0.6, via_drill=0.36)
    assert not full.success, "full 0.6mm via should NOT tap the ringed 0.4mm pad"
    if verbose:
        print("  full via: no tap (as expected)")

    # 2. Escalation clamps the via to fit and taps in-pad.
    res = tap_pad_with_escalation(pad, "F.Cu", 1, pcb, cfg, max_search_radius=R,
                                  via_size=0.6, via_drill=0.36, fine_for_all=True)
    assert res.success, "clamped fine-pitch tap should connect the pad"
    assert res.via is not None, "expected a placed via-in-pad (not a reused trace)"
    assert res.via["size"] < 0.6 - 1e-9, \
        f"via should be shrunk below the caller's 0.6mm, got {res.via['size']}"
    assert res.via["size"] <= pad.size_x + 1e-9, \
        f"clamped via {res.via['size']} must fit the {pad.size_x}mm pad"
    # Lands inside the pad (via-in-pad: no trace needed).
    import math
    assert math.hypot(res.via["x"] - pad.global_x, res.via["y"] - pad.global_y) \
        <= pad.size_x / 2 + 1e-9, "via must land inside the pad copper"
    assert res.params_label == "fine"
    if verbose:
        print(f"  clamped via: size={res.via['size']} drill={res.via['drill']} "
              f"@({res.via['x']:.3f},{res.via['y']:.3f}) -> tapped in-pad")
    print("PASS: fine-pitch plane tap clamps the via to fit the pad (#360)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    run(args.verbose)

#!/usr/bin/env python3
"""Board-edge clamp for plane tap vias (min_copper_edge_clearance / #338).

The via-placement obstacle map blocks via CELLS within the board-edge keep-out,
but a via-in-pad is placed at the pad's true (off-grid) centre, which can sit a
fraction of a grid step closer to the edge than the blocked band -- slipping a
copper_edge_clearance violation past the grid-resolution map (verified on
lna3030's GND tap via at 0.486 mm vs a 0.5 mm rule).

clamp_tap_via_to_edge closes that gap with a float-exact clamp against the real
Edge.Cuts geometry, capped to keep the via inside the pad copper so the
via-in-pad connection holds. This test isolates the function on synthetic
rectangular boards.

Run:  python3 tests/test_plane_tap_edge_clamp.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from kicad_parser import Pad, PCBData, BoardInfo
from routing_config import GridRouteConfig
from routing_utils import point_in_pad_rect
from plane_pad_tap import clamp_tap_via_to_edge


def _bi(bounds=(0.0, 0.0, 20.0, 20.0)):
    bi = BoardInfo(layers={0: "F.Cu", 31: "B.Cu"}, copper_layers=["F.Cu", "B.Cu"])
    bi.board_bounds = bounds
    return bi


def _mkpcb(bounds=(0.0, 0.0, 20.0, 20.0)):
    return PCBData(board_info=_bi(bounds), nets={}, footprints={},
                   vias=[], segments=[], pads_by_net={})


def _cfg(edge=0.5, via=0.45):
    return GridRouteConfig(track_width=0.15, clearance=0.1, via_size=via,
                           via_drill=0.2, grid_step=0.1, board_edge_clearance=edge,
                           hole_to_hole_clearance=0.2)


def _pad(x, y, w=0.6, h=0.6, net=1, layer="F.Cu"):
    return Pad(component_ref="U1", pad_number="1", global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=w, size_y=h, shape="rect",
               layers=[layer], net_id=net, net_name="GND")


def _edist_rect(x, y, b):
    return min(x - b[0], b[2] - x, y - b[1], b[3] - y)


def test_violating_via_pulled_clear():
    """A via-in-pad at a pad centre that intrudes into the edge band is moved
    interior until its copper clears, staying inside the pad."""
    b = (0.0, 0.0, 20.0, 20.0)
    via_size = 0.45
    cfg = _cfg(edge=0.5, via=via_size)
    required = 0.5 + via_size / 2  # = 0.725
    # Pad centre 0.715 mm from the bottom edge -> via ring at 0.49 mm (violates),
    # but the 0.6 mm-tall pad reaches to y=1.015 so a via at y>=0.725 fits inside.
    pad = _pad(10.0, 0.715, w=0.6, h=0.6)
    pcb = _mkpcb(b)
    pos, moved = clamp_tap_via_to_edge((10.0, 0.715), pad, pcb, cfg, via_size)
    assert moved, "expected the violating via to move"
    assert _edist_rect(pos[0], pos[1], b) >= required - 1e-6, \
        f"via still in band: edist {_edist_rect(pos[0], pos[1], b):.4f} < {required}"
    assert point_in_pad_rect(pos[0], pos[1], pad, 1e-6), "via left the pad copper"
    print(f"  PASS: violating via {(10.0,0.715)} -> {tuple(round(v,4) for v in pos)} "
          f"(edist {_edist_rect(pos[0],pos[1],b):.4f} >= {required})")


def test_clearing_via_untouched():
    """A via already clearing the edge is returned unchanged."""
    b = (0.0, 0.0, 20.0, 20.0)
    cfg = _cfg(edge=0.5, via=0.45)
    pad = _pad(10.0, 10.0, w=0.6, h=0.6)
    pos, moved = clamp_tap_via_to_edge((10.0, 10.0), pad, pcb := _mkpcb(b), cfg, 0.45)
    assert not moved and pos == (10.0, 10.0), "interior via should be untouched"
    print("  PASS: interior via untouched")


def test_no_rule_is_inert():
    """No project edge rule (board_edge_clearance<=0) -> completely inert, even
    for a via hard against the edge (backward compatible)."""
    b = (0.0, 0.0, 20.0, 20.0)
    cfg = _cfg(edge=0.0, via=0.45)
    pad = _pad(10.0, 0.2, w=0.6, h=0.6)
    pcb = _mkpcb(b)
    pos, moved = clamp_tap_via_to_edge((10.0, 0.2), pad, pcb, cfg, 0.45)
    assert not moved and pos == (10.0, 0.2), "no edge rule must be inert"
    print("  PASS: inert when the project records no edge rule")


def test_in_band_pad_best_effort_no_fail():
    """A pad whose own copper is entirely inside the band (board's own design)
    cannot fully clear; the via is moved as far interior as the pad allows and
    the tap is NOT failed (mirrors the #338 in-band-pad reach exemption)."""
    b = (0.0, 0.0, 20.0, 20.0)
    via_size = 0.45
    cfg = _cfg(edge=0.5, via=via_size)
    # Tiny 0.3 mm pad centred 0.3 mm from the edge: even its inner edge (0.45 mm)
    # cannot reach the 0.725 mm the via needs. Best effort must still move it
    # interior (>= its start) and keep it on the pad, without raising.
    pad = _pad(10.0, 0.3, w=0.3, h=0.3)
    pcb = _mkpcb(b)
    start = (10.0, 0.3)
    pos, moved = clamp_tap_via_to_edge(start, pad, pcb, cfg, via_size)
    assert point_in_pad_rect(pos[0], pos[1], pad, 1e-6), "via left the (in-band) pad"
    assert _edist_rect(pos[0], pos[1], b) >= _edist_rect(*start, b) - 1e-9, \
        "in-band via moved the wrong way (toward the edge)"
    print(f"  PASS: in-band pad best-effort {start} -> "
          f"{tuple(round(v,4) for v in pos)} (no tap failure)")


def test_out_of_pad_via_untouched():
    """A via that is not inside the pad (grid-aligned external site) is left
    alone -- those clear the map's edge band by construction."""
    b = (0.0, 0.0, 20.0, 20.0)
    cfg = _cfg(edge=0.5, via=0.45)
    pad = _pad(10.0, 0.715, w=0.6, h=0.6)
    external = (10.0, 3.0)  # well outside the pad
    pos, moved = clamp_tap_via_to_edge(external, pad, _mkpcb(b), cfg, 0.45)
    assert not moved and pos == external, "non-in-pad via should be untouched"
    print("  PASS: out-of-pad via untouched")


if __name__ == "__main__":
    test_violating_via_pulled_clear()
    test_clearing_via_untouched()
    test_no_rule_is_inert()
    test_in_band_pad_best_effort_no_fail()
    test_out_of_pad_via_untouched()
    print("\nALL PASS: plane tap board-edge via clamp")

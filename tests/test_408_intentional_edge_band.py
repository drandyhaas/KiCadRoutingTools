#!/usr/bin/env python3
"""#408: the router reports the NET NAMES it intentionally routed into the board-
edge clearance band (in-band pads: edge connectors, edge-mounted parts) so a
grader can accept-by-design every copper_edge_clearance violation on those nets.

Covers compute_intentional_edge_band_nets (report-only) and asserts the shared
in-band detection still drives the #338 reach exemption identically after the
refactor. Pure Python, no pcbnew.
"""
import os
import sys

_TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS_DIR)
for p in (_ROOT, _TESTS_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

from kicad_parser import BoardInfo, Footprint
from synth import make_pad, make_pcb
from routing_config import GridRouteConfig, GridCoord
from obstacle_map import (compute_intentional_edge_band_nets,
                          _edge_band_pad_exemption)

EDGE_CLEAR = 0.5  # board copper-to-edge rule


def _board(pads):
    """A 20x20 mm rectangular board carrying the given pads (one footprint)."""
    fp = Footprint(reference='J3', footprint_name='conn', x=0, y=0,
                   rotation=0, layer='F.Cu', pads=pads)
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 20.0, 20.0))
    return make_pcb(footprints={'J3': fp}, board_info=bi)


def test_reports_in_band_net():
    # pad copper edge 0.2mm from the left edge (< 0.5 rule) -> in-band.
    edge = make_pad(net_id=7, x=0.45, y=10.0, ref='J3', num='15',
                    net_name='/RST0', size_x=0.5, size_y=0.5)
    # pad safely interior -> not reported.
    interior = make_pad(net_id=8, x=10.0, y=10.0, ref='J3', num='2',
                        net_name='/SIG', size_x=0.5, size_y=0.5)
    got = compute_intentional_edge_band_nets(_board([edge, interior]), EDGE_CLEAR)
    assert got == ['/RST0'], got


def test_dedup_and_sorted():
    # Two in-band pads on the same net collapse to one entry; output is sorted.
    a = make_pad(net_id=7, x=0.45, y=6.0, ref='J3', num='1', net_name='/RST0')
    b = make_pad(net_id=7, x=0.45, y=9.0, ref='J3', num='2', net_name='/RST0')
    c = make_pad(net_id=9, x=0.45, y=12.0, ref='J3', num='3', net_name='+5V')
    got = compute_intentional_edge_band_nets(_board([a, b, c]), EDGE_CLEAR)
    assert got == ['+5V', '/RST0'], got


def test_unconnected_in_band_pad_not_reported():
    # In-band but net_id 0: no routed copper to attribute a violation to.
    free = make_pad(net_id=0, x=0.45, y=10.0, ref='J3', num='9', net_name='')
    assert compute_intentional_edge_band_nets(_board([free]), EDGE_CLEAR) == []


def test_edge_honoring_board_reports_nothing():
    # Every pad interior -> the common case, empty list.
    p = make_pad(net_id=7, x=10.0, y=10.0, ref='J3', num='1', net_name='/A')
    assert compute_intentional_edge_band_nets(_board([p]), EDGE_CLEAR) == []


def test_off_board_pad_reported():
    # Pad center off the board (x < 0): copper crosses the edge -> its net reported.
    off = make_pad(net_id=7, x=-0.1, y=10.0, ref='J3', num='1', net_name='/A',
                   size_x=0.5, size_y=0.5)
    assert compute_intentional_edge_band_nets(_board([off]), EDGE_CLEAR) == ['/A']


def test_zero_edge_clearance_inert():
    edge = make_pad(net_id=7, x=0.1, y=10.0, ref='J3', num='1', net_name='/A')
    assert compute_intentional_edge_band_nets(_board([edge]), 0.0) == []


def test_report_matches_exemption_firing():
    """The report is non-empty exactly when the #338 reach exemption fires,
    and empty exactly when it does not -- both read the same in-band detection."""
    coord = GridCoord(0.1)
    track_edge = EDGE_CLEAR + 0.2 / 2  # clearance + track half-width

    edge = make_pad(net_id=7, x=0.45, y=10.0, ref='J3', num='15',
                    net_name='/RST0')
    pcb_in = _board([edge])
    assert compute_intentional_edge_band_nets(pcb_in, EDGE_CLEAR)
    assert _edge_band_pad_exemption(pcb_in, coord, EDGE_CLEAR, track_edge) is not None

    interior = make_pad(net_id=7, x=10.0, y=10.0, ref='J3', num='1',
                        net_name='/A')
    pcb_clear = _board([interior])
    assert compute_intentional_edge_band_nets(pcb_clear, EDGE_CLEAR) == []
    assert _edge_band_pad_exemption(pcb_clear, coord, EDGE_CLEAR, track_edge) is None


if __name__ == '__main__':
    fns = [v for k, v in sorted(globals().items()) if k.startswith('test_')]
    for fn in fns:
        fn()
        print(f"ok  {fn.__name__}")
    print(f"\nAll {len(fns)} tests passed.")

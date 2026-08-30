#!/usr/bin/env python3
"""#578: filter_already_routed's fragment gate.

Run 6's VCC3V3 sat in 7 KiCad islands while this filter said "Already fully
connected" and a plain --nets call routed nothing. With fragment_gate=True a
zone-less net whose pads grade connected but whose copper is multiple strict
fragments re-enters nets_to_route; default False preserves route_diff's
behavior; a poured (zone-owning) net is exempt.
"""
import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),      # #522 layout
          os.path.join(ROOT, 'py_tools'), os.path.join(ROOT, 'rust_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import make_pad, make_seg, make_net, make_pcb
from kicad_parser import BoardInfo
from routing_config import GridRouteConfig
from routing_common import filter_already_routed


def _board(with_phantom_split=True):
    """Two-pad net whose copper is (a) one whole run, or (b) two fragments
    the PERMISSIVE grader shades connected: a thin track ending 0.09mm from a
    tiny pad center (copper gap real, old 0.1mm flat pad radius joins it)."""
    pads = [make_pad(1, 0.0, 0.0, num='1', size_x=0.06, size_y=0.06),
            make_pad(1, 10.0, 0.0, num='2', size_x=0.06, size_y=0.06)]
    if with_phantom_split:
        segs = [make_seg(0.0, 0.0, 4.0, 0.0, width=0.06),
                make_seg(4.0, 0.0, 4.0, 2.0, width=0.06),   # stub fragment 1
                make_seg(0.09, 0.0, 9.91, 0.0, width=0.06)]
        # fragment 1 = pad1's short run+stub; fragment 2 = the long run whose
        # start (0.09, 0) is outside pad1's copper but inside the permissive
        # pad radius. Both ends of the long run attach a pad exactly.
        segs = [make_seg(0.09, 0.0, 9.91, 0.0, width=0.06),
                make_seg(0.0, 0.0, 0.0, 1.5, width=0.06)]
    else:
        segs = [make_seg(0.0, 0.0, 10.0, 0.0, width=0.06)]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'])
    pcb = make_pcb(nets={1: make_net(1, 'RAIL')}, segments=segs,
                   pads_by_net={1: pads}, board_info=bi)
    pcb.zones = []
    return pcb


def test_gate_catches_phantom_split():
    pcb = _board(True)
    cfg = GridRouteConfig()
    to_route, skipped = filter_already_routed(pcb, [('RAIL', 1)], cfg,
                                              fragment_gate=True)
    assert to_route == [('RAIL', 1)], (to_route, skipped)
    print("  PASS: fragment gate re-queues the phantom-connected split net")


def test_default_keeps_old_behavior():
    pcb = _board(True)
    cfg = GridRouteConfig()
    to_route, skipped = filter_already_routed(pcb, [('RAIL', 1)], cfg)
    assert to_route == [] and skipped, (to_route, skipped)
    print("  PASS: default (route_diff parity) still skips it")


def test_whole_net_still_skipped():
    pcb = _board(False)
    cfg = GridRouteConfig()
    to_route, skipped = filter_already_routed(pcb, [('RAIL', 1)], cfg,
                                              fragment_gate=True)
    assert to_route == [] and skipped, (to_route, skipped)
    print("  PASS: a genuinely whole net is still skipped under the gate")


if __name__ == '__main__':
    test_gate_catches_phantom_split()
    test_default_keeps_old_behavior()
    test_whole_net_still_skipped()
    print("ALL PASS")

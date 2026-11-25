#!/usr/bin/env python3
"""The smoothing shortcut keeps out of pour barrier islands (#1260).

The router stamps a barrier island (a pour of a zone net outside the run
that only this run could cut) as a hard obstacle and detours round it. The
octolinear smoothing pass checked foreign copper, holes, the edge and rule
areas, but not the barriers, so its shortcut could lay the track straight
back across the island.

Here a 90-degree elbow (10,10)-(14,10)-(14,14), which smoothing shortcuts
with the diagonal, has a barrier island in the elbow's inside corner, on the
diagonal. With the barrier the elbow stays; on another layer, or with no
barrier, the shortcut is taken as before.

    python3 tests/test_smoothing_pour_barrier.py
"""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment  # noqa: E402
from pcb_modification import smooth_octolinear_chains  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402

W = 0.2
ELBOW = [(10.0, 10.0), (14.0, 10.0), (14.0, 14.0)]
ISLAND = [(11.5, 11.5), (12.5, 11.5), (12.5, 12.5), (11.5, 12.5)]


def _pad(ref, x, y):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=0.5, size_y=0.5, shape='rect',
               layers=['F.Cu'], net_id=1, net_name='/SIG', drill=0)


def _board():
    segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                    width=W, layer='F.Cu', net_id=1) for a, b in zip(ELBOW, ELBOW[1:])]
    return PCBData(footprints={}, nets={1: Net(1, '/SIG'), 2: Net(2, 'ZONE_NET')},
                   segments=segs, vias=[],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(0.0, 0.0, 30.0, 30.0)),
                   pads_by_net={1: [_pad('A', *ELBOW[0]), _pad('B', *ELBOW[-1])]})


def _smooth(barrier_layer):
    pcb = _board()
    cfg = GridRouteConfig(track_width=W, clearance=0.2, grid_step=0.05,
                          layers=['F.Cu', 'B.Cu'])
    if barrier_layer:
        cfg.pour_barriers = [{'net': 'ZONE_NET', 'layer': barrier_layer,
                              'polygons': [ISLAND], 'clearance': 0.3}]
    out = smooth_octolinear_chains([], pcb, clearance=0.2, config=cfg)
    return pcb, out[4]


def _corner_kept(pcb):
    return any(abs(s.end_x - 14.0) < 1e-9 and abs(s.end_y - 10.0) < 1e-9
               or abs(s.start_x - 14.0) < 1e-9 and abs(s.start_y - 10.0) < 1e-9
               for s in pcb.segments)


class TestSmoothingPourBarrier(unittest.TestCase):
    def test_no_barrier_shortcut_taken(self):
        pcb, st = _smooth(None)
        self.assertGreater(st.get('spans', 0), 0, st)
        self.assertFalse(_corner_kept(pcb), [(s.start_x, s.start_y, s.end_x, s.end_y)
                                            for s in pcb.segments])

    def test_barrier_on_the_layer_refuses_the_shortcut(self):
        pcb, st = _smooth('F.Cu')
        self.assertTrue(_corner_kept(pcb), [(s.start_x, s.start_y, s.end_x, s.end_y)
                                           for s in pcb.segments])

    def test_barrier_on_another_layer_does_not(self):
        pcb, st = _smooth('B.Cu')
        self.assertFalse(_corner_kept(pcb), st)


if __name__ == '__main__':
    unittest.main()

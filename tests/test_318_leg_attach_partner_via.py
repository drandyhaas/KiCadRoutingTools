#!/usr/bin/env python3
"""A hybrid leg's grid stand-in at the coupled middle collapses off a partner VIA (#318, #215).

A hybrid leg's A* ends on the grid cell nearest the coupled middle's
off-grid near end and joins it with a short segment; when that stand-in
corner sits below clearance to the partner, `_collapse_leg_attach_join`
moves the leg's last corner onto the exact near end (#215). It measured
the partner's TRACKS only. Since #318's gate counts a partner via as copper,
watchy USB_D's N leg -- its stand-in (81.40, 88.10) 0.088 mm from P's
middle via at (81.2, 87.9), where the float near end (81.40, 88.125)
clears it by 0.101 -- was refused on every layer pair, and the pair fell
back to two single-ended routes. The collapse now sees partner vias on
every layer they span.

Checks (watchy's geometry, 0.1 mm track and clearance, 0.15 gap):
  1. precondition: given the partner's tracks alone, the fold is left as
     it is (two segments, corner 0.083 mm from the via at the 0.1 width;
     0.088 once the gate necks it);
  2. given the via, the leg ends on the exact near end and clears it by
     the full 0.1;
  3. a via that the stand-in already clears changes nothing;
  4. a buried via that does not reach the leg's layer changes nothing.

    python3 tests/test_318_leg_attach_partner_via.py
"""
import os
import sys
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Segment                                     # noqa: E402
from routing_config import GridRouteConfig                           # noqa: E402
import diff_pair_routing as dpr                                      # noqa: E402

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
N, P = 58, 59
ATTACH = (81.40, 88.125)


def _leg():
    return [Segment(start_x=81.45, start_y=88.15, end_x=81.40, end_y=88.10,
                    width=0.1, layer='In1.Cu', net_id=N),
            Segment(start_x=81.40, start_y=88.10, end_x=ATTACH[0], end_y=ATTACH[1],
                    width=0.1, layer='In1.Cu', net_id=N)]


def _via(x, y, layers=('F.Cu', 'B.Cu'), via_type='through'):
    return SimpleNamespace(x=x, y=y, size=0.3, drill=0.2, net_id=P,
                           layers=list(layers), via_type=via_type)


def _gap(segs, via):
    cu = dpr._vias_as_layer_copper([via], 'In1.Cu')
    return min(dpr._seg_to_seglist_min_edge(s.start_x, s.start_y, s.end_x, s.end_y,
                                            s.width, s.layer, cu) for s in segs)


def main():
    cfg = GridRouteConfig(layers=LAYERS, track_width=0.1, clearance=0.1,
                          diff_pair_gap=0.15, grid_step=0.05)
    p_track = [Segment(start_x=81.2, start_y=87.9, end_x=81.4, end_y=87.9, width=0.1,
                       layer='F.Cu', net_id=P)]   # P's own copper, another layer
    via = _via(81.2, 87.9)

    leg = _leg()
    dpr._collapse_leg_attach_join(leg, ATTACH, cfg, None, N, p_track)
    assert len(leg) == 2 and abs(_gap(leg, via) - 0.0828) < 1e-3, (len(leg), _gap(leg, via))
    print(f"  PASS: precondition: tracks alone leave the fold ({_gap(leg, via):.4f} mm to P's via)")

    leg = _leg()
    dpr._collapse_leg_attach_join(leg, ATTACH, cfg, None, N, p_track, partner_vias=[via])
    assert len(leg) == 1, leg
    assert (leg[0].end_x, leg[0].end_y) == ATTACH, (leg[0].end_x, leg[0].end_y)
    assert _gap(leg, via) >= 0.1 - 1e-6, _gap(leg, via)
    print(f"  PASS: with the via, the leg ends on the exact near end, {_gap(leg, via):.4f} mm clear")

    far = _via(80.9, 87.6)
    leg = _leg()
    dpr._collapse_leg_attach_join(leg, ATTACH, cfg, None, N, p_track, partner_vias=[far])
    assert len(leg) == 2, leg
    print("  PASS: a via the stand-in already clears changes nothing")

    buried = _via(81.2, 87.9, layers=('In2.Cu', 'B.Cu'), via_type='buried')
    assert not dpr._vias_as_layer_copper([buried], 'In1.Cu'), 'fixture: the buried via must miss In1.Cu'
    leg = _leg()
    dpr._collapse_leg_attach_join(leg, ATTACH, cfg, None, N, p_track, partner_vias=[buried])
    assert len(leg) == 2, leg
    print("  PASS: a via that does not reach the leg's layer changes nothing")
    print('test_318_leg_attach_partner_via: all passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())

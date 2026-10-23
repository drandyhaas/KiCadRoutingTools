#!/usr/bin/env python3
"""#1303: a hybrid diff-pair route whose legs join a net's terminals WITHOUT its
coupled middle is refused (`diff_pair_routing._middle_bypassed`).

On imx6ulz DQS0 the N leg from U2 changed layers for free on the via at the
middle's FAR end (the U1 leg's), rose to F.Cu there and ran back along the
middle to its near end. Both pads met at that via; the middle hung off the
signal path as a loop, the post-route cleanup broke the loop and removed the
dangling middle, and DQS0 shipped 0 mm coupled with 12 mm of skew. The
cleanup was right -- copper the signal does not pass through couples nothing
-- so the leg plan is refused instead and the next plan, polarity or layer
pair is tried.

Driven on synthetic copper: pads A (0,0) and B (10,0) on F.Cu, a coupled
middle (3,0)-(7,0) on F.Cu.
  - legs A-(3,0) and (7,0)-B meet the middle at its own ends: not bypassed;
  - the A leg reaches the far end (7,0) and runs back to (3,0): bypassed;
  - a leg that hops the middle's own layer-change via to the far side: bypassed;
  - board copper that already joins A and B is not the legs' doing: not bypassed.

    python3 tests/test_1303_middle_bypass.py
"""
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from diff_pair_routing import _middle_bypassed                  # noqa: E402
from kicad_parser import BoardInfo, Net, Pad, PCBData, Segment, Via  # noqa: E402

NET = 1
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def seg(pts, layer='F.Cu'):
    return [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.1,
                    layer=layer, net_id=NET) for a, b in zip(pts, pts[1:])]


def via(x, y):
    return Via(x=x, y=y, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=NET)


def pad(num, x):
    return Pad(pad_number=num, net_id=NET, net_name='D_N', shape='rect',
               global_x=x, global_y=0.0, local_x=0.0, local_y=0.0,
               size_x=0.3, size_y=0.3, layers=['F.Cu'], drill=0.0,
               pad_type='smd', component_ref=f'U{num}')


def board(segments=()):
    info = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'])
    return PCBData(board_info=info, nets={NET: Net(net_id=NET, name='D_N')},
                   footprints={}, segments=list(segments), vias=[],
                   pads_by_net={NET: [pad('1', 0.0), pad('2', 10.0)]})


def main():
    # legs meet the middle at its own ends
    legs = seg([(0, 0), (3, 0)]) + seg([(7, 0), (10, 0)])
    check('legs that meet the middle at its own ends: not bypassed',
          not _middle_bypassed(board(), NET, [], legs, []))

    # the A leg goes to the FAR end on B.Cu, rises on a via there, and runs back
    # along the middle on F.Cu to the near end (the DQS0 shape)
    far = (seg([(0, 0), (0, 1)]) + seg([(0, 1), (7, 1), (7, 0)], 'B.Cu')
           + seg([(7, 0), (3, 0)]) + seg([(7, 0), (10, 0)]))
    check('a leg through the far end and back along the middle: bypassed',
          _middle_bypassed(board(), NET, [], far, [via(0, 1), via(7, 0)]))

    # the middle changes layer at (5,0); a leg reaching that via from both sides
    # joins the pads without the middle's tracks
    hop = seg([(0, 0), (0, 2), (5, 2), (5, 0)], 'F.Cu') + seg([(5, 0), (10, 0)], 'B.Cu')
    check("a leg hopping the middle's own layer-change via: bypassed",
          _middle_bypassed(board(), NET, [via(5, 0)], hop, [via(10, 0)]))

    # board copper already joins the pads: the legs did not make that
    check('board copper that joins the pads on its own: not bypassed',
          not _middle_bypassed(board(seg([(0, 0), (0, -1), (10, -1), (10, 0)])),
                               NET, [], legs, []))

    # a multipoint leg is judged on ITS two terminals only
    third = Pad(pad_number='3', net_id=NET, net_name='D_N', shape='rect',
                global_x=20.0, global_y=0.0, local_x=0.0, local_y=0.0,
                size_x=0.3, size_y=0.3, layers=['F.Cu'], drill=0.0,
                pad_type='smd', component_ref='U3')
    b = board()
    b.pads_by_net[NET].append(third)
    check('a multipoint leg is judged on its own terminals',
          _middle_bypassed(b, NET, [], far, [via(0, 1), via(7, 0)],
                           terminal_pads={NET: b.pads_by_net[NET][:2]}))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

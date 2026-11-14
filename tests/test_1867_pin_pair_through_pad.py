#!/usr/bin/env python3
"""#1867: `pin_pair_path_length` between two pads whose copper path passes
through a third pad of the net.

A lane that enters an in-line part's pad on one track and leaves it on
another (an ESD part, a test point) is joined: check_connected and
`net_pads_joined` say so. `pin_pair_path_length` built its graph from the two
pads asked about alone, so the walk stopped at the in-line pad and the pair
read None, the value reserved for an unrouted or broken net; board_score's
`pin_pair` length group then skipped the lane.

The in-line pad is now read as copper, but only when the tracks alone do not
join the pair, and charged from where a track meets it to its centre and out
again. So a pair the tracks already join measures exactly what it measured.

Boards (in memory, F.Cu, 0.2 mm tracks): A at (0, 0), B at (10, 0), and M, a
1.0 x 0.6 mm pad at (5, 0), all on one net.

Rows:
  1. two tracks ending inside M, 0.6 mm apart: A->M and M->B are 4.7 mm each,
     as before, and A->B is 4.7 + 0.3 + 0.3 + 4.7 = 10.0 mm, both ways round,
     where it read None; `net_pads_joined` is True;
  2. one track with a V inside M, joined end to end: A->B is the
     track's own length, not the shorter way across the pad's centre;
  3. the two tracks stop short of M: A->B is still None, and
     `net_pads_joined` is False;
  4. the in-line pad on another layer than the tracks joins nothing.
"""
import math
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from synth import make_pad, make_seg, make_net, make_pcb  # noqa: E402
from kicad_parser import Footprint, BoardInfo             # noqa: E402
from net_queries import pin_pair_path_length, net_pads_joined  # noqa: E402

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def board(segs, m_layer='F.Cu'):
    a = make_pad(1, 0.0, 0.0, ref='J1', num='1', net_name='N')
    m = make_pad(1, 5.0, 0.0, ref='U2', num='5', net_name='N', size_x=1.0, size_y=0.6,
                 layers=(m_layer,))
    b = make_pad(1, 10.0, 0.0, ref='U1', num='9', net_name='N')
    pads = [a, m, b]
    fps = {p.component_ref: Footprint(reference=p.component_ref, footprint_name='t:P',
                                      x=p.global_x, y=p.global_y, rotation=0.0,
                                      layer=p.layers[0], pads=[p]) for p in pads}
    pcb = make_pcb(nets={1: make_net(1, 'N', pads=pads)}, footprints=fps,
                   pads_by_net={1: pads}, segments=[make_seg(*s) for s in segs],
                   board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                        copper_layers=['F.Cu', 'B.Cu'],
                                        board_bounds=(-2.0, -2.0, 12.0, 2.0)),
                   zones=[])
    return pcb, a, m, b


def near(v, want):
    return v is not None and abs(v - want) < 1e-6


def main():
    print("1. two tracks that end inside the in-line pad")
    pcb, a, m, b = board([(0, 0, 4.7, 0), (5.3, 0, 10, 0)])
    ok(net_pads_joined(pcb, 1), "net_pads_joined is True")
    ok(near(pin_pair_path_length(pcb, 1, a, m), 4.7)
       and near(pin_pair_path_length(pcb, 1, m, b), 4.7),
       "each leg to the in-line pad is 4.7 mm, as before")
    ab, ba = pin_pair_path_length(pcb, 1, a, b), pin_pair_path_length(pcb, 1, b, a)
    ok(near(ab, 10.0) and near(ba, 10.0),
       f"end to end is 10.0 mm both ways round, through the pad: {ab} / {ba}")

    print("2. one track with a V inside the pad")
    legs = [(0, 0, 4.6, 0.2), (4.6, 0.2, 5.0, -0.25), (5.0, -0.25, 5.4, 0.2),
            (5.4, 0.2, 10, 0)]
    pcb, a, m, b = board(legs)
    own = sum(math.hypot(x2 - x1, y2 - y1) for x1, y1, x2, y2 in legs)
    across = 2 * (math.hypot(4.6, 0.2) + math.hypot(0.4, 0.2))
    ab = pin_pair_path_length(pcb, 1, a, b)
    ok(near(ab, own) and own > across,
       f"end to end is the track's own {own:.4f} mm, not {across:.4f} mm across the pad: {ab}")

    print("3. two tracks that stop short of the pad")
    pcb, a, m, b = board([(0, 0, 4.2, 0), (5.8, 0, 10, 0)])
    ok(pin_pair_path_length(pcb, 1, a, b) is None and not net_pads_joined(pcb, 1),
       "end to end is None, and the net is not joined")

    print("4. the in-line pad on another layer")
    pcb, a, m, b = board([(0, 0, 4.7, 0), (5.3, 0, 10, 0)], m_layer='B.Cu')
    ok(pin_pair_path_length(pcb, 1, a, b) is None,
       "a pad the tracks do not touch joins nothing")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())

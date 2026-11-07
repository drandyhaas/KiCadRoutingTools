#!/usr/bin/env python3
"""The direction a stub escapes in at its free end (braid._end_dir), which the fanout's sidecar records as each
tooth's and the whole route reads a tooth's face and a pair's end connectors from.

  python3 tests/test_622_end_dir.py

It is the longest segment of the free end's RUN -- the stub from its free end back while each turn stays under 60
degrees -- pointed toward the free end:

1. an escape down a channel and out of the array's face at a corner: the way out of the face, though the channel
   is the longer;
2. a run ended by a 45-degree jog: the run's way, the jog taken in;
3. a half-pitch jog along the row and a half-pitch run out, one as long as the other: the run out;
4. a stub of one segment: its way, pointed toward the free end;
5. no copper at the free end: +x.
"""
import os
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import braid as bd  # noqa: E402

fails = []


def check(cond, msg):
    if not cond:
        fails.append(msg)
        print('FAIL', msg)


def board(pts, nid=7):
    """a pad at pts[0] and a stub through the points, as a board _end_dir reads"""
    segs = [types.SimpleNamespace(net_id=nid, start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], layer='In1.Cu')
            for a, b in zip(pts, pts[1:])]
    pad = types.SimpleNamespace(global_x=pts[0][0], global_y=pts[0][1], size_x=0.35, size_y=0.35)
    return types.SimpleNamespace(segments=segs), [pad]


def way(pts):
    pcb, pads = board(pts)
    d = bd._end_dir(pcb, 7, pts[-1], pads)
    return (round(d[0], 3), round(d[1], 3))


check(way([(0.0, 0.0), (0.0, 0.4), (0.4, 0.4), (0.4, 2.0), (1.625, 2.0)]) == (1.0, 0.0),
      '1: down a channel 1.6 mm, out of the face 1.225 mm: the way out')
check(way([(0.0, 0.0), (2.0, 0.0), (2.3, 0.3)]) == (1.0, 0.0), '2: a run and a 45-degree jog: the run')
check(way([(0.0, 0.0), (-0.4257, 0.0), (-0.4257, 0.425)]) == (0.0, 1.0),
      '3: a half-pitch jog along the row, a half-pitch run out: the run out')
check(way([(0.0, 0.0), (0.0, -1.0)]) == (0.0, -1.0), '4: one segment')
pcb, pads = board([(0.0, 0.0), (1.0, 0.0)])
check(bd._end_dir(pcb, 8, (1.0, 0.0), pads) == (1.0, 0.0), '5: no copper at the free end')
print('end_dir: ' + ('FAIL' if fails else 'PASS: the longest segment of the free end\'s run -- a corner ends it, a '
                     '45-degree jog does not'))
sys.exit(1 if fails else 0)

#!/usr/bin/env python3
"""merge_close_same_net_vias never ships a zero-length leg.

When a new plane via lands within hole-to-hole of a same-net via, the merge
drops it and moves every segment end that sat on it onto the survivor. A leg
that ran from the dropped via TO the survivor then has both ends on the
survivor: zero-length copper that connects nothing (the survivor's barrel
already spans the leg's layer) and that KiCad grades as "Tracks crossing"
against copper millimetres away.

Found on orangecrab (set3, route.py's plane finalize -> repair_planes ->
merge_close_same_net_vias): 21 zero-length segments shipped on the final
board. The merge now drops exactly the re-anchored segments that
degenerated, from the write list and from the board mirror alike.

Cases:
  * a new via merges onto a pre-existing one: its leg to the survivor is
    dropped from both lists, its other leg survives re-anchored;
  * two NEW vias with a leg between them: the second merges onto the first
    and the leg between them is dropped;
  * a zero-length segment the merge did not touch is left alone (the drop is
    scoped to what the merge itself made).

    python3 tests/test_merge_vias_zero_length.py
"""
import contextlib
import io
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import Segment, Via  # noqa: E402
from pcb_modification import merge_close_same_net_vias  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


class _PCB:
    def __init__(self, vias, segments):
        self.vias = vias
        self.segments = segments
        self.nets = {}
        self.pads_by_net = {}


def _nv(x, y):
    return {'x': x, 'y': y, 'drill': 0.3, 'size': 0.6,
            'layers': ['F.Cu', 'B.Cu'], 'net_id': 1, 'via_type': 'through'}


def _ws(a, b, layer):
    return {'start': a, 'end': b, 'width': 0.2, 'layer': layer, 'net_id': 1}


def _bs(a, b, layer):
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                   width=0.2, layer=layer, net_id=1)


def _len(s):
    if isinstance(s, dict):
        (x1, y1), (x2, y2) = s['start'], s['end']
    else:
        x1, y1, x2, y2 = s.start_x, s.start_y, s.end_x, s.end_y
    return math.hypot(x2 - x1, y2 - y1)


def _zero(segs):
    return [s for s in segs if _len(s) < 1e-6]


def _merge(vias, segs, pcb):
    with contextlib.redirect_stdout(io.StringIO()) as out:
        n = merge_close_same_net_vias(vias, segs, pcb, hole_to_hole_clearance=0.25)
    return n, out.getvalue()


# 1. a new via onto a PRE-EXISTING one. S at (10,10); the new via V 0.2 away
#    (h2h needs 0.3 + 0.25 = 0.55); legs V->S on F.Cu and V->P on B.Cu. The
#    plane engines stamp V and its legs into pcb_data too (#508 finding 12).
S, V, P = (10.0, 10.0), (10.2, 10.0), (12.0, 10.0)
vias = [_nv(*V)]
segs = [_ws(V, S, 'F.Cu'), _ws(V, P, 'B.Cu')]
board = _PCB([Via(x=S[0], y=S[1], size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1),
              Via(x=V[0], y=V[1], size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)],
             [_bs(V, S, 'F.Cu'), _bs(V, P, 'B.Cu')])
n, log = _merge(vias, segs, board)
check("the new via merges onto the existing one", n == 1 and not vias)
check("no zero-length leg in the write list", not _zero(segs),
      f"{len(_zero(segs))} left")
check("no zero-length leg on the board", not _zero(board.segments),
      f"{len(_zero(board.segments))} left")
check("the other leg survives, re-anchored on the survivor",
      len(segs) == 1 and segs[0]['start'] == S and segs[0]['end'] == P
      and len(board.segments) == 1
      and (board.segments[0].start_x, board.segments[0].start_y) == S)
check("the run says what it dropped", "zero-length" in log, log.strip())

# 2. two NEW vias with a leg between them: the second merges onto the first.
A, B = (20.0, 20.0), (20.0, 20.3)
vias = [_nv(*A), _nv(*B)]
segs = [_ws(A, B, 'In1.Cu'), _ws(B, (20.0, 22.0), 'F.Cu')]
board = _PCB([Via(x=A[0], y=A[1], size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1),
              Via(x=B[0], y=B[1], size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)],
             [_bs(A, B, 'In1.Cu'), _bs(B, (20.0, 22.0), 'F.Cu')])
n, _log = _merge(vias, segs, board)
check("two new vias: one merged", n == 1 and len(vias) == 1)
check("two new vias: the leg between them is gone from both lists",
      not _zero(segs) and not _zero(board.segments)
      and len(segs) == 1 and len(board.segments) == 1)

# 3. scope: a zero-length segment the merge never touched stays.
Q = (30.0, 30.0)
vias = [_nv(*V)]
segs = [_ws(V, S, 'F.Cu'), _ws(Q, Q, 'F.Cu')]
board = _PCB([Via(x=S[0], y=S[1], size=0.6, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)],
             [])
n, _log = _merge(vias, segs, board)
check("an untouched zero-length segment is left alone",
      n == 1 and len(segs) == 1 and segs[0]['start'] == Q)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")

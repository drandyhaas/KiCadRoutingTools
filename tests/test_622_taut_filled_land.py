#!/usr/bin/env python3
"""A taut string goes round a FILLED copper graphic, as it goes round a pad -- it does not cross it as it crosses a track.

  python3 tests/test_622_taut_filled_land.py

A footprint's land drawn as a filled polygon (#908, #1181: graphic segments, `graphic_filled`, the ring shared by its
segments) is copper area. braid._build_obstacles names its outline apart (braid.GFX, one name per shape) and the taut
strings see it as they see a pad drawn as capsules: the one disc round it (taut_fast.pad_discs). Named as tracks, its
edges were crossed as a track is (a dive) and pushed a string only where it ran along them: on the zynq board X2.1's
chamfered land took four strings in and folded each, 30% longer every block, to 846 mm for a 32 mm lane. This pins,
on a board of two drawn shapes and nothing else:

1. a string straight across a filled land: no point of it inside the land, and its length a little over the chord;
2. the same land NOT filled (a drawn outline, copper only along its lines): crossed, as a track is -- the fix is the
   filled shape's, not every graphic's;
3. braid.build_obstacles with the land's net among the excluded (a base DERIVED from the board's full model) against
   the model built without them: the same signature and the same answers inside the land -- the derivation must leave
   a filled shape's outline out as the build does.
"""
import contextlib
import io
import math
import os
import sys
import tempfile
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
with contextlib.redirect_stdout(io.StringIO()):
    from kicad_parser import Segment  # noqa: E402
    import braid  # noqa: E402
    import taut_fast as tf  # noqa: E402

BAD = []
LAND, OWN, OTHER = 11, 7, 12               # the land's net, the string's, a far track's
# X2.1's land as the zynq board draws it: a 1.15 x 1.4 mm rectangle, one corner chamfered
RING = ((4.425, -0.7), (5.575, -0.7), (5.575, 0.7), (4.804, 0.7), (4.425, 0.338))


def check(ok, what):
    print(('ok    ' if ok else 'FAIL  ') + what)
    if not ok:
        BAD.append(what)


def board(filled, path=None):
    segs = [Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=0.3, layer='F.Cu', net_id=LAND,
                    graphic=True, owner_ref='X2', graphic_kind='poly', graphic_filled=filled,
                    graphic_ring=RING if filled else None)
            for a, b in zip(RING, RING[1:] + RING[:1])]
    # (the string's own net and another's: a track each, far off, so a model can leave the string's net out)
    segs += [Segment(start_x=0.0, start_y=-6.0, end_x=1.0, end_y=-6.0, width=0.15, layer='F.Cu', net_id=OWN),
             Segment(start_x=9.0, start_y=-6.0, end_x=10.0, end_y=-6.0, width=0.15, layer='F.Cu', net_id=OTHER)]
    return SimpleNamespace(footprints={}, segments=segs, vias=[], source_path=path)


def inside(x, y):
    hit = False
    for (x0, y0), (x1, y1) in zip(RING, RING[1:] + RING[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            hit = not hit
    return hit


def string(pcb):
    obs = braid._build_obstacles(pcb, frozenset(), 'F.Cu').exclude({OWN})
    with contextlib.redirect_stderr(io.StringIO()):
        pts, _it = tf.relax_many([((0.0, 0.1), (10.0, -0.1), obs)])[0]
    return pts, sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def main():
    chord = math.dist((0.0, 0.1), (10.0, -0.1))
    pts, L = string(board(True))
    n_in = sum(1 for x, y in pts if inside(x, y))
    check(n_in == 0, f'1. a string across a FILLED land: {n_in} of its {len(pts)} points inside the land (want 0)')
    check(L < 1.15 * chord, f'1. ...and it goes round it: {L:.2f} mm for a chord of {chord:.2f}')
    pts, L = string(board(False))
    n_in = sum(1 for x, y in pts if inside(x, y))
    check(n_in > 0, f'2. a string across the same outline NOT filled: crossed as a track is ({n_in} points inside)')
    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, 'board.kicad_pcb')
        open(src, 'w').write('(kicad_pcb)\n')
        pcb = board(True, src)
        kids = frozenset({OWN, LAND})
        derived = braid.build_obstacles(pcb, OWN, kids, 'F.Cu')
        built = braid._build_obstacles(pcb, kids, 'F.Cu').exclude({OWN})
        check(derived.signature() == built.signature(),
              '3. a base derived without the land\'s net against one built without it: the same signature')
        probe = [(5.0, 0.0), (4.6, 0.5), (5.4, -0.5), (4.2, 0.0)]
        check([derived.point_violation(p) for p in probe] == [built.point_violation(p) for p in probe],
              '3. ...and the same answers in and round the land')
    print(f'{"FAIL" if BAD else "PASS"}: {len(BAD)} failure(s)')
    return 1 if BAD else 0


if __name__ == '__main__':
    sys.exit(main())

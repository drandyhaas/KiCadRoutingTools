#!/usr/bin/env python3
"""The under-pad engine checks an escape stub against other nets' TRACKS on the stub's own layer only.

  python3 tests/test_underpad_stub_own_layer.py

A stub is single-layer copper: a track on another layer cannot touch it, and the via at the stub's end, which spans
the layers, is checked against every via apart. Read against tracks on every layer, the stub check refused real
escapes -- zynq U2's plane-drop stubs under a B.Cu DDR3 lane, each clear of every F.Cu track by 0.2 mm or more, and
the bus's teeth on H3's U1 under the other layer's stubs. This pins bga_fanout.underpad.stub_track_conflict, which
the engine's stub check calls:

1. a foreign track on the OTHER layer, straight along the stub: no conflict;
2. the same track on the stub's layer: a conflict;
3. the stub's own net's track on its layer: no conflict;
4. the clearance is the edge: a foreign track on the layer just outside track + clearance is clear, just inside is
   not (so check 2 is not a check that refuses everything);
5. the engine's stub check calls it (read from the source: a check of the function alone would pass with the engine
   back on its old every-layer loop).
"""
import inspect
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from bga_fanout import underpad  # noqa: E402

F, B = 0, 31                 # layer indices: the stub's (F.Cu) and another
HW, CLR, TW = 0.05, 0.1, 0.1  # the stub's half width; clearance; the foreign track's width


def conflict(seg_y, layer, net=7, own=1):
    """the stub (0,0)-(1,0) of net `own` against a track of `net` along y = seg_y on `layer`"""
    return underpad.stub_track_conflict(own, (0.0, 0.0), (1.0, 0.0),
                                        [(0.0, seg_y, 1.0, seg_y, TW / 2, net, layer)], F, HW, CLR)


def main():
    print('=' * 60)
    print('under-pad stubs: other nets\' tracks on the stub\'s own layer only')
    print('=' * 60)
    fails = []
    if conflict(0.0, B):
        fails.append('a foreign track on the OTHER layer, straight along the stub, refused it')
    if not conflict(0.0, F):
        fails.append('a foreign track on the stub\'s own layer, straight along it, did not refuse it')
    if conflict(0.0, F, net=1):
        fails.append('the stub\'s own net\'s track refused it')
    edge = HW + TW / 2 + CLR
    if conflict(edge + 0.001, F):
        fails.append(f'a foreign track {edge + 0.001:.3f} mm off, outside track + clearance ({edge:.3f}), refused it')
    if not conflict(edge - 0.001, F):
        fails.append(f'a foreign track {edge - 0.001:.3f} mm off, inside track + clearance ({edge:.3f}), did not')
    src = inspect.getsource(underpad.generate_underpad_escape)
    if 'stub_track_conflict(' not in src:
        fails.append('the engine\'s stub check (generate_underpad_escape) does not call stub_track_conflict')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: a track on the other layer is no conflict, one on the stub\'s layer is, to the clearance\'s edge, '
          'and the engine checks its stubs so')
    return 0


if __name__ == '__main__':
    sys.exit(main())

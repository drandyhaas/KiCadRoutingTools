"""The per-net clearance excess tables are cached, and the cache must follow the maps' CONTENT: a caller that edits
net_clearances / track_clearances in place (same dict object, same length, new value) must see the new clearance.

One foreign 0.2 mm track on net 2 runs 1 mm from the queried net-1 segment: its edge is 0.9 mm away. With net 2's
class clearance raised from 0.1 to 0.4 mm over a 0.1 mm base, its 0.3 mm excess comes off: 0.6 mm."""
import os
import sys
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'py_router'))

import single_ended_routing as ser  # noqa: E402


def _pcb():
    seg = NS(start_x=0.0, start_y=1.0, end_x=1.0, end_y=1.0, layer='F.Cu', width=0.2, net_id=2)
    return NS(segments=[seg], vias=[], footprints=None)


def _dist(pcb, nc=None, tc=None, numpy_only=False):
    f = ser._seg_foreign_seg_dist_numpy if numpy_only else None
    if f:
        return f(pcb, 1, 0.0, 0.0, 1.0, 0.0, 'F.Cu', nc, 0.1, tc, None)
    return ser._seg_foreign_seg_dist(pcb, 1, 0.0, 0.0, 1.0, 0.0, 'F.Cu', net_clearances=nc,
                                     base_clearance=0.1, track_clearances=tc)


def test_net_clearances_edited_in_place():
    for numpy_only in (False,):
        pcb = _pcb()
        nc = {2: 0.1}
        a = _dist(pcb, nc=nc, numpy_only=numpy_only)
        nc[2] = 0.4                       # same object, same length
        b = _dist(pcb, nc=nc, numpy_only=numpy_only)
        assert abs(a - 0.9) < 1e-9 and abs(b - 0.6) < 1e-9, (numpy_only, a, b)


def test_track_clearances_edited_in_place():
    for numpy_only in (False,):
        pcb = _pcb()
        tc = {2: 0.1}
        a = _dist(pcb, tc=tc, numpy_only=numpy_only)
        tc[2] = 0.4
        b = _dist(pcb, tc=tc, numpy_only=numpy_only)
        assert abs(a - 0.9) < 1e-9 and abs(b - 0.6) < 1e-9, (numpy_only, a, b)


def test_tables_follow_content():
    nc = {1: 0.1, 2: 0.1}
    fnid = np.array([1, 2])
    assert list(ser._net_excess(fnid, nc, None, 0.1)) == [0.0, 0.0]
    nc[2] = 0.3
    assert np.allclose(ser._net_excess(fnid, nc, None, 0.1), [0.0, 0.2])


if __name__ == '__main__':
    test_net_clearances_edited_in_place()
    test_track_clearances_edited_in_place()
    test_tables_follow_content()
    print('PASS: the net-excess caches follow in-place edits of the clearance maps')

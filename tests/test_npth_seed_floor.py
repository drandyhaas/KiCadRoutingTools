#!/usr/bin/env python3
"""Issue #390: plane-repair route seeds must honor the NPTH-to-track floor.

Region-join pseudo-anchors and KiCad-oracle island seeds are stamped
source/target cells, which OVERRIDE the obstacle map's NPTH drill keep-out.
The fill-validity margin ladder relaxed below the fab floor, approving a seed
on real zone fill 0.15mm from an NPTH hole edge -- the 0.2-wide GNDR strap
terminating there shipped a copper-to-hole violation (crkbd vs rEXSW1).

Now `npth_floor_ok` enforces drill/2 + NPTH_TO_TRACK_CLEARANCE + track_half
non-relaxably inside `_real_fill_point` and `_island_seed_points`.

    python3 tests/test_npth_seed_floor.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from plane_region_connector import npth_floor_ok, _real_fill_point
from kicad_oracle import _island_seed_points
from routing_defaults import NPTH_TO_TRACK_CLEARANCE


def _npth(x, y, drill):
    return SimpleNamespace(pad_type='np_thru_hole', drill=drill,
                           global_x=x, global_y=y, hole_x=None, hole_y=None,
                           net_id=0, layers=['*.Cu', '*.Mask'],
                           size_x=drill, size_y=drill, shape='circle',
                           rect_rotation=0.0)


def _board(npths):
    return SimpleNamespace(pads_by_net={0: list(npths)}, vias=[], segments=[],
                           zones=[])


def run():
    fails = []

    def check(name, cond):
        print(('PASS' if cond else 'FAIL') + f': {name}')
        if not cond:
            fails.append(name)

    # NPTH drill 1.9 at origin; floor for a 0.2-wide track = 0.95+0.2+0.1=1.25
    pcb = _board([_npth(0.0, 0.0, 1.9)])
    check('inside the floor rejected', not npth_floor_ok(1.20, 0.0, pcb, 0.1))
    check('outside the floor accepted', npth_floor_ok(1.30, 0.0, pcb, 0.1))
    check('width widens the floor',
          npth_floor_ok(1.30, 0.0, pcb, 0.1) and not npth_floor_ok(1.30, 0.0, pcb, 0.25))

    # _real_fill_point: NPTH floor is NOT relaxable by a small margin. Zone
    # square (-5..5); point on fill 1.2 from the hole center passes the
    # relaxed margin geometry but must still be rejected.
    poly = [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    ok = _real_fill_point((1.20, 0.0), 4, pcb, [poly], 'B.Cu', 0.15,
                          npth_track_half=0.1)
    check('_real_fill_point enforces the floor at relaxed margin', not ok)
    ok = _real_fill_point((2.0, 0.0), 4, pcb, [poly], 'B.Cu', 0.15,
                          npth_track_half=0.1)
    check('_real_fill_point keeps clear fill valid', ok)

    # _island_seed_points: bare fill cells inside the floor are dropped, the
    # rest survive; the raw reported point is dropped too when violating.
    pcb2 = _board([_npth(0.0, 0.0, 1.9)])
    step = 0.25
    cells = {(4, 0), (8, 0)}  # (1.0, 0) inside floor; (2.0, 0) clear
    pts = _island_seed_points(cells, step, pcb2, 4, 'B.Cu',
                              ['F.Cu', 'B.Cu'], (1.0, 0.0), track_half=0.1)
    xs = {round(p[0], 2) for p in pts}
    check('violating fill cell dropped', 1.0 not in xs)
    check('clear fill cell kept', 2.0 in xs)
    pts = _island_seed_points(cells, step, pcb2, 4, 'B.Cu',
                              ['F.Cu', 'B.Cu'], (2.0, 0.0), track_half=0.1)
    check('clear reported point kept',
          any(round(p[0], 2) == 2.0 for p in pts))

    print()
    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(run())

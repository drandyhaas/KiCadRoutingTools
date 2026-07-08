#!/usr/bin/env python3
"""Regression test for issue #346 (core1106_cam /5V_IN right-edge pads).

check_net_connectivity used to union same-net pads when their centres sat
within the sum of their bounding-CIRCLE radii (max half-extent each). For a
castellated module's edge pads -- 1.5x0.7mm rects at 1.0mm pitch, real
edge-to-edge gap 0.3mm -- that test called adjacent pads "connected", so:

  * the grader passed nets whose pads had NO copper on them at all, and
  * the multipoint router, whose terminal grouping trusts this checker's
    graph (#317), never routed the trivially-connectable pads.

The union must use the exact pad shapes (rect/roundrect/circle/oval +
rect_rotation + custom polygons). Genuinely-overlapping pads must stay
connected: castellated THT+SMD stacks (#92) and EP/thermal-pad overlaps
(#108).

    python3 tests/test_pad_overlap_connectivity.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad
from check_connected import (check_net_connectivity, _pads_copper_touch,
                             _net_pads_connected_by_overlap)


def _pad(ref, num, x, y, sx=1.5, sy=0.7, shape='rect', layers=('F.Cu',),
         drill=0.0, net_id=70):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=sx, size_y=sy, shape=shape,
               layers=list(layers), net_id=net_id, net_name='/5V_IN',
               rotation=0.0, drill=drill)


def main():
    results = []

    # 1. The #346 shape: 1.5x0.7 rect pads at 1.0mm pitch (0.3mm real gap).
    #    Bounding circles (r=0.75) overlap; the copper does not.
    a = _pad('U1', '79', 131.0, 67.5)
    b = _pad('U1', '80', 131.0, 66.5)
    results.append(("adjacent 1.0mm-pitch rect pads do NOT touch",
                    not _pads_copper_touch(a, b)))

    r = check_net_connectivity(70, [], [], [a, b], [])
    results.append(("net of two adjacent copper-less pads graded DISCONNECTED",
                    not r['connected']))

    results.append(("_net_pads_connected_by_overlap agrees (not connected)",
                    not _net_pads_connected_by_overlap([a, b], ['F.Cu', 'B.Cu'])))

    # 2. Pads whose rects genuinely overlap stay connected (#108 EP class).
    ep = _pad('U2', 'EP', 100.0, 100.0, sx=3.0, sy=3.0)
    corner = _pad('U2', 'T1', 101.6, 101.6, sx=0.8, sy=0.8)  # overlaps EP corner
    results.append(("overlapping EP + corner pad touch",
                    _pads_copper_touch(ep, corner)))
    r = check_net_connectivity(70, [], [], [ep, corner], [])
    results.append(("overlapping pads graded connected", r['connected']))

    # 3. Castellated THT+SMD stack at the same spot stays connected (#92).
    tht = _pad('U3', '1', 50.0, 50.0, sx=0.9, sy=0.9, shape='circle',
               layers=('*.Cu',), drill=0.5)
    smd = _pad('U3', '1s', 50.0, 50.2, sx=0.8, sy=1.2)
    results.append(("castellated THT+SMD stack still connected",
                    _net_pads_connected_by_overlap([tht, smd], ['F.Cu', 'B.Cu'])))

    # 4. Bounding circles that overlap only corner-to-corner must not connect:
    #    two 2.0x0.4 rects side by side, offset perpendicular to their length.
    thin1 = _pad('U4', '1', 10.0, 10.0, sx=2.0, sy=0.4)
    thin2 = _pad('U4', '2', 10.0, 11.0, sx=2.0, sy=0.4)  # 0.6mm real gap
    results.append(("thin parallel rect pads 0.6mm apart do NOT touch",
                    not _pads_copper_touch(thin1, thin2)))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} pad-overlap connectivity tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

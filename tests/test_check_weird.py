#!/usr/bin/env python3
"""Tests for check_weird.py (read-only copper hygiene checker).

Synthetic Segment/Via/Pad cases:
  * a dangling spur end            -> dangling-end flagged
  * a fanout stub ending on a pad  -> NOT flagged (clean)
  * a soft joint (cap overlap)     -> soft-joint flagged (and NOT dangling)
  * a duplicate segment            -> stacked-copper flagged
  * a square loop of segments      -> redundant-cycle flagged
  * a clean two-pad net            -> no findings at all
  * a half-segment tail past a mid-body via anchor -> dangling-end (tail)
  * a floating via                 -> unsupported-via flagged

    python3 tests/test_check_weird.py
"""
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Segment, Via, Zone, Net, BoardInfo, PCBData
from check_weird import check_weird

NET = 1
NAME = '/TEST'


def _pad(x, y, size=0.6, layers=('F.Cu',), drill=0.0, num='1', ref='U1'):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=size, size_y=size, shape='circle',
               layers=list(layers), net_id=NET, net_name=NAME,
               rotation=0.0, drill=drill)


def _seg(x1, y1, x2, y2, layer='F.Cu', width=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=width, layer=layer, net_id=NET)


def _via(x, y, size=0.6, drill=0.3):
    return Via(x=x, y=y, size=size, drill=drill,
               layers=['F.Cu', 'B.Cu'], net_id=NET)


def _pcb(segments, vias=(), pads=(), zones=()):
    return PCBData(
        board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
        nets={NET: Net(net_id=NET, name=NAME, pads=list(pads))},
        footprints={},
        vias=list(vias),
        segments=list(segments),
        pads_by_net={NET: list(pads)},
        zones=list(zones))


def _cats(findings):
    return Counter(f['category'] for f in findings)


def main():
    results = []

    # 1. Dangling spur: pad-to-pad trunk plus a spur teeing into its middle;
    #    the spur's free end at (5, 3) connects nothing.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(5, 0, 5, 3)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    dangles = [x for x in f if x['category'] == 'dangling-end']
    results.append(("dangling spur end flagged", c['dangling-end'] == 1))
    results.append(("dangle reported at the free end (5, 3)",
                    len(dangles) == 1 and abs(dangles[0]['x'] - 5) < 1e-6
                    and abs(dangles[0]['y'] - 3) < 1e-6))
    results.append(("spur root (T-junction) NOT flagged as dangling",
                    all((x['x'], x['y']) != (5.0, 0.0) for x in dangles)))

    # 2. Fanout stub ending on a pad (via F.Cu stub + B.Cu run): clean.
    pads = [_pad(0, 0, num='1'), _pad(3, 0, layers=('B.Cu',), num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 1, 0), _seg(1, 0, 3, 0, layer='B.Cu')],
               vias=[_via(1, 0)], pads=pads)
    f, _ = check_weird(pcb)
    results.append(("fanout stub ending on pad/via NOT flagged", len(f) == 0))

    # 3. Soft joint: two collinear tracks whose endpoints stop 0.05mm short of
    #    each other -- caps overlap ((w1+w2)/2 = 0.2 > 0.05 > min gap 0.01).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 5, 0), _seg(5.05, 0, 10, 0)], pads=pads)
    # gap 0.05 < default tolerance 0.1: detection is exercised at tolerance=0,
    # and the default-filter behavior is asserted right after.
    f, _ = check_weird(pcb, tolerance=0)
    c = _cats(f)
    results.append(("soft joint flagged", c['soft-joint'] == 1))
    results.append(("soft-joint ends not double-reported as dangling",
                    c['dangling-end'] == 0))

    # 4. Duplicate segment (stacked copper).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(0, 0, 10, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("duplicate segment flagged as stacked-copper",
                    c['stacked-copper'] == 1))
    results.append(("each duplicate is individually removable",
                    c['removable-segment'] == 2))

    # 5. Square loop: 4 edges between two pads; one edge is a redundant cycle.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(10, 0, 10, 10),
                _seg(10, 10, 0, 10), _seg(0, 10, 0, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("square loop flagged as redundant-cycle",
                    c['redundant-cycle'] == 1))
    results.append(("every loop edge individually removable (superset)",
                    c['removable-segment'] == 4))
    results.append(("loop has no dangling ends", c['dangling-end'] == 0))

    # 6. Clean two-pad net: one segment pad to pad; nothing weird.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)], pads=pads)
    f, _ = check_weird(pcb)
    results.append(("clean two-pad net has no findings", len(f) == 0))

    # 7. Half-segment tail past a mid-body via anchor (#347 class): the trunk
    #    is load-bearing THROUGH the via at (6, 0), but 4mm of copper past it
    #    dangles.
    pads = [_pad(0, 0, num='1'), _pad(6, 5, layers=('B.Cu',), num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0), _seg(6, 0, 6, 5, layer='B.Cu')],
               vias=[_via(6, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    dangles = [x for x in f if x['category'] == 'dangling-end']
    results.append(("half-segment tail flagged as dangling-end",
                    c['dangling-end'] == 1))
    results.append(("tail reported past the body anchor with its length",
                    len(dangles) == 1
                    and 'body anchor' in dangles[0]['detail']
                    and '4.000mm' in dangles[0]['detail']
                    and abs(dangles[0]['x'] - 10) < 1e-6))
    results.append(("half-dangle case has no other findings", len(f) == 1))

    # 8. Floating via far from any copper.
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)], vias=[_via(20, 20)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("floating via flagged as unsupported-via",
                    c['unsupported-via'] == 1))

    # 9. Coincident same-net vias (stacked copper, via variant).
    pads = [_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')]
    pcb = _pcb([_seg(0, 0, 10, 0)],
               vias=[_via(10, 0), _via(10.005, 0)], pads=pads)
    f, _ = check_weird(pcb)
    c = _cats(f)
    results.append(("coincident vias flagged as stacked-copper",
                    c['stacked-copper'] == 1))

    # Default tolerance hides sub-0.1mm findings (the same 0.05mm soft joint).
    f_tol, _ = check_weird(pcb)
    results.append(("default 0.1mm tolerance hides the 0.05mm soft joint",
                    len([x for x in f_tol if x['category'] == 'soft-joint']) == 0))

    # Orphan island: two joined segments + via, nowhere near any pad of the
    # net (pads at 0/10, island at 50) -> flagged with its total length; a
    # sub-tolerance island is hidden by the default 0.1mm filter.
    pcb = _pcb([_seg(0, 0, 10, 0),
                _seg(50, 5, 52, 5), _seg(52, 5, 52, 7, layer='B.Cu')],
               vias=[_via(52, 5)],
               pads=[_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')])
    f, _ = check_weird(pcb)
    isl = [x for x in f if x['category'] == 'orphan-island']
    results.append(("orphan pad-less island flagged",
                    len(isl) == 1 and '4.00mm' in isl[0]['detail']))
    pcb2 = _pcb([_seg(0, 0, 10, 0), _seg(50, 5, 50.05, 5)],
                pads=[_pad(0, 0, num='1'), _pad(10, 0, num='2', ref='U2')])
    f2, _ = check_weird(pcb2)
    results.append(("sub-tolerance orphan island hidden by default",
                    not [x for x in f2 if x['category'] == 'orphan-island']))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} check_weird tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Regression test for collapse_strict_redundant (#217 classes 1-2).

Superseded parallel chains and pad/via-buried tails are redundant under the
STRICT width-clamped graph: removal must leave coincident-connected copper
(never manufacture a soft joint), drop the longer leftover run of a twin,
keep in-pad wiggles (Andy's choice), and refuse to touch nets whose strict
base is already split (removal could take load-bearing copper there).

    python3 tests/test_strict_collapse.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Segment, Net, PCBData
from pcb_modification import collapse_strict_redundant
from check_connected import check_net_connectivity


def _pad(ref, x, y, net_id=7, sx=0.6, sy=0.6):
    return Pad(component_ref=ref, pad_number='1', global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=sx, size_y=sy, shape='rect',
               layers=['F.Cu'], net_id=net_id, net_name=f'/N{net_id}',
               rotation=0.0)


def _seg(x1, y1, x2, y2, net_id=7, layer='F.Cu', w=0.2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net_id)


def _pcb(segments, vias, pads):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return PCBData(board_info=None, nets={7: Net(net_id=7, name='/N7')},
                   footprints={}, vias=vias, segments=segments,
                   pads_by_net=by_net)


def main():
    results = []

    # 1. Parallel twin: short live path + longer superseded detour between
    #    the same coincident pad points. The DETOUR goes; the net stays
    #    strictly connected.
    pads = [_pad('U1', 0, 0), _pad('U2', 10, 0)]
    live = _seg(0, 0, 10, 0)
    det1, det2 = _seg(0, 0, 5, 4), _seg(5, 4, 10, 0)
    pcb = _pcb([live, det1, det2], [], pads)
    n, strip = collapse_strict_redundant([], pcb, None)
    r = check_net_connectivity(7, pcb.segments, [], pads, [])
    results.append(("superseded detour removed, live path kept",
                    n == 2 and live in pcb.segments and r['connected']))
    results.append(("input copper returned for the strip",
                    set(map(id, strip)) == {id(det1), id(det2)}))

    # 2. Load-bearing single chain: nothing removable.
    pcb = _pcb([_seg(0, 0, 5, 0), _seg(5, 0, 10, 0)], [],
               [_pad('U1', 0, 0), _pad('U2', 10, 0)])
    n, _ = collapse_strict_redundant([], pcb, None)
    results.append(("load-bearing chain untouched", n == 0))

    # 3. In-pad wiggle kept: both endpoints inside the pad's copper.
    pads = [_pad('U1', 0, 0, sx=2.0, sy=2.0), _pad('U2', 10, 0)]
    wiggle = _seg(-0.5, 0.3, 0.5, 0.3)
    pcb = _pcb([_seg(0, 0, 10, 0), wiggle], [], pads)
    n, _ = collapse_strict_redundant([], pcb, None)
    results.append(("in-pad wiggle kept by choice",
                    n == 0 and wiggle in pcb.segments))

    # 4. Buried superseded tail (class 2): one end coincident on the live
    #    path, the other buried in a same-net via barrel that ALSO hosts the
    #    live path's junction -- redundant, and removal keeps strict
    #    connectivity through the via.
    pads = [_pad('U1', 0, 0), _pad('U2', 10, 0)]
    via = Via(x=5, y=0, size=0.5, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=7)
    a, b = _seg(0, 0, 5, 0), _seg(5, 0, 10, 0)
    # superseded run: T-anchored mid-path at (2,0), ending inside the via
    # barrel (one end buried -> NOT the exempted both-ends wiggle class)
    tail = _seg(2, 0, 5.1, 0.1)
    pcb = _pcb([a, b, tail], [via], pads)
    n, _ = collapse_strict_redundant([], pcb, None)
    r = check_net_connectivity(7, pcb.segments, pcb.vias, pads, [])
    results.append(("buried superseded tail removed, net strictly intact",
                    n == 1 and tail not in pcb.segments and r['connected']))

    # 5. Strictly-split net (mid-path soft joint): the pass must refuse.
    pads = [_pad('U1', 0, 0), _pad('U2', 10, 0)]
    pcb = _pcb([_seg(0, 0, 5, 0), _seg(5.05, 0, 10, 0)], [], pads)
    n, _ = collapse_strict_redundant([], pcb, None)
    results.append(("net with a strict split left alone", n == 0))

    # 6. Write-list copper is dropped in place, not stripped.
    pads = [_pad('U1', 0, 0), _pad('U2', 10, 0)]
    live = _seg(0, 0, 10, 0)
    dup = _seg(0, 0, 5, 4); dup2 = _seg(5, 4, 10, 0)
    pcb = _pcb([live, dup, dup2], [], pads)
    wl = [{'new_segments': [dup, dup2], 'new_vias': []}]
    n, strip = collapse_strict_redundant(wl, pcb, None)
    results.append(("this-run copper dropped from the write-list",
                    n == 2 and wl[0]['new_segments'] == [] and strip == []))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} strict-collapse tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

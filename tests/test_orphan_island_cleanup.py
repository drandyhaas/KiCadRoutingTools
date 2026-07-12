#!/usr/bin/env python3
"""Regression test for the orphan-island cleanup pass (#217).

Rip/reroute churn can strand a connected group of track copper that reaches
NO pad of its net (hackrf VREGMODE: 4 segments / 2.84mm connected to
nothing). Such an island evades every other subtractive pass: its interior
joints are degree-2 (not dead ends), its vias have tracks touching (not
unsupported), and removing any single segment splits it (not a redundant
cycle). remove_orphan_islands drops the whole component; removal cannot
affect any pad's connectivity by construction.

    python3 tests/test_orphan_island_cleanup.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Segment, Footprint, Net, PCBData
from pcb_modification import remove_orphan_islands
from check_connected import check_net_connectivity


def _pad(ref, num, x, y, net_id):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.6, size_y=0.6, shape='rect',
               layers=['F.Cu'], net_id=net_id, net_name=f'/N{net_id}',
               rotation=0.0)


def _seg(x1, y1, x2, y2, net_id, layer='F.Cu', graphic=False):
    s = Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                width=0.2, layer=layer, net_id=net_id)
    if graphic:
        s.graphic = True
    return s


def _pcb(segments, vias, pads):
    by_net = {}
    for p in pads:
        by_net.setdefault(p.net_id, []).append(p)
    return PCBData(board_info=None, nets={7: Net(net_id=7, name='/N7')},
                   footprints={}, vias=vias, segments=segments,
                   pads_by_net=by_net)


def main():
    results = []

    # Net 7: a real route pad1->pad2, plus a 3-segment orphan island with a
    # via, nowhere near the pads.
    pads = [_pad('U1', '1', 0, 0, 7), _pad('U2', '1', 10, 0, 7)]
    route = [_seg(0, 0, 10, 0, 7)]
    island = [_seg(50, 5, 52, 5, 7), _seg(52, 5, 52, 7, 7, layer='B.Cu'),
              _seg(52, 7, 54, 7, 7, layer='B.Cu')]
    via = Via(x=52, y=5, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=7)
    pcb = _pcb(route + island, [via], pads)
    write_list = [{'new_segments': [island[0]], 'new_vias': []}]

    n_isl, n_segs, strip, via_strip = remove_orphan_islands(write_list, pcb, None)
    results.append(("island removed (1 island, 3 segments)",
                    n_isl == 1 and n_segs == 3))
    results.append(("this-run segment dropped from write-list",
                    island[0] not in write_list[0]['new_segments']))
    results.append(("input segments returned for the writer strip",
                    set(map(id, strip)) == {id(island[1]), id(island[2])}))
    results.append(("pcb_data mutated in place (route copper kept)",
                    [s for s in pcb.segments] == route))
    r = check_net_connectivity(7, pcb.segments, pcb.vias, pads, [])
    results.append(("pads still fully connected", r['connected']))
    results.append(("island via stripped with its island",
                    via not in pcb.vias and via_strip == [via]))

    # A pad-connected chain must never be touched, nor an island anchored by
    # graphics copper (#337 immutable input art).
    pads2 = [_pad('U1', '1', 0, 0, 7), _pad('U2', '1', 10, 0, 7)]
    keep = [_seg(0, 0, 5, 0, 7), _seg(5, 0, 10, 0, 7)]
    art_island = [_seg(30, 5, 32, 5, 7), _seg(32, 5, 34, 5, 7, graphic=True)]
    pcb2 = _pcb(keep + art_island, [], pads2)
    n_isl, n_segs, strip, _vs = remove_orphan_islands([], pcb2, None)
    results.append(("graphics-anchored island kept; route kept",
                    n_isl == 0 and len(pcb2.segments) == 4))

    # Scope: an out-of-scope net's orphan island stays.
    pcb3 = _pcb(route[:] + [_seg(50, 5, 52, 5, 7)], [], pads)
    n_isl, _, _, _ = remove_orphan_islands([], pcb3, scope_net_ids={99})
    results.append(("out-of-scope island untouched", n_isl == 0))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} orphan-island cleanup tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

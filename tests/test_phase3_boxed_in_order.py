#!/usr/bin/env python3
"""Regression test for issue #347 (core1106 /RST1): Phase 3 must route the
most boxed-in pending terminals FIRST.

In insertion order, an early net's tap can wall in a later net's terminal
(a fine-pitch connector row's single escape corridor fits one tap); the
later net then only connects by ripping the early tap, whose re-route may
strand and veto the trade (#85). _order_nets_by_boxed_in_risk sorts pending
multipoint nets by the local foreign-copper density around their PENDING
terminals, descending, stable on ties.

    python3 tests/test_phase3_boxed_in_order.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Via, Footprint, Net, PCBData
from phase3_routing import _order_nets_by_boxed_in_risk


def _pad(ref, num, net_id, x, y):
    return Pad(component_ref=ref, pad_number=num, global_x=x, global_y=y,
               local_x=0, local_y=0, size_x=0.3, size_y=0.8, shape='rect',
               layers=['B.Cu'], net_id=net_id, net_name=f'/N{net_id}',
               rotation=0.0)


def _fp(ref, pads):
    fp = Footprint(reference=ref, footprint_name='test:conn', x=0, y=0,
                   rotation=0.0, layer='B.Cu', pads=pads)
    for p in pads:
        p.component_ref = ref
    return fp


def _result(pad_info, routed):
    return {'multipoint_pad_info': pad_info, 'routed_pad_indices': routed,
            'mst_edges': [(0, 1, 1.0)] * (len(pad_info) - 1),
            'new_segments': [], 'new_vias': []}


def main():
    results = []

    # A connector row at 0.5mm pitch (boxed) and a lone pad in open space.
    row = [_pad('J1', str(i + 1), 100 + i, 10.0 + 0.5 * i, 5.0) for i in range(6)]
    lone = _pad('R1', '1', 200, 50.0, 50.0)
    far = _pad('R1', '2', 200, 60.0, 50.0)
    far2 = _pad('U9', '1', 102, 30.0, 30.0)

    pcb = PCBData(
        board_info=None,
        nets={102: Net(net_id=102, name='/BOXED'),
              200: Net(net_id=200, name='/OPEN')},
        footprints={'J1': _fp('J1', row), 'R1': _fp('R1', [lone, far]),
                    'U9': _fp('U9', [far2])},
        vias=[Via(x=10.2, y=5.7, size=0.35, drill=0.2,
                  layers=['F.Cu', 'B.Cu'], net_id=101)],
        segments=[], pads_by_net={})

    # Net 200 (open space) inserted FIRST; its pending terminal is `far`.
    # Net 102 (row pad J1.3 + U9.1) second; pending terminal = row pad.
    open_info = [(0, 0, 0, lone.global_x, lone.global_y, lone),
                 (0, 0, 0, far.global_x, far.global_y, far)]
    boxed_info = [(0, 0, 0, far2.global_x, far2.global_y, far2),
                  (0, 0, 0, row[2].global_x, row[2].global_y, row[2])]
    pending = {200: _result(open_info, {0}), 102: _result(boxed_info, {0})}

    ordered = [k for k, _ in _order_nets_by_boxed_in_risk(pending, pcb)]
    results.append(("boxed connector-row terminal ordered first",
                    ordered == [102, 200]))

    # A terminal already ROUTED doesn't contribute risk: mark the row pad
    # routed and only the open terminal pending -> insertion order holds.
    pending2 = {200: _result(open_info, {0}), 102: _result(boxed_info, {0, 1})}
    ordered2 = [k for k, _ in _order_nets_by_boxed_in_risk(pending2, pcb)]
    results.append(("routed terminals contribute no risk (stable order)",
                    ordered2 == [200, 102]))

    # Equal risk -> stable insertion order: both pending terminals in empty
    # space (far and far2 each have no foreign copper within 1mm).
    iso_info = [(0, 0, 0, row[2].global_x, row[2].global_y, row[2]),
                (0, 0, 0, far2.global_x, far2.global_y, far2)]
    pending3 = {200: _result(open_info, {0}), 102: _result(iso_info, {0})}
    ordered3 = [k for k, _ in _order_nets_by_boxed_in_risk(pending3, pcb)]
    results.append(("ties keep insertion order", ordered3 == [200, 102]))

    # Single net: passthrough.
    ordered4 = _order_nets_by_boxed_in_risk({102: _result(boxed_info, {0})}, pcb)
    results.append(("single net passthrough", [k for k, _ in ordered4] == [102]))

    passed = 0
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
        passed += bool(ok)
    print(f"\n{passed}/{len(results)} phase-3 ordering tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

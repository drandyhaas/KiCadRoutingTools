#!/usr/bin/env python3
"""Regression test for issue #133 (different-net coincident-via short).

The signal route writes its output from the `results` write-list, not from
pcb_data. A result snapshot taken before a rip-reroute can keep referencing
copper that was later ripped off the board and not restored (e.g. a multipoint
net's completed_result, committed after try_phase3_ripup ripped that net's own
main route out from under it). Writing that phantom copper put a DIFFERENT-net
via at a cell another net legitimately took while this net was ripped --
EPHY_TX_N and EPHY_RX_P both ended with a via at (94.4, 105.8), a drill-on-drill
short -- even though pcb_data never held both vias at once.

drop_phantom_copper reconciles the write-list to the board by object identity:
copper still in pcb_data is kept, copper removed by a rip is dropped.

    python3 tests/test_phantom_copper.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from types import SimpleNamespace

from kicad_parser import Via, Segment
from pcb_modification import drop_phantom_copper


def _via(x, y, net_id):
    return Via(x=x, y=y, size=0.5, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=net_id)


def _seg(x1, y1, x2, y2, net_id):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=0.127,
                   layer='F.Cu', net_id=net_id)


def _empty_pcb():
    # drop_phantom_copper only reads .segments and .vias.
    return SimpleNamespace(vias=[], segments=[])


def run():
    failures = []

    # 1. A via still on the board is kept; a phantom via (in the result but ripped
    #    from pcb_data) is dropped. This is the #133 short: net 167's via is live,
    #    net 118's via at the same cell is phantom and must not be written.
    live_via = _via(94.4, 105.8, 167)
    phantom_via = _via(94.4, 105.8, 118)   # same cell, ripped -> not on board
    live_seg = _seg(0, 0, 1, 0, 167)
    phantom_seg = _seg(2, 2, 3, 2, 118)

    pcb = _empty_pcb()
    pcb.vias = [live_via]            # only 167's via survived on the board
    pcb.segments = [live_seg]

    r167 = {'new_vias': [live_via], 'new_segments': [live_seg]}
    r118 = {'new_vias': [phantom_via], 'new_segments': [phantom_seg]}
    results = [r167, r118]

    dropped_segs, dropped_vias = drop_phantom_copper(results, pcb)

    if dropped_vias != 1:
        failures.append(f"expected 1 phantom via dropped, got {dropped_vias}")
    if dropped_segs != 1:
        failures.append(f"expected 1 phantom segment dropped, got {dropped_segs}")
    if r167['new_vias'] != [live_via]:
        failures.append("live via was wrongly dropped")
    if r118['new_vias'] != []:
        failures.append("phantom via was not dropped")
    # No different-net coincident via remains in the write-list.
    written = [(round(v.x, 3), round(v.y, 3), v.net_id)
               for r in results for v in r['new_vias']]
    cell_nets = {}
    for x, y, nid in written:
        cell_nets.setdefault((x, y), set()).add(nid)
    if any(len(nids) > 1 for nids in cell_nets.values()):
        failures.append(f"different-net coincident via still in write-list: {written}")

    # 2. Identity, not position: a same-position via that is a DIFFERENT object but
    #    genuinely on the board is kept (a re-placed via must not be mistaken for a
    #    ripped one).
    pcb2 = _empty_pcb()
    board_via = _via(10.0, 10.0, 5)
    pcb2.vias = [board_via]
    result_via_same_pos = _via(10.0, 10.0, 5)   # different object, same position
    r = {'new_vias': [board_via, result_via_same_pos], 'new_segments': []}
    _, dv = drop_phantom_copper([r], pcb2)
    if dv != 1:
        failures.append(f"identity case: expected 1 dropped, got {dv}")
    if r['new_vias'] != [board_via]:
        failures.append("identity case: kept the wrong via object")

    # 3. A fully live result is untouched.
    pcb3 = _empty_pcb()
    v = _via(1, 1, 9)
    s = _seg(0, 0, 1, 1, 9)
    pcb3.vias = [v]
    pcb3.segments = [s]
    r = {'new_vias': [v], 'new_segments': [s]}
    ds, dv = drop_phantom_copper([r], pcb3)
    if (ds, dv) != (0, 0):
        failures.append(f"live result was disturbed: dropped {(ds, dv)}")

    print("=" * 60)
    if failures:
        for f in failures:
            print(f"  FAIL  {f}")
        print(f"\n{len(failures)} failure(s)")
        return 1
    print("  PASS  phantom via dropped, live copper kept, identity-matched")
    print("  PASS  same-position different-object via kept")
    print("  PASS  fully-live result untouched")
    print("\n3/3 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

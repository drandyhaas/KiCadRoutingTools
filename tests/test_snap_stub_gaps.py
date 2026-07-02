#!/usr/bin/env python3
"""snap_stub_gaps closes a small same-net gap when clear, and refuses when the
connector would violate another net's clearance or the gap is too big (#84).

    python3 tests/test_snap_stub_gaps.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment
from pcb_modification import snap_stub_gaps


def _seg(x1, y1, x2, y2, net=5, layer='F.Cu', w=0.1):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w, layer=layer, net_id=net)


def _pcb(segments, pads_by_net=None):
    nets = {}
    for s in segments:
        nets.setdefault(s.net_id, SimpleNamespace(pads=[]))
    pbn = pads_by_net or {}
    for nid, pads in pbn.items():
        nets.setdefault(nid, SimpleNamespace(pads=pads))
        nets[nid].pads = pads
    return SimpleNamespace(segments=list(segments), vias=[],
                           pads_by_net=pbn, nets=nets)


class Cfg:
    clearance = 0.1


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. A stub ends 0.12mm short of a same-net trace (gap < 1.5*0.1=0.15); no
    #    other nets -> snapped with one connector.
    trunk = _seg(0, 0, 10, 0)                 # horizontal trace at y=0
    stub = _seg(5, 2, 5, 0.12)                # vertical stub ending 0.12mm above trunk
    pcb = _pcb([trunk, stub])
    results = []
    n = snap_stub_gaps(results, pcb, {5}, Cfg())
    check("1 one connector added", n == 1)
    check("1 connector lands on trunk", results and
          abs(results[0]['new_segments'][0].end_y - 0.0) < 1e-6)

    # 2. Gap too big (0.4mm > 1.5*0.1) -> not snapped.
    pcb2 = _pcb([_seg(0, 0, 10, 0), _seg(5, 2, 5, 0.4)])
    r2 = []
    check("2 gap beyond limit not snapped", snap_stub_gaps(r2, pcb2, {5}, Cfg()) == 0)

    # 3. Another net's trace sits in the connector's path (within clearance) ->
    #    snap refused.
    trunk3 = _seg(0, 0, 10, 0, net=5)
    stub3 = _seg(5, 2, 5, 0.12, net=5)
    blocker = _seg(4, 0.06, 6, 0.06, net=9)   # net 9 right between stub end and trunk
    pcb3 = _pcb([trunk3, stub3, blocker])
    r3 = []
    check("3 blocked connector refused", snap_stub_gaps(r3, pcb3, {5}, Cfg()) == 0)

    # 4. Stub to a same-net pad just outside its copper -> snapped onto the pad.
    pad = SimpleNamespace(global_x=5.0, global_y=0.0, size_x=0.5, size_y=0.5,
                          layers=['F.Cu'], rotation=0.0)
    stub4 = _seg(5, 0.6, 5, 0.37, net=5)      # ends 0.12mm above pad copper edge (0.25)
    pcb4 = _pcb([stub4], pads_by_net={5: [pad]})
    r4 = []
    check("4 snap to pad copper", snap_stub_gaps(r4, pcb4, {5}, Cfg()) == 1)

    # 5. Another net's copper pour (zone) lies in the connector's path -> refused
    #    (the connector must not enter another net's plane).
    trunk5 = _seg(0, 0, 10, 0, net=5)
    stub5 = _seg(5, 2, 5, 0.12, net=5)
    zone = SimpleNamespace(net_id=9, net_name='GND', layer='F.Cu',
                           polygon=[(4, 0.02), (6, 0.02), (6, 0.1), (4, 0.1)])
    pcb5 = _pcb([trunk5, stub5])
    pcb5.zones = [zone]
    r5 = []
    check("5 connector into another net's pour refused", snap_stub_gaps(r5, pcb5, {5}, Cfg()) == 0)

    # 6. (#281) The connector would cross a board CUTOUT (reverse-mount LED
    #    window) -> refused, even though no foreign copper is in the way.
    trunk6 = _seg(0, 0, 10, 0, net=5)
    stub6 = _seg(5, 2, 5, 0.12, net=5)
    pcb6 = _pcb([trunk6, stub6])
    pcb6.board_info = SimpleNamespace(
        board_outline=[(-5, -5), (15, -5), (15, 10), (-5, 10)],
        board_cutouts=[[(4.5, 0.03), (5.5, 0.03), (5.5, 0.09), (4.5, 0.09)]])
    r6 = []
    check("6 connector across a cutout refused (#281)",
          snap_stub_gaps(r6, pcb6, {5}, Cfg()) == 0)
    # ...and with the cutout elsewhere the same snap succeeds.
    pcb6b = _pcb([_seg(0, 0, 10, 0, net=5), _seg(5, 2, 5, 0.12, net=5)])
    pcb6b.board_info = SimpleNamespace(
        board_outline=[(-5, -5), (15, -5), (15, 10), (-5, 10)],
        board_cutouts=[[(8, 5), (9, 5), (9, 6), (8, 6)]])
    r6b = []
    check("6b connector clear of cutout snapped",
          snap_stub_gaps(r6b, pcb6b, {5}, Cfg()) == 1)

    # 7. (#281) A 'custom'-shaped pad's size is only a bounding box; when a
    #    coincident non-custom anchor pad exists, the snap must target the
    #    anchor's (smaller, real-copper) extent -- here the custom bbox corner
    #    is within snap range but the anchor is not, so no connector.
    custom = SimpleNamespace(global_x=5.0, global_y=-1.0, size_x=0.5, size_y=2.2,
                             layers=['F.Cu'], rotation=0.0, shape='custom',
                             component_ref='LED1', pad_number='1')
    anchor = SimpleNamespace(global_x=5.0, global_y=-1.0, size_x=0.5, size_y=0.8,
                             layers=['F.Cu'], rotation=0.0, shape='roundrect',
                             component_ref='LED1', pad_number='1')
    stub7 = _seg(5, 0.5, 5, 0.22, net=5)   # 0.12 above custom bbox top (0.1)
    pcb7 = _pcb([stub7], pads_by_net={5: [custom, anchor]})
    r7 = []
    check("7 custom pad bbox not trusted when anchor exists",
          snap_stub_gaps(r7, pcb7, {5}, Cfg()) == 0)

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  snaps a clear small gap (trace + pad), refuses a too-big gap,")
    print("        a connector blocked by another net / a pour / a board cutout,")
    print("        and doesn't trust a custom pad's bbox (8 cases)")
    print("\n8/8 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

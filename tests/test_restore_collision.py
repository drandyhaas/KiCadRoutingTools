#!/usr/bin/env python3
"""restore_net must refuse to re-add a ripped net's saved copper when doing so
would short another net's copper that occupied its corridor while it was ripped
(#134). A non-colliding restore must still go through.

    python3 tests/test_restore_collision.py
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment, Via
from rip_up_reroute import _saved_route_collides


def _seg(x1, y1, x2, y2, net, layer='F.Cu', w=0.127):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=w, layer=layer, net_id=net)


def _via(x, y, net, size=0.3):
    return Via(x=x, y=y, size=size, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=net)


def _pcb(segments, vias=None):
    return SimpleNamespace(segments=list(segments), vias=list(vias or []))


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. The real ottercast case: RESETn's saved F.Cu segment lies collinear on
    #    top of an IPS segment now occupying the same corridor -> collides.
    ips = _seg(107.45, 109.80, 104.25, 109.80, net=45)
    resetn_saved = {'new_segments': [_seg(106.15, 109.80, 104.25, 109.80, net=93)],
                    'new_vias': []}
    pcb = _pcb([ips])
    check("1 collinear different-net overlap detected",
          _saved_route_collides(resetn_saved, pcb, [93], clearance=0.1))

    # 2. Same geometry but IPS belongs to the net being restored (own) -> no short.
    pcb2 = _pcb([_seg(107.45, 109.80, 104.25, 109.80, net=93)])
    check("2 same-net overlap is not a collision",
          not _saved_route_collides(resetn_saved, pcb2, [93], clearance=0.1))

    # 3. Restored via dropped onto another net's inner trace (the RESETn-via on
    #    EPHY_RX_P case) -> collides. (Via spans layers, trace on In1.Cu.)
    via_saved = {'new_segments': [],
                 'new_vias': [_via(107.25, 107.10, net=93)]}
    ephy = _seg(102.75, 107.05, 107.70, 107.05, net=167, layer='In1.Cu')
    check("3 restored via on other-net trace detected",
          _saved_route_collides(via_saved, _pcb([ephy]), [93], clearance=0.1))

    # 4. Restore well clear of everything -> allowed.
    far = _seg(50.0, 50.0, 55.0, 50.0, net=45)
    clean_saved = {'new_segments': [_seg(10.0, 10.0, 15.0, 10.0, net=93)],
                   'new_vias': []}
    check("4 clear restore not a collision",
          not _saved_route_collides(clean_saved, _pcb([far]), [93], clearance=0.1))

    # 5. Parallel other-net trace just outside clearance -> allowed; just inside
    #    -> refused.
    saved5 = {'new_segments': [_seg(0, 0, 10, 0, net=93)], 'new_vias': []}
    near = _seg(0, 0.30, 10, 0.30, net=45)   # gap 0.30 - 0.127 = 0.173 > 0.1
    far5 = _seg(0, 0.20, 10, 0.20, net=45)    # gap 0.20 - 0.127 = 0.073 < 0.1
    check("5a parallel beyond clearance allowed",
          not _saved_route_collides(saved5, _pcb([near]), [93], clearance=0.1))
    check("5b parallel within clearance refused",
          _saved_route_collides(saved5, _pcb([far5]), [93], clearance=0.1))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  refuses restores that short other-net copper (collinear seg,")
    print("        via-on-trace, within-clearance parallel); allows same-net and")
    print("        clear restores")
    print("\n6/6 checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(run())

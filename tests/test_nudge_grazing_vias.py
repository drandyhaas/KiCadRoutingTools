#!/usr/bin/env python3
"""nudge_grazing_vias (#280): a via a few um inside clearance of a foreign
via/track/hole moves ALONE by its sub-grid shortfall; boxed-in or badly
mis-placed vias stay put and stay visible in DRC.

    python3 tests/test_nudge_grazing_vias.py
"""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Segment, Via
from pcb_modification import nudge_grazing_vias

CLR = 0.127
H2H = 0.20


def _via(x, y, net, size=0.45, drill=0.3):
    return Via(x=x, y=y, size=size, drill=drill, layers=['F.Cu', 'B.Cu'], net_id=net)


def _seg(x1, y1, x2, y2, net, layer='F.Cu', w=0.127):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=w,
                   layer=layer, net_id=net)


def _pcb(segments, vias):
    nets = {}
    for o in list(segments) + list(vias):
        nets.setdefault(o.net_id, SimpleNamespace(pads=[]))
    return SimpleNamespace(segments=list(segments), vias=list(vias),
                           pads_by_net={}, footprints={}, nets=nets, zones=[],
                           board_info=SimpleNamespace(
                               copper_layers=['F.Cu', 'B.Cu'],
                               board_outline=[], board_cutouts=[],
                               board_bounds=None))


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # 1. usb_sniffer geometry: two 0.45 vias 0.570mm apart = 7um inside the
    #    0.127 clearance. Only net 2's via is in scope -> IT moves, the graze
    #    clears, and the move respects both caps (grid/2=0.025, size/4=0.1125).
    a = _via(59.85, 62.45, net=1)          # signal via (foreign, static)
    b = _via(60.40, 62.60, net=2)          # plane stitch via (scoped)
    # anchor each via with a short same-net track so connectivity is checkable
    segs = [_seg(59.85, 62.45, 59.0, 62.45, net=1),
            _seg(60.40, 62.60, 61.3, 62.60, net=2)]
    pcb = _pcb(segs, [a, b])
    moved, nets, moves = nudge_grazing_vias([{'new_vias': [b]}], pcb, {2},
                                            clearance=CLR,
                                            hole_to_hole=H2H, max_shift=0.025)
    d = math.hypot(b.x - a.x, b.y - a.y)
    shift = math.hypot(b.x - 60.40, b.y - 62.60)
    check("1 via moved", moved == 1 and moves and moves[0][0] == 2)
    check("1 graze cleared", d >= 0.45 + CLR - 1e-6)
    check("1 shift under caps", 0 < shift <= min(0.025, 0.45 / 4) + 1e-9)
    check("1 foreign via untouched", (a.x, a.y) == (59.85, 62.45))

    # 2. Shortfall beyond the cap (via 60um inside clearance, cap 25um) ->
    #    left alone, stays visible in DRC.
    a2 = _via(0.0, 0.0, net=1)
    b2 = _via(0.517, 0.0, net=2)           # needs 0.06 -> over the 0.025 cap
    pcb2 = _pcb([_seg(0, 0, -1, 0, net=1), _seg(0.517, 0, 1.5, 0, net=2)],
                [a2, b2])
    m2, _, _ = nudge_grazing_vias([{'new_vias': [b2]}], pcb2, {2},
                                  clearance=CLR,
                                  hole_to_hole=H2H, max_shift=0.025)
    check("2 over-cap via not moved", m2 == 0 and (b2.x, b2.y) == (0.517, 0.0))

    # 3. Boxed in: foreign copper on every side at the same shortfall -> no
    #    clean direction, via stays.
    c = _via(0.0, 0.0, net=2)
    ring = [_via(0.570, 0.0, net=1), _via(-0.570, 0.0, net=1),
            _via(0.0, 0.570, net=1), _via(0.0, -0.570, net=1)]
    segs3 = [_seg(0, 0, 0, 0.001, net=2)] + \
            [_seg(v.x, v.y, v.x + 0.5, v.y, net=1) for v in ring]
    pcb3 = _pcb(segs3, [c] + ring)
    m3, _, _ = nudge_grazing_vias([{'new_vias': [c]}], pcb3, {2},
                                  clearance=CLR,
                                  hole_to_hole=H2H, max_shift=0.025)
    check("3 boxed-in via not moved", m3 == 0 and (c.x, c.y) == (0.0, 0.0))

    # 4. Drill hole-to-hole (SAME net, no copper conflict): two 0.3 drills
    #    0.49mm apart = 10um inside the 0.20 hole-to-hole floor -> nudged.
    d1 = _via(0.0, 0.0, net=2)
    d2 = _via(0.49, 0.0, net=2)
    pcb4 = _pcb([_seg(0, 0, -1, 0, net=2), _seg(0.49, 0, 1.5, 0, net=2)],
                [d1, d2])
    m4, _, _ = nudge_grazing_vias([{'new_vias': [d1, d2]}], pcb4, {2},
                                  clearance=CLR,
                                  hole_to_hole=H2H, max_shift=0.025)
    gap_h = math.hypot(d2.x - d1.x, d2.y - d1.y) - 0.3 - H2H
    check("4 same-net hole-to-hole nudged", m4 >= 1 and gap_h >= -1e-6)

    # 5. A via NOT created by this run (absent from results['new_vias']) is
    #    never moved -- the writers keep the input file's text for it, so a
    #    move would silently revert in the output.
    a5 = _via(59.85, 62.45, net=1)
    b5 = _via(60.40, 62.60, net=2)
    pcb5 = _pcb([_seg(59.85, 62.45, 59.0, 62.45, net=1),
                 _seg(60.40, 62.60, 61.3, 62.60, net=2)], [a5, b5])
    m5, _, _ = nudge_grazing_vias([{'new_vias': []}], pcb5, {2}, clearance=CLR,
                                  hole_to_hole=H2H, max_shift=0.025)
    check("5 input-file via never moved",
          m5 == 0 and (b5.x, b5.y) == (60.40, 62.60))

    print("=" * 60)
    if fails:
        for f in fails:
            print(f"  FAIL  {f}")
        print(f"\n{len(fails)} failure(s)")
        return 1
    print("  PASS  moves a grazing via by its sub-grid shortfall (copper and")
    print("        hole-to-hole), refuses over-cap, boxed-in, and input-file vias (5 cases)")
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())

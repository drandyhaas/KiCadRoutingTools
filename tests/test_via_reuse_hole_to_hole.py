#!/usr/bin/env python3
"""merge_close_same_net_vias must reuse a same-net via that another plane via lands
within hole-to-hole clearance of (dropping the duplicate, reconnecting its segment
to the survivor), and must NOT merge across nets or beyond the clearance.

Root case: route_planes phase-3 placed two same-net VCC through-vias 0.05mm apart
tapping decoupling cap C133.1 (hackrf) -> overlapping drills, a check_drc
via-drill-hole KiCad net-unifies and hides. Same-net copper isn't an obstacle, so
the per-net hole-to-hole via block never fires between them.

    python3 tests/test_via_reuse_hole_to_hole.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pcb_modification import merge_close_same_net_vias


class _Via:
    def __init__(self, x, y, drill=0.2, size=0.45, layers=('F.Cu', 'B.Cu'), net_id=1):
        self.x, self.y, self.drill, self.size = x, y, drill, size
        self.layers, self.net_id = list(layers), net_id


class _PCB:
    def __init__(self, vias, pads_by_net=None):
        self.vias = vias
        self.nets = {}
        self.pads_by_net = pads_by_net or {}


class _Pad:
    def __init__(self, x, y, drill=1.0, net_id=1):
        self.global_x, self.global_y = x, y
        self.hole_x = self.hole_y = None
        self.drill = drill
        self.net_id = net_id
        self.pad_type = 'thru_hole'
        self.layers = ['*.Cu', '*.Mask']


def _v(x, y, net_id=1, drill=0.2, layers=('F.Cu', 'B.Cu')):
    return {'x': x, 'y': y, 'drill': drill, 'size': 0.45,
            'layers': list(layers), 'net_id': net_id}


def run():
    fails = []

    def check(name, cond):
        if not cond:
            fails.append(name)

    # (1) Two NEW same-net through-vias 0.05mm apart -> one merged away; the segment
    #     that ended on the dropped via is reconnected to the survivor.
    vias = [_v(140.65, 151.25), _v(140.65, 151.20)]
    segs = [{'start': (140.65, 151.20), 'end': (140.20, 151.20),
             'width': 0.127, 'layer': 'F.Cu', 'net_id': 1}]
    n = merge_close_same_net_vias(vias, segs, _PCB([]), hole_to_hole_clearance=0.25)
    surv = (vias[0]['x'], vias[0]['y'])
    check("merges one of two near same-net vias", n == 1 and len(vias) == 1)
    check("segment reconnected to survivor",
          segs[0]['start'] == surv)

    # (2b, #479 gap 3) Reuse a plated THT PAD barrel: a new via within
    #     hole-to-hole of a same-net barrel merges onto it; the attached
    #     segment re-anchors to the barrel position.
    vias = [_v(60.00, 60.75)]
    segs = [{'start': (60.00, 60.75), 'end': (61.50, 60.75),
             'width': 0.127, 'layer': 'F.Cu', 'net_id': 1}]
    n = merge_close_same_net_vias(vias, segs, _PCB([], {1: [_Pad(60.00, 60.00)]}),
                                  hole_to_hole_clearance=0.25)
    check("new via merges onto same-net THT barrel", n == 1 and len(vias) == 0)
    check("segment re-anchored to barrel", segs[0]['start'] == (60.00, 60.00))
    # ...but a FOREIGN barrel must not swallow the via.
    vias = [_v(70.00, 70.75)]
    n = merge_close_same_net_vias(vias, [], _PCB([], {2: [_Pad(70.00, 70.00, net_id=2)]}),
                                  hole_to_hole_clearance=0.25)
    check("foreign barrel does not merge", n == 0 and len(vias) == 1)

    # (2) Reuse a PRE-EXISTING board via: the new via drops, no new via remains.
    vias = [_v(50.00, 50.03)]
    n = merge_close_same_net_vias(vias, [], _PCB([_Via(50.00, 50.00)]),
                                  hole_to_hole_clearance=0.25)
    check("new via reuses existing same-net via", n == 1 and len(vias) == 0)

    # (3) Different nets within range -> NOT merged (hole-to-hole is enforced during
    #     routing for cross-net; this pass is same-net reuse only).
    vias = [_v(60.00, 60.03, net_id=2)]
    n = merge_close_same_net_vias(vias, [], _PCB([_Via(60.00, 60.00, net_id=1)]),
                                  hole_to_hole_clearance=0.25)
    check("does not merge across nets", n == 0 and len(vias) == 1)

    # (4) Same net but comfortably beyond hole-to-hole -> both kept (legit stitching).
    vias = [_v(70.00, 70.00), _v(70.00, 71.00)]  # 1mm apart, drill 0.2, h2h 0.25
    n = merge_close_same_net_vias(vias, [], _PCB([]), hole_to_hole_clearance=0.25)
    check("keeps well-separated same-net vias", n == 0 and len(vias) == 2)

    # (5) Survivor span must COVER the dropped via's layers; a blind via whose layers
    #     the nearby via doesn't span is left in place (can't reuse safely).
    vias = [_v(80.00, 80.03, layers=('In1.Cu', 'In2.Cu'))]
    n = merge_close_same_net_vias(vias, [], _PCB([_Via(80.00, 80.00,
                                  layers=('F.Cu', 'In1.Cu'))]),
                                  hole_to_hole_clearance=0.25)
    check("does not merge when survivor span misses a layer", n == 0 and len(vias) == 1)

    if fails:
        print("FAIL: " + ", ".join(fails))
        return 1
    print("PASS: same-net via reuse merges hole-to-hole violators, respects nets/span/distance")
    return 0


if __name__ == "__main__":
    sys.exit(run())

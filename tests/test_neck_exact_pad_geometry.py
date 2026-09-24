"""neck_wide_segments_grazing_pads must judge a graze against the pad's REAL
copper (the geometry check_drc grades), not its bounding circle
max(size_x, size_y)/2. On an elongated pad (0.3 x 1.5 mm QFN/connector pad)
the circle reaches 0.6 mm past the long side, so a legal wide trace running
beside it was necked to the layer default -- and a real graze near the long
side was missed because the circle said even the default width could not clear.
The cheap pre-reject must use the CIRCUMSCRIBED circle: max(size)/2 falls short
of a rect's corners, so a corner graze was rejected before the exact check (F).
"""
import math
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_router'))  # #522
sys.path.insert(0, os.path.join(os.path.join(os.path.dirname(__file__), '..'), 'py_tools'))  # #522

from kicad_parser import Pad, Segment  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
from pcb_modification import neck_wide_segments_grazing_pads  # noqa: E402

WIDE, DEFAULT, CLR = 0.4, 0.1, 0.1


def pad(rect_rotation=0.0, size=(0.3, 1.5)):
    # 0.3 x 1.5 rect at the origin, foreign net 2. Real long side at x=+-0.15,
    # short end at y=+-0.75; bounding circle radius 0.75.
    return Pad(component_ref='J1', pad_number='1', global_x=0.0, global_y=0.0,
               local_x=0.0, local_y=0.0, size_x=size[0], size_y=size[1], shape='rect',
               layers=['F.Cu'], net_id=2, net_name='B', rect_rotation=rect_rotation)


def run(seg, p):
    pcb = SimpleNamespace(footprints={'J1': SimpleNamespace(pads=[p])})
    cfg = GridRouteConfig(track_width=DEFAULT, clearance=CLR, layers=['F.Cu'])
    n = neck_wide_segments_grazing_pads([{'new_segments': [seg]}], pcb, cfg)
    return n, seg.width


def seg(x1, y1, x2, y2):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=WIDE, layer='F.Cu', net_id=1)


def rot(x, y, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return x * c - y * s, x * s + y * c


def main():
    fails = []

    def check(name, got, want):
        ok = got == want
        print(f"{'ok  ' if ok else 'FAIL'} {name}: necked={got[0]} width={got[1]} "
              f"(want necked={want[0]} width={want[1]})")
        if not ok:
            fails.append(name)

    # A. Beside the long side, centreline x=0.95: real gap 0.95-0.15-0.2 = 0.60
    #    (legal). The circle reads 0.95-0.75-0.2 = 0.00 < 0.1 -> wrongly necked.
    check("A legal beside long side", run(seg(0.95, -0.5, 0.95, 0.5), pad()),
          (0, WIDE))

    # B. True graze on the long side, x=0.4: real gap 0.05 < 0.1, default
    #    clears (0.20) -> must neck. The circle missed it (default "fails").
    check("B graze on long side", run(seg(0.4, -0.5, 0.4, 0.5), pad()),
          (1, DEFAULT))

    # C. True graze off the short end, y=0.95: real gap 0.00 < 0.1, default
    #    clears (0.15) -> must neck (both old and new).
    check("C graze off short end", run(seg(-0.5, 0.95, 0.5, 0.95), pad()),
          (1, DEFAULT))

    # D. Case A rotated 30 deg with the pad (rect_rotation): still legal.
    (x1, y1), (x2, y2) = rot(0.95, -0.5, 30), rot(0.95, 0.5, 30)
    check("D legal beside rotated pad", run(seg(x1, y1, x2, y2), pad(30.0)),
          (0, WIDE))

    # E. Case B rotated 30 deg: still a graze -> neck.
    (x1, y1), (x2, y2) = rot(0.4, -0.5, 30), rot(0.4, 0.5, 30)
    check("E graze beside rotated pad", run(seg(x1, y1, x2, y2), pad(30.0)),
          (1, DEFAULT))

    # F. Corner graze on a 1 x 1 rect: a segment square to the diagonal, its
    #    centreline 0.957 from the centre, so its edge is 0.05 off the corner
    #    (0.707 out) -- a violation that necking to 0.1 clears (gap 0.20).
    #    max(size)/2 = 0.5 read a 0.257 gap and rejected it before the exact check.
    c, h = 0.957 / math.sqrt(2), 0.5 / math.sqrt(2)
    check("F graze off a rect corner",
          run(seg(c - h, c + h, c + h, c - h), pad(size=(1.0, 1.0))),
          (1, DEFAULT))

    if fails:
        print(f"FAIL: {len(fails)} case(s): {', '.join(fails)}")
        sys.exit(1)
    print("PASS: neck-down judges pad grazing on exact pad copper")


if __name__ == '__main__':
    main()

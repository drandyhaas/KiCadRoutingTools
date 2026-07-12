#!/usr/bin/env python3
"""Foreign-pad distance uses the pad's true shape, not a bounding rect (#315).

route.py's graze/neck/nudge passes measure clearance to foreign pads with
_seg_foreign_pad_dist. Modelling every pad as its bounding RECTANGLE makes a
round BGA ball's corner stick out ~half its diameter past the real copper, so a
track running past the ball at true-clear distance reads as a phantom graze --
and the graze passes then mangle perfectly clean fanout copper to "fix" it
(butterstick DQ5 vs the round DQ6 ball). The rounded-rect model (per-pad corner
radius) measures circle/oval/roundrect pads by their real outline.

    python3 tests/test_pad_shape_distance.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, Net, PCBData, BoardInfo
from single_ended_routing import _pad_corner_radius, _seg_foreign_pad_dist, _pt_foreign_pad_dist


def _pad(shape, x, y, sx, sy, net_id, rratio=0.0, rot=0.0):
    return Pad(component_ref='U1', pad_number='1', global_x=x, global_y=y,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
               layers=['F.Cu'], net_id=net_id, net_name='/OTHER', drill=0,
               roundrect_rratio=rratio, rect_rotation=rot)


def _board(pad):
    return PCBData(footprints={}, nets={1: Net(1, '/SIG'), 2: Net(2, '/OTHER')},
                   segments=[], vias=[],
                   board_info=BoardInfo(layers={}, copper_layers=['F.Cu', 'B.Cu']),
                   pads_by_net={2: [pad]})


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    # --- corner radius per shape ---
    check("circle corner_r = radius", abs(_pad_corner_radius(_pad('circle', 0, 0, 0.4, 0.4, 2)) - 0.2) < 1e-9)
    check("oval corner_r = min/2", abs(_pad_corner_radius(_pad('oval', 0, 0, 0.6, 0.4, 2)) - 0.2) < 1e-9)
    check("roundrect corner_r = rratio*min", abs(_pad_corner_radius(_pad('roundrect', 0, 0, 0.5, 0.4, 2, rratio=0.25)) - 0.1) < 1e-9)
    check("rect corner_r = 0", _pad_corner_radius(_pad('rect', 0, 0, 0.4, 0.4, 2)) == 0.0)
    check("rotated circle keeps its radius (local-frame SDF, #356)",
          abs(_pad_corner_radius(_pad('circle', 0, 0, 0.4, 0.4, 2, rot=30.0)) - 0.2) < 1e-9)

    # --- circle pad: distance to a diagonal approach is the TRUE circle distance,
    #     NOT the bounding-rect corner (the #315 phantom). ---
    # Circle centre (0.7,0.9) r=0.2; a track point at (0.5,0.6) is off the corner.
    circle = _pad('circle', 0.7, 0.9, 0.4, 0.4, 2)
    pcb = _board(circle)
    true_edge = math.hypot(0.7 - 0.5, 0.9 - 0.6) - 0.2   # dist to centre - radius
    rect_edge = math.hypot(max(abs(0.5 - 0.7) - 0.2, 0), max(abs(0.6 - 0.9) - 0.2, 0))  # old bbox corner
    d = _pt_foreign_pad_dist(pcb, 1, 0.5, 0.6, 'F.Cu')
    check("circle: point distance = true circle edge (not rect corner)",
          abs(d - true_edge) < 1e-6, f"got {d:.4f}, circle {true_edge:.4f}, oldrect {rect_edge:.4f}")
    check("circle model differs from bbox at the corner", abs(true_edge - rect_edge) > 0.05)

    # segment sweeping past the ball: same true-circle result
    ds = _seg_foreign_pad_dist(pcb, 1, 0.3, 0.6, 0.5, 0.6, 'F.Cu')
    check("circle: segment distance uses true circle edge", ds >= true_edge - 1e-6)

    # --- rect pad: model is UNCHANGED (exact bounding rect) ---
    rect = _pad('rect', 0.7, 0.9, 0.4, 0.4, 2)
    pcbr = _board(rect)
    dr = _pt_foreign_pad_dist(pcbr, 1, 0.5, 0.6, 'F.Cu')
    check("rect: point distance = exact rect corner (unchanged)", abs(dr - rect_edge) < 1e-6,
          f"got {dr:.4f}, rect {rect_edge:.4f}")

    # --- rotated oval: distance is measured against the TILTED capsule, not the
    #     unrotated rect (#356: bfo9000 SW_A2.2, a 4.21x2.25 oval at -48.1 deg --
    #     the old board-axis model under-covered its lower capsule end by ~2mm and
    #     nudge_grazing_octolinear re-bent an SDA track 0.054mm from its copper). ---
    tilt_deg = -48.099999999999994
    tilt = _pad('oval', 73.04499547808979, 53.8799956103105, 4.211556, 2.25, 2, rot=tilt_deg)
    pcbt = _board(tilt)
    # True capsule: axis half-length (4.211556-2.25)/2, radius 2.25/2, tilted.
    th = math.radians(tilt_deg)
    axl = (4.211556 - 2.25) / 2.0
    e1 = (73.04499547808979 + axl * math.cos(th), 53.8799956103105 + axl * math.sin(th))
    e2 = (73.04499547808979 - axl * math.cos(th), 53.8799956103105 - axl * math.sin(th))

    def capsule_dist(px, py):
        # min over the two end circles and the (finite) axis segment
        ax, ay = e2; bx, by = e1
        abx, aby = bx - ax, by - ay
        tt = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / (abx * abx + aby * aby)))
        qx, qy = ax + tt * abx, ay + tt * aby
        return math.hypot(px - qx, py - qy) - 2.25 / 2.0

    # The point where the routed SDA track grazed on the real board.
    for (px, py) in ((72.2, 55.9), (71.9, 55.9), (72.5, 56.2), (75.0, 54.0), (70.5, 53.0)):
        want = capsule_dist(px, py)
        got = _pt_foreign_pad_dist(pcbt, 1, px, py, 'F.Cu')
        check(f"tilted oval: point ({px},{py}) = true capsule distance",
              abs(got - want) < 1e-6, f"got {got:.4f}, want {want:.4f}")
    # The exact graze segment: true distance is sub-clearance (0.179mm edge);
    # the old model reported ~0.9mm and let the nudge pass park a track there.
    dseg = _seg_foreign_pad_dist(pcbt, 1, 72.2, 55.9, 71.6, 55.9, 'F.Cu')
    check("tilted oval: graze segment reads sub-clearance (not 0.9mm)",
          dseg < 0.2, f"got {dseg:.4f}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())

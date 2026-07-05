#!/usr/bin/env python3
"""Drill keep-outs model a milled SLOT as a capsule, not a circle.

The hole-to-hole keep-outs (obstacle_map via-vs-pad-drill, plane_region_connector,
add_gnd_vias) used `pad.drill / 2` as a plain circle at the pad centre. For a
milled SLOT drill (drill_w != drill_h) that is wrong: a via placed at the slot's
END is far from the centre but right on the copper-free hole, so the circle
model lets it through (a real hole-to-hole violation). The capsule model
(pad_drill_capsule + distance to the slot axis) covers the whole slot.

    python3 tests/test_slot_drill_keepout.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, pad_drill_capsule
from geometry_utils import point_to_segment_distance


def _slot_pad(cx, cy, w, h):
    p = Pad(component_ref='J1', pad_number='1', global_x=cx, global_y=cy,
            local_x=0.0, local_y=0.0, size_x=max(w, h) + 0.3, size_y=min(w, h) + 0.3,
            shape='oval', layers=['*.Cu'], net_id=5, net_name='/SLOT', drill=min(w, h))
    p.drill_w = w
    p.drill_h = h
    return p


def _keepout_dist(pad, x, y):
    """Edge distance from (x,y) to the drill hole under the CAPSULE model."""
    (p1x, p1y), (p2x, p2y), r = pad_drill_capsule(pad)
    return point_to_segment_distance(x, y, p1x, p1y, p2x, p2y) - r


def run():
    fails = []

    def check(name, cond, detail=""):
        if not cond:
            fails.append(name)
        print(("  PASS " if cond else "  FAIL ") + name + (f"  {detail}" if detail else ""))

    h2h = 0.2
    via_r = 0.15  # via_drill / 2

    # 2.0 x 0.5 mm slot along x, centred at (10, 10): axis endpoints (9.25,10)-(10.75,10), r=0.25
    slot = _slot_pad(10.0, 10.0, 2.0, 0.5)
    (p1x, p1y), (p2x, p2y), r = pad_drill_capsule(slot)
    check("slot capsule axis spans the slot", abs(p2x - p1x - 1.5) < 1e-6 and abs(r - 0.25) < 1e-6,
          f"axis {p1x:.2f}->{p2x:.2f} r={r:.2f}")

    # A via just past the slot END: 0.05mm beyond the hole edge along the axis.
    end_x = 10.75 + 0.05  # capsule end x=10.75, +0.05 beyond the hole edge
    cap_edge = _keepout_dist(slot, end_x, 10.0)   # capsule: ~0.05 from the hole
    circle_edge = math.hypot(end_x - 10.0, 0.0) - slot.drill / 2.0  # OLD circle-at-centre model
    required = h2h + via_r
    check("capsule: via at slot END is inside the keep-out (blocked)", cap_edge < required,
          f"cap_edge={cap_edge:.3f} required={required:.3f}")
    check("old circle model WRONGLY allowed it (regression guard)", circle_edge > required,
          f"circle_edge={circle_edge:.3f} required={required:.3f}")

    # A via well clear of the whole slot is allowed under the capsule model.
    far = _keepout_dist(slot, 10.0, 11.0)   # 1mm above the axis, minus r=0.25 -> 0.75
    check("capsule: via far from the slot is allowed", far > required, f"far_edge={far:.3f}")

    # Round drill (drill_w == drill_h): capsule degenerates to the circle -> unchanged.
    round_pad = _slot_pad(10.0, 10.0, 0.6, 0.6)
    (rp1x, rp1y), (rp2x, rp2y), rr = pad_drill_capsule(round_pad)
    check("round drill is a degenerate capsule (== circle)",
          abs(rp1x - rp2x) < 1e-9 and abs(rp1y - rp2y) < 1e-9 and abs(rr - 0.3) < 1e-6)
    rd = _keepout_dist(round_pad, 10.0 + 0.3 + 0.05, 10.0)  # 0.05 past the round hole edge
    check("round drill: edge distance == circle model", abs(rd - 0.05) < 1e-6, f"rd={rd:.3f}")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): {fails}")
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(run())

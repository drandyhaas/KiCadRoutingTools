#!/usr/bin/env python3
"""The #189 via-in-pad unblock must see IN-FLIGHT copper (issue #310).

Phase-3 tap rip-up retries stamp their tap copper into the working obstacle
map while ripped victims re-route, committing it to pcb_data only afterwards.
The via-in-pad unblock builds its via-placement map from pcb_data, so before
the fix it drilled a fab-floor via straight through a pending foreign track
(snapdragon845 ETH_ISOLATEB 0.3mm via on PCIE1_WAKE_N In2.Cu, centerline
0.035mm from the via centre; same mechanism on kria_k26 USB_3V3/USB_1V2).

The fix registers the pending copper on pcb_data (push/pop_inflight_copper)
and _place_shrunk_via_in_pad merges it into the tap's obstacle window via
extra_vias/extra_segments.

    python3 tests/test_inflight_copper_unblock.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kicad_parser import Pad, PCBData, BoardInfo, Segment, Via
from routing_config import GridRouteConfig, GridCoord
from plane_pad_tap import (push_inflight_copper, pop_inflight_copper,
                           inflight_copper_dicts)
from single_ended_routing import _place_shrunk_via_in_pad

LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'B.Cu']


def _make_pcb():
    """A lone SMD pad on B.Cu, snapdragon-style 6-layer board."""
    pad = Pad(pad_number='20', net_id=5, net_name='ETH', global_x=10.0,
              global_y=10.0, size_x=0.71, size_y=0.22, shape='roundrect',
              layers=['B.Cu'], drill=0, component_ref='U9',
              local_x=0.0, local_y=0.0)
    board_info = BoardInfo(layers={}, copper_layers=list(LAYERS),
                           board_bounds=(0.0, 0.0, 20.0, 20.0))
    pcb = PCBData(board_info=board_info, nets={}, footprints={}, vias=[],
                  segments=[], pads_by_net={5: [pad]})
    return pcb, pad


def _config():
    return GridRouteConfig(grid_step=0.05, clearance=0.09, track_width=0.09,
                           via_size=0.6, via_drill=0.2, layers=list(LAYERS))


def _pt_seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def run():
    fails = []

    def check(name, cond):
        print(('  PASS  ' if cond else '  FAIL  ') + name)
        if not cond:
            fails.append(name)

    # Registry mechanics: nested push/pop, dict conversion.
    pcb, _ = _make_pcb()
    seg = Segment(start_x=8.6, start_y=8.6, end_x=11.4, end_y=11.4,
                  width=0.09, layer='In2.Cu', net_id=7)
    via = Via(x=12.0, y=12.0, size=0.3, drill=0.15,
              layers=['F.Cu', 'B.Cu'], net_id=7)
    t1 = push_inflight_copper(pcb, [seg], [via])
    t2 = push_inflight_copper(pcb, [seg], [])
    ev, es = inflight_copper_dicts(pcb)
    check("registry: nested windows accumulate", len(ev) == 1 and len(es) == 2)
    pop_inflight_copper(pcb, t2)
    ev, es = inflight_copper_dicts(pcb)
    check("registry: pop removes one window", len(ev) == 1 and len(es) == 1)
    check("registry: segment dict format",
          es[0]['start'] == (8.6, 8.6) and es[0]['layer'] == 'In2.Cu'
          and es[0]['net_id'] == 7)
    pop_inflight_copper(pcb, t1)
    check("registry: empty after final pop",
          inflight_copper_dicts(pcb) == (None, None))

    config = _config()
    coord = GridCoord(config.grid_step)

    # Baseline: with no obstacle near the pad, the unblock places a via in it.
    pcb, pad = _make_pcb()
    r = _place_shrunk_via_in_pad(pad, None, config, pcb, 5, coord, LAYERS)
    check("baseline: via-in-pad placed on an open board", r is not None)

    # A COMMITTED foreign In2 track through the pad centre blocks the via
    # (this is the pre-existing behavior the in-flight path must match).
    pcb, pad = _make_pcb()
    pcb.segments.append(seg)
    r = _place_shrunk_via_in_pad(pad, None, config, pcb, 5, coord, LAYERS)
    committed_ok = True
    if r is not None:
        v = r[0]
        d = _pt_seg_dist(v.x, v.y, seg.start_x, seg.start_y, seg.end_x, seg.end_y)
        committed_ok = d >= v.size / 2 + seg.width / 2  # at least no overlap
    check("committed foreign track blocks/deflects the via", committed_ok)

    # The snapdragon shape: the SAME track only IN-FLIGHT (stamped in the
    # working obstacle map, absent from pcb_data). Before the fix the via
    # landed essentially ON the track centerline.
    pcb, pad = _make_pcb()
    token = push_inflight_copper(pcb, [seg], [])
    r = _place_shrunk_via_in_pad(pad, None, config, pcb, 5, coord, LAYERS)
    inflight_ok = True
    if r is not None:
        v = r[0]
        d = _pt_seg_dist(v.x, v.y, seg.start_x, seg.start_y, seg.end_x, seg.end_y)
        inflight_ok = d >= v.size / 2 + seg.width / 2
        print(f"    (via at {v.x:.3f},{v.y:.3f} size {v.size}, "
              f"dist to in-flight track {d:.3f}mm)")
    check("in-flight foreign track blocks/deflects the via", inflight_ok)

    # Failure under a TRANSIENT window must not be memoised: after the window
    # closes, the same pad must be tappable again.
    pop_inflight_copper(pcb, token)
    r2 = _place_shrunk_via_in_pad(pad, None, config, pcb, 5, coord, LAYERS)
    check("no failure memoisation from a transient in-flight window",
          r2 is not None)

    # In-flight VIAS block too (hole-to-hole / via-via with the new via).
    pcb, pad = _make_pcb()
    inflight_via = Via(x=10.0, y=10.0, size=0.3, drill=0.15,
                       layers=['F.Cu', 'B.Cu'], net_id=7)
    push_inflight_copper(pcb, [], [inflight_via])
    r = _place_shrunk_via_in_pad(pad, None, config, pcb, 5, coord, LAYERS)
    via_ok = True
    if r is not None:
        v = r[0]
        d = math.hypot(v.x - inflight_via.x, v.y - inflight_via.y)
        via_ok = d >= v.size / 2 + inflight_via.size / 2
    check("in-flight foreign via blocks/deflects the via", via_ok)

    print()
    if fails:
        print(f"FAILED: {len(fails)} check(s): {fails}")
        return 1
    print("All checks passed.")
    return 0


if __name__ == '__main__':
    sys.exit(run())

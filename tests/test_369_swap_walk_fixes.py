#!/usr/bin/env python3
"""
Tests for issue #369 A10, A11, A13, A15.

  A10: find_connected_segment_positions honors its tolerance (soft joints no
       longer truncate relabel walks) and only crosses layers through a
       same-net via (co-located other-layer copper is not grabbed).
  A11: a blind/micro via at the pad no longer counts as a pad via for layer
       switching (has_pad_via), so switches drill the through via they need.
  A13: the visualization base obstacle map no longer hard-stamps BGA zones
       into blocked_cells (--visualize routed a different board).
  A15: swap validators' foreign-pad prefilters window by pad EDGE, so a large
       exposed pad with a far-away center is still distance-checked.

Run:
    python3 tests/test_369_swap_walk_fixes.py
"""

import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import PCBData, Segment, Via, Pad, Net
from routing_config import GridRouteConfig
from connectivity import find_connected_segment_positions
from stub_layer_switching import (get_stub_info, needs_pad_via_for_switch,
                                  via_barrel_clear_of_foreign_copper,
                                  stub_clear_of_foreign_pads)


def _seg(x1, y1, x2, y2, net_id=1, layer='F.Cu'):
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2,
                   width=0.2, layer=layer, net_id=net_id)


def _pcb(segments, vias=None, pads=None):
    return PCBData(board_info=None, nets={1: Net(1, 'P'), 2: Net(2, 'N')},
                   footprints={}, vias=vias or [], segments=segments,
                   pads_by_net=pads or {})


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- A10a: soft joint no longer truncates the walk -----------------------
    print("A10: tolerance honored across a soft joint")
    # chain: (0,0)->(5,0) then a 6um soft joint, then (5.006,0)->(10,0)
    segs = [_seg(0, 0, 5, 0), _seg(5.006, 0, 10, 0)]
    pos = find_connected_segment_positions(_pcb(segs), 0, 0, 1)
    check("walk crosses the 6um soft joint (was truncated)",
          any(abs(k[0] - 10.0) < 1e-6 for k in pos))

    # -- A10b: no cross-layer grab without a via ------------------------------
    print("\nA10: cross-layer only through a same-net via")
    segs = [_seg(0, 0, 5, 0, layer='F.Cu'),
            _seg(5, 0, 5, 8, layer='In1.Cu'),   # co-located at (5,0), NO via
            _seg(5, 8, 9, 8, layer='In1.Cu')]
    pos = find_connected_segment_positions(_pcb(segs), 0, 0, 1)
    check("stacked other-layer chain NOT pulled in without a via",
          not any(abs(k[1] - 8.0) < 1e-6 for k in pos))
    via = Via(x=5, y=0, size=0.5, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=1)
    pos = find_connected_segment_positions(_pcb(segs, vias=[via]), 0, 0, 1)
    check("same chain IS reached through a same-net via",
          any(abs(k[1] - 8.0) < 1e-6 for k in pos))

    # -- A11: blind via does not count as a pad via ---------------------------
    print("\nA11: blind via at the pad is not an all-layer pad via")
    pad = Pad(component_ref='U1', pad_number='1', global_x=0, global_y=0,
              local_x=0, local_y=0, size_x=0.5, size_y=0.5, shape='circle',
              layers=['F.Cu'], net_id=1, net_name='P')
    segs = [_seg(0, 0, 2, 0, layer='F.Cu')]
    blind = Via(x=0, y=0, size=0.4, drill=0.2, layers=['F.Cu', 'In1.Cu'], net_id=1)
    pcb = _pcb(segs, vias=[blind], pads={1: [pad]})
    stub = get_stub_info(pcb, 1, 2.0, 0.0, 'F.Cu')
    check("blind via not credited as pad via",
          stub is not None and not stub.has_pad_via)
    check("switch from F.Cu therefore drills a real through via",
          stub is not None and needs_pad_via_for_switch(stub))
    through = Via(x=0, y=0, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=1)
    pcb = _pcb(segs, vias=[through], pads={1: [pad]})
    stub = get_stub_info(pcb, 1, 2.0, 0.0, 'F.Cu')
    check("through via still credited", stub is not None and stub.has_pad_via)

    # -- A13: vis base map matches the non-vis twin on BGA zones -------------
    print("\nA13: --visualize obstacle map parity on BGA zones")
    from obstacle_map import build_base_obstacle_map, build_base_obstacle_map_with_vis
    from kicad_parser import BoardInfo
    cfg = GridRouteConfig()
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.grid_step = 0.1
    cfg.bga_exclusion_zones = [(1.0, 1.0, 3.0, 3.0)]
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=cfg.layers,
                   board_bounds=(0.0, 0.0, 5.0, 5.0))
    pcb = PCBData(board_info=bi, nets={}, footprints={}, vias=[], segments=[],
                  pads_by_net={})
    plain = build_base_obstacle_map(pcb, cfg, [])
    vis, _visdata = build_base_obstacle_map_with_vis(pcb, cfg, [])
    # An allowed-cells window inside the zone must unblock cells on BOTH maps
    # (the vis map's hard stamp used to take precedence and keep them blocked).
    for m in (plain, vis):
        m.add_allowed_cell(20, 20)
    check("allowed-cells window works on the plain map",
          not plain.is_blocked(20, 20, 0))
    check("allowed-cells window works on the VIS map (was dead)",
          not vis.is_blocked(20, 20, 0))

    # -- A15: pad-edge windows in the swap validators -------------------------
    print("\nA15: swap validators see large pads with far centers")
    # 7mm exposed pad centered 2.6mm from the via spot: center outside the old
    # 2.0mm window, copper edge only 0.1mm South of the via -> must reject.
    ep = Pad(component_ref='U9', pad_number='EP', global_x=2.6, global_y=0,
             local_x=0, local_y=0, size_x=5.0, size_y=5.0, shape='rect',
             layers=['B.Cu'], net_id=2, net_name='N')
    pcb = _pcb([], pads={2: [ep]})
    cfg = GridRouteConfig()
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.clearance = 0.15
    cfg.via_size = 0.5
    cfg.track_width = 0.2
    clear, reason = via_barrel_clear_of_foreign_copper(0.0, 0.0, 1, pcb, cfg, set())
    check("pad via vs far-centered EP rejected (was invisible)", not clear)

    moved = [_seg(0, 0, 0.5, 0, layer='B.Cu')]
    clear, reason = stub_clear_of_foreign_pads(moved, 'B.Cu', 1, pcb, cfg, set())
    check("moved stub vs far-centered EP rejected (was invisible)", not clear)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

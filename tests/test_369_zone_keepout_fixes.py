#!/usr/bin/env python3
"""
Tests for issue #369 A5 + A12 (zone/keepout reader fixes).

  A5: - v5/v6 single-line zones and keepouts parse (the newline-anchored
        block iterator yielded nothing for them);
      - unquoted layer tokens ((layer F.Cu), (layers F&B.Cu)) parse;
      - rule areas with `*.Cu` / `F&B.Cu` layer specs actually BLOCK tracks
        in the obstacle map (they silently disabled themselves before).
  A12: - plane_io.extract_zones sees multi-layer (layers ...) pours, so
         check_existing_zones refuses to pour a conflicting plane over them;
       - zone net_name is unescaped like every other net name.

Run:
    python3 tests/test_369_zone_keepout_fixes.py
"""

import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)

from kicad_parser import extract_zones, extract_keepouts


def main():
    fails = []

    def check(name, cond):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}")
        if not cond:
            fails.append(name)

    # -- A5: v5/v6 single-line zone + keepout, unquoted layers ---------------
    print("A5: v5/v6 single-line zones and unquoted layer tokens")
    v5_board = (
        '(kicad_pcb\n'
        '\t(zone (net 1) (net_name "GND") (layer F.Cu) (tstamp 0) (hatch edge 0.508)\n'
        '\t\t(connect_pads (clearance 0.508))\n'
        '\t\t(min_thickness 0.254)\n'
        '\t\t(fill yes (arc_segments 16) (thermal_gap 0.508) (thermal_bridge_width 0.508))\n'
        '\t\t(polygon (pts (xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)))\n'
        '\t)\n'
        '\t(zone (net 0) (net_name "") (layers F&B.Cu) (tstamp 1) (hatch edge 0.508)\n'
        '\t\t(keepout (tracks not_allowed) (vias not_allowed) (copperpour not_allowed))\n'
        '\t\t(polygon (pts (xy 20 0) (xy 30 0) (xy 30 10) (xy 20 10)))\n'
        '\t)\n'
        ')\n'
    )
    zones = extract_zones(v5_board)
    check("single-line v5 zone parsed (was zero)",
          len(zones) == 1 and zones[0].net_id == 1 and zones[0].layer == 'F.Cu')
    kos = extract_keepouts(v5_board)
    check("single-line v5 keepout parsed with unquoted F&B.Cu",
          len(kos) == 1 and kos[0]['layers'] == {'F&B.Cu'}
          and not kos[0]['tracks_allowed'])

    # -- A5: wildcard-layer rule areas block tracks in the obstacle map ------
    print("\nA5: '*.Cu' rule area blocks tracks (was silently disabled)")
    from kicad_parser import PCBData, Net, BoardInfo
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import GridObstacleMap, add_rule_area_keepout_obstacles

    cfg = GridRouteConfig()
    cfg.layers = ['F.Cu', 'B.Cu']
    cfg.grid_step = 0.1
    cfg.clearance = 0.1
    cfg.track_width = 0.2
    cfg.via_size = 0.5

    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                   copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(0.0, 0.0, 10.0, 10.0))
    for ko_layers in ({'*.Cu', '*.Mask'}, {'F&B.Cu'}):
        pcb = PCBData(board_info=bi, nets={}, footprints={}, vias=[],
                      segments=[], pads_by_net={})
        bi.keepouts = [{
            'polygon': [(2.0, 2.0), (8.0, 2.0), (8.0, 8.0), (2.0, 8.0)],
            'holes': [], 'layers': set(ko_layers),
            'tracks_allowed': False, 'vias_allowed': False}]
        pcb.board_info = bi
        obstacles = GridObstacleMap(len(cfg.layers))
        add_rule_area_keepout_obstacles(obstacles, pcb, cfg)
        coord = GridCoord(cfg.grid_step)
        gx, gy = coord.to_grid(5.0, 5.0)
        check(f"cell inside {sorted(ko_layers)} keepout blocked on F.Cu",
              obstacles.is_blocked(gx, gy, 0))
        check(f"cell inside {sorted(ko_layers)} keepout blocked on B.Cu",
              obstacles.is_blocked(gx, gy, 1))

    # -- A12: plane_io sees multi-layer pours; conflict detected -------------
    print("\nA12: plane_io multi-layer zone visibility")
    v10_board = '''(kicad_pcb
\t(version 20241229)
\t(net 0 "")
\t(net 1 "+3V3")
\t(zone
\t\t(net 1)
\t\t(net_name "+3V3")
\t\t(layers "In1.Cu" "In2.Cu")
\t\t(uuid "z1")
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)
\t\t\t)
\t\t)
\t)
)'''
    import plane_io
    with tempfile.NamedTemporaryFile('w', suffix='.kicad_pcb', delete=False) as f:
        f.write(v10_board)
    zinfos = plane_io.extract_zones(f.name)
    check("multi-layer pour visible on BOTH layers (was invisible)",
          {z.layer for z in zinfos} == {'In1.Cu', 'In2.Cu'}
          and all(z.net_name == '+3V3' for z in zinfos))
    should_create, should_continue, _ = plane_io.check_existing_zones(
        zinfos, 'In1.Cu', 'GND', 2)
    check("GND plane on In1.Cu refused (conflicting +3V3 pour there)",
          not should_create and not should_continue)

    # -- A12: zone net_name unescaped -----------------------------------------
    print("\nA12: zone net_name unescaped")
    esc_board = '''(kicad_pcb
\t(zone
\t\t(net 1)
\t\t(net_name "A\\\\B")
\t\t(layer "In1.Cu")
\t\t(uuid "z1")
\t\t(polygon
\t\t\t(pts
\t\t\t\t(xy 0 0) (xy 10 0) (xy 10 10) (xy 0 10)
\t\t\t)
\t\t)
\t)
)'''
    zones = extract_zones(esc_board)
    check("escaped zone net_name stored unescaped",
          len(zones) == 1 and zones[0].net_name == 'A\\B')

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())

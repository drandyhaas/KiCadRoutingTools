#!/usr/bin/env python3
"""#1487: a hybrid leg never lands on (or launches from) an island tap inside
another net's clearance halo.

The #444 chain seam re-ask routes each net's leg as one island-aware A*:
every grid cell of the net's existing copper island is a tap the leg may
land on (`_terminal_island_taps`). `_route_hybrid_leg` used to mark every tap
as a source/target cell -- exempt from clearance, like the one exact
terminal cell it exists for -- so a tap whose cell rounds into the partner's
halo carried the leg's last cell, and its weld back to the island, below
clearance (smartknob /STRAIN_S- landed on (80.70, 80.60), 0.07 mm from
/STRAIN_S+, and welded back to (80.679, 80.621)). A blocked tap is now
dropped; the island offers its free cells.

Synthetic board, F.Cu, grid 0.05, clearance 0.12, 0.1 mm tracks:
  partner (net 2): y = 10.0, x 8.5 .. 11.5
  own island (net 1): y = 10.224 from x 9.0 to 11.0 (0.124 mm clear of the
    partner -- legal), then up x = 11.0 to y = 11.0. Its lower run's taps
    round to y = 10.20, inside the partner's 0.22 mm halo; the upright's
    taps are free.
  leg: from (9.0, 10.6) to the island's top end, landing on any free tap.

    python3 tests/test_1487_hybrid_leg_taps_clear.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo, Net                            # noqa: E402
from routing_config import GridRouteConfig, GridCoord              # noqa: E402
from obstacle_map import build_base_obstacle_map                   # noqa: E402
from synth import make_pcb, make_seg                               # noqa: E402
import diff_pair_routing as dpr                                    # noqa: E402

RUN_ALL_TIMEOUT = 120
CLR = 0.12
W = 0.1

fails = []


def check(name, ok, detail=''):
    print(('PASS' if ok else 'FAIL') + f': {name}' + (f' -- {detail}' if detail else ''))
    if not ok:
        fails.append(name)


def edge_gap(s, t):
    """Min copper edge distance between two segments (exact)."""
    from diff_pair_routing import _seg_to_seglist_min_edge
    return _seg_to_seglist_min_edge(s.start_x, s.start_y, s.end_x, s.end_y,
                                    s.width, s.layer, [t])


partner = make_seg(8.5, 10.0, 11.5, 10.0, width=W, net_id=2)
island = [make_seg(9.0, 10.224, 11.0, 10.224, width=W, net_id=1),
          make_seg(11.0, 10.224, 11.0, 11.0, width=W, net_id=1)]
pcb = make_pcb(
    segments=[partner] + island,
    nets={1: Net(1, '/P'), 2: Net(2, '/N')},
    board_info=BoardInfo(layers={0: 'F.Cu'}, board_bounds=(5.0, 5.0, 15.0, 15.0),
                         copper_layers=['F.Cu']))
cfg = GridRouteConfig(layers=['F.Cu'], track_width=W, clearance=CLR, grid_step=0.05)
coord = GridCoord(cfg.grid_step)
layers = ['F.Cu']

print("1. precondition: the island is legal, and its lower taps sit in the halo")
check("the island clears the partner", edge_gap(island[0], partner) >= CLR - 1e-9,
      f"{edge_gap(island[0], partner):.3f}")
taps = dpr._terminal_island_taps((11.0, 11.0), island, [], coord, layers)
obs = build_base_obstacle_map(pcb, cfg, nets_to_route=[1, 2])
from obstacle_map import add_segments_list_as_obstacles, remove_segments_list_from_obstacles  # noqa: E402
add_segments_list_as_obstacles(obs, [partner], cfg)
in_halo = [t for t in taps if obs.is_blocked(int(t[0]), int(t[1]), int(t[2]))]
free = [t for t in taps if not obs.is_blocked(int(t[0]), int(t[1]), int(t[2]))]
remove_segments_list_from_obstacles(obs, [partner], cfg)
check("some island taps are in the partner's halo, some free", bool(in_halo) and bool(free),
      f"{len(in_halo)} in halo, {len(free)} free")

print("2. the leg lands on a free tap, and clears the partner")
segs, vias, _it, path = dpr._route_hybrid_leg(
    pcb, 1, cfg, obs, layers, coord, (11.0, 11.0, 0), (9.0, 10.6, 0),
    [partner], [], [], target_taps=taps)
check("the leg routes", segs is not None)
if segs:
    worst = min(edge_gap(s, partner) for s in segs)
    check("every leg segment clears the partner by the clearance", worst >= CLR - 1e-6,
          f"worst {worst:.4f} mm vs {CLR}; "
          + ', '.join(f'({s.start_x:.3f},{s.start_y:.3f})-({s.end_x:.3f},{s.end_y:.3f})'
                      for s in segs[-3:]))

if fails:
    print(f"\nFAILED: {fails}")
    sys.exit(1)
print("\nALL PASSED")

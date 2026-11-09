#!/usr/bin/env python3
"""A hybrid pair's leg vias stay out of the connector regions of pairs not yet
routed.

route_diff keeps vias out of each pair's connector region (the stretch from
its pads to its setback point, where its connectors fan out) on the
centreline map. A pair that falls back to the hybrid escape (#766) attaches
each terminal with a single-ended leg that drops its own escape via, and the
legs route on a map built from the other base, which carried none of those
regions. Measured with the #1736 pad stamps on two connector rows: a
hybrid's leg via landed in front of the next pair's pads and sealed its
launch, and that pair went single-ended.

Rows:
  - `add_connector_region_via_blocking` hands back exactly the cells it
    blocks (`cells_out`);
  - the leg map (`build_diff_pair_leg_obstacles`) blocks vias in the region
    of a pair that is not routed, and not in its own pair's region or in the
    region of a pair already routed.

    python3 tests/test_1736_hybrid_leg_launch_vias.py
"""
import inspect
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))

from kicad_parser import PCBData, BoardInfo, Net                     # noqa: E402
from routing_config import GridRouteConfig, GridCoord                # noqa: E402
from obstacle_map import (build_base_obstacle_map,                   # noqa: E402
                          add_connector_region_via_blocking)
from routing_context import build_diff_pair_leg_obstacles            # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def main():
    cfg = GridRouteConfig(track_width=0.1, clearance=0.1, grid_step=0.05,
                          via_size=0.6, via_drill=0.3, diff_pair_gap=0.1,
                          layers=['F.Cu', 'B.Cu'])
    nets = {i: Net(net_id=i, name=f'N{i}') for i in range(1, 7)}
    pcb = PCBData(board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                       copper_layers=['F.Cu', 'B.Cu'],
                                       board_bounds=(0.0, 0.0, 20.0, 20.0)),
                  nets=nets, footprints={}, vias=[], segments=[],
                  pads_by_net={}, zones=[])
    coord = GridCoord(cfg.grid_step)
    base = build_base_obstacle_map(pcb, cfg, list(nets))

    # three pairs' connector regions, each a launch off a terminal at x=5
    # (read off a map of its own, so the cells are the region whatever the
    # recorder hands back)
    regions, reported = {}, {}
    has_out = 'cells_out' in inspect.signature(add_connector_region_via_blocking).parameters
    for (p, n), y in (((1, 2), 5.0), ((3, 4), 10.0), ((5, 6), 15.0)):
        one = base.clone_fresh()
        cells = []
        kw = {'cells_out': cells} if has_out else {}
        add_connector_region_via_blocking(one, 5.0, y, 1.0, 0.0, 0.4, 0.1, cfg, **kw)
        x0, y0 = coord.to_grid(3.0, y - 2.0)
        x1, y1 = coord.to_grid(9.0, y + 2.0)
        regions[(p, n)] = [(gx, gy) for gx in range(x0, x1 + 1) for gy in range(y0, y1 + 1)
                           if one.is_via_blocked(gx, gy)]
        reported[(p, n)] = cells
    probe = {k: coord.to_grid(6.0, y) for k, y in
             (((1, 2), 5.0), ((3, 4), 10.0), ((5, 6), 15.0))}
    check("every probe stands in its own pair's region",
          all(tuple(probe[k]) in set(regions[k]) for k in regions))
    check("cells_out hands back exactly the cells the region blocks",
          has_out and all(set(map(tuple, reported[k])) == set(regions[k]) for k in regions),
          f"{sum(len(c) for c in regions.values())} cells" if has_out else "no cells_out")
    pcb._pair_launch_via_cells = regions

    # pair 1/2's hybrid: pair 3/4 still unrouted, pair 5/6 routed
    legs = build_diff_pair_leg_obstacles(
        base, pcb, cfg, routed_net_ids=[5, 6], remaining_net_ids=[1, 2, 3, 4],
        all_unrouted_net_ids=[1, 2, 3, 4], p_net_id=1, n_net_id=2,
        gnd_net_id=None, track_proximity_cache={}, layer_map={'F.Cu': 0, 'B.Cu': 1})
    check("a leg via cannot land in an unrouted pair's connector region",
          legs.is_via_blocked(*probe[(3, 4)]))
    check("a leg via may land in its own pair's region",
          not legs.is_via_blocked(*probe[(1, 2)]))
    check("a leg via may land in a routed pair's region",
          not legs.is_via_blocked(*probe[(5, 6)]))
    check("the base map is left as it was",
          not base.is_via_blocked(*probe[(3, 4)]))

    if FAILS:
        print(f"\nFAIL: {len(FAILS)} row(s): {FAILS}")
        return 1
    print("\na hybrid's leg vias keep out of unrouted pairs' connector regions. OK")
    return 0


if __name__ == '__main__':
    sys.exit(main())

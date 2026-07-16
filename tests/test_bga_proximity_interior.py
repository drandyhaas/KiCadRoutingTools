#!/usr/bin/env python3
"""BGA proximity cost must cover the zone INTERIOR, not just the outside ring.

Historically add_bga_proximity_costs skipped courtyard-interior cells as
"already blocked" (true when set_bga_zone hard-blocked the whole rectangle).
Once allowed_cells windows could unblock cells INSIDE the zone (endpoint
windows, the #189 via-in-pad unblock), those windows carried ZERO proximity
cost while the ring just outside carried full cost - so the router preferred
dropping vias inside the escape window under the BGA, sealing escape capacity
for later nets (the "free-via-in-allowed-window hole").

This test asserts:
  1. interior cells carry the full edge-tier cost (dist=0 in the falloff),
  2. the exterior ring falloff is unchanged (edge -> 0 at radius),
  3. the cost is a preference, not a block: an allowed cell inside the zone
     stays via-placeable.

    python3 tests/test_bga_proximity_interior.py
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'rust_router'))

from routing_config import GridRouteConfig, GridCoord
from obstacle_costs import add_bga_proximity_costs
from grid_router import GridObstacleMap


def main():
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'])
    cfg.bga_exclusion_zones = [(10.0, 10.0, 20.0, 20.0)]

    coord = GridCoord(cfg.grid_step)
    obs = GridObstacleMap(2)

    # Mirror build_obstacle_map: zone rectangle + proximity costs
    gmin = coord.to_grid(10.0, 10.0)
    gmax = coord.to_grid(20.0, 20.0)
    obs.set_bga_zone(gmin[0], gmin[1], gmax[0], gmax[1])
    add_bga_proximity_costs(obs, cfg)

    full = cfg.cell_cost(cfg.bga_proximity_cost)
    radius = cfg.bga_proximity_radius
    failures = []

    def cost_at(x, y):
        gx, gy = coord.to_grid(x, y)
        return obs.get_stub_proximity_cost(gx, gy)

    # 1. Interior cells: full edge-tier cost everywhere inside the zone
    for name, x, y in [('center', 15.0, 15.0), ('near-corner', 10.2, 10.2),
                       ('near-edge', 19.8, 15.0)]:
        c = cost_at(x, y)
        if c != full:
            failures.append(f'interior {name}: cost {c} != full tier {full}')

    # 2. Ring falloff unchanged: monotonic decrease from edge to zero at radius
    c_edge = cost_at(10.0 - cfg.grid_step, 15.0)
    c_mid = cost_at(10.0 - radius / 2, 15.0)
    c_beyond = cost_at(10.0 - radius - 1.0, 15.0)
    if not (full >= c_edge > c_mid > 0):
        failures.append(f'ring falloff broken: edge={c_edge} mid={c_mid} full={full}')
    if c_beyond != 0:
        failures.append(f'cost beyond radius: {c_beyond} != 0')

    # 3. Preference, not a block: an allowed cell inside the zone stays
    # via-placeable despite carrying the interior cost
    agx, agy = coord.to_grid(15.0, 15.0)
    if not obs.is_via_blocked(agx, agy):
        failures.append('zone interior should be via-blocked without allowed cell')
    obs.add_allowed_cell(agx, agy)
    if obs.is_via_blocked(agx, agy):
        failures.append('allowed cell inside zone must stay via-placeable')
    if obs.get_stub_proximity_cost(agx, agy) != full:
        failures.append('allowed cell lost its interior proximity cost')

    if failures:
        for f in failures:
            print(f'FAIL: {f}')
        return 1
    print(f'PASS: interior stamped at full tier ({full}), ring falloff intact, '
          f'allowed cells stay placeable')
    return 0


if __name__ == '__main__':
    sys.exit(main())

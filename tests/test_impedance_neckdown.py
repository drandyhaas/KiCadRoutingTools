#!/usr/bin/env python3
"""Impedance neck-down gate (#156 follow-up, env knob tracked on #465).

An impedance-width net whose full-width route is blocked may neck down to the
NOMINAL track width (completion over strict impedance). Default ALLOW;
KICAD_IMPEDANCE_NECKDOWN=0 forbids it (the net fails instead). Power-net
neck-down is unaffected by the env var.

Geometry: a closed single-layer corridor (net-2 walls) whose gap fits the
0.1 mm nominal width but not the 0.3 mm impedance/power width, so the
full-width attempt always fails and the ladder is the only way through.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'rust_router'))

from grid_router import GridRouter, GridObstacleMap
from routing_config import GridRouteConfig
from obstacle_map import add_segments_list_as_obstacles
from single_ended_routing import _route_main_connection
from synth import make_pcb, make_net, make_seg

CHECKS = []


def check(name, ok):
    CHECKS.append((name, ok))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}")


def _corridor_setup(config):
    """Closed box of net-2 walls around a y=0 channel; gap fits only nominal."""
    walls = [
        make_seg(0.0, 0.3, 10.0, 0.3, net_id=2, width=0.2, layer='F.Cu'),
        make_seg(0.0, -0.3, 10.0, -0.3, net_id=2, width=0.2, layer='F.Cu'),
        make_seg(0.0, -0.3, 0.0, 0.3, net_id=2, width=0.2, layer='F.Cu'),
        make_seg(10.0, -0.3, 10.0, 0.3, net_id=2, width=0.2, layer='F.Cu'),
    ]
    obstacles = GridObstacleMap(len(config.layers))
    add_segments_list_as_obstacles(obstacles, walls, config)
    pcb = make_pcb(nets={1: make_net(1, 'IMP1'), 2: make_net(2, 'WALL')},
                   segments=walls)
    router = GridRouter(via_cost=50, h_weight=1.5, turn_cost=100)
    sources = [(20, 0, 0)]
    targets = [(80, 0, 0)]
    return router, obstacles, pcb, sources, targets


def _run(config, env_value):
    old = os.environ.get('KICAD_IMPEDANCE_NECKDOWN')
    try:
        if env_value is None:
            os.environ.pop('KICAD_IMPEDANCE_NECKDOWN', None)
        else:
            os.environ['KICAD_IMPEDANCE_NECKDOWN'] = env_value
        router, obstacles, pcb, sources, targets = _corridor_setup(config)
        margins = config.track_margins_for_net(1)
        res = _route_main_connection(router, obstacles, config, sources, targets,
                                     margins, pcb, 1)
        return res[0], res[-2], res[-1]  # path, necked_down, uniform_width
    finally:
        if old is None:
            os.environ.pop('KICAD_IMPEDANCE_NECKDOWN', None)
        else:
            os.environ['KICAD_IMPEDANCE_NECKDOWN'] = old


def _impedance_config():
    cfg = GridRouteConfig(track_width=0.1, clearance=0.1, grid_step=0.1,
                          layers=['F.Cu'])
    cfg.layer_widths = {'F.Cu': 0.3}
    cfg.impedance_target = 40.0
    return cfg


def _power_config():
    cfg = GridRouteConfig(track_width=0.1, clearance=0.1, grid_step=0.1,
                          layers=['F.Cu'])
    cfg.power_net_widths = {1: 0.3}
    return cfg


def main():
    print("=" * 60)
    print("impedance neck-down env gate (#156 / #465)")
    print("=" * 60)

    # Default (env unset): impedance net necks down and connects
    path, necked, uniform = _run(_impedance_config(), None)
    check("default allows impedance neck-down (routes)", path is not None)
    check("route is genuinely necked (uniform width < impedance width)",
          uniform is not None and uniform < 0.3 - 1e-9)

    # Explicit allow spelling
    path, _n, _u = _run(_impedance_config(), '1')
    check("env=1 allows too", path is not None)

    # Forbid: strict impedance -- the net fails instead of narrowing
    path, necked, uniform = _run(_impedance_config(), '0')
    check("env=0 forbids impedance neck-down (fails)", path is None)
    check("env=0 leaves no rewidth markers", necked is False and uniform is None)

    # Power neck-down is NOT gated by the env var
    path, _n, uniform = _run(_power_config(), '0')
    check("power neck-down unaffected by env=0", path is not None
          and uniform is not None and uniform < 0.3 - 1e-9)

    failed = [n for n, okk in CHECKS if not okk]
    print("-" * 60)
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
        return 1
    print("ALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

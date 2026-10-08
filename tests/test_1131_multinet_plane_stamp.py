#!/usr/bin/env python3
"""#1131: the multi-net plane split keeps a foreign track at its PAIR
clearance, in millimetres.

When several nets share a plane layer, `route_planes._generate_multinet_layer_
zones` routes each net's region-connection paths (the Voronoi seeds that split
the layer between its nets) on `route_planes.build_plane_base_obstacles`. That
map stamps every foreign track on the plane layer through
`plane_obstacle_builder._add_segment_routing_obstacle`, which takes
MILLIMETRES (an exact capsule since #173) -- and was handed a GRID-CELL count,
so a 0.2 mm track with a 0.2 mm clearance kept the paths 8 mm away at a
0.05 mm grid and 4 mm at 0.1 mm instead of 0.4 mm. It also priced the track at
the flat `config.clearance`, ignoring the class map `create_plane` installs
and the .kicad_dru layer rule.

Each case measures how far the keep-out reaches from the track's centreline
(walking cell by cell to the first free one) and holds it to
`track_width/2 + seg_width/2 + pair`, where `pair` is check_drc's value for the
two nets on the plane layer: max(clearance, both classes), then the layer rule,
which REPLACES it. Both grid steps, the flat case, a class on either net, and a
layer rule (tightening, relaxing a class, and on another layer). The previous-
route stamp (the other plane nets' paths, which takes GRID cells) is held to
the same pair value. Last, the consequence: a region connection 1 mm from a
foreign track has to route.

    python3 tests/test_1131_multinet_plane_stamp.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'rust_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from kicad_parser import BoardInfo, Net                 # noqa: E402
from routing_config import GridRouteConfig, GridCoord   # noqa: E402
from synth import make_pcb, make_seg                    # noqa: E402
import route_planes                                     # noqa: E402

RUN_ALL_TIMEOUT = 120

LAYER = 'In1.Cu'
PLANE_NET, FOREIGN_NET = 1, 2
CLEARANCE = 0.2
TRACK_W = 0.2       # the plane config's track width
SEG_W = 0.2         # the foreign track's width
STEPS = (0.05, 0.1)
X_PROBE = 20.0      # mid-track, far from both ends


def _config(step, classes=None, layer_rules=None):
    cfg = GridRouteConfig(clearance=CLEARANCE, track_width=TRACK_W,
                          grid_step=step, layers=[LAYER])
    if classes:
        cfg.net_clearances = dict(classes)   # what create_plane installs
    if layer_rules:
        cfg.layer_clearances = dict(layer_rules)
    return cfg


def _pcb(segments=()):
    # The bounds are 20 mm from the track, so the board-edge band the map
    # also stamps never meets the walk.
    return make_pcb(
        segments=list(segments),
        nets={PLANE_NET: Net(PLANE_NET, 'GND'),
              FOREIGN_NET: Net(FOREIGN_NET, '/HV')},
        board_info=BoardInfo(layers={}, board_bounds=(-5, -20, 45, 20),
                             copper_layers=[LAYER]))


def _track(y=0.0):
    return make_seg(0, y, 40, y, net_id=FOREIGN_NET, width=SEG_W, layer=LAYER)


def _reach(obs, step, y0=0.0):
    """mm from the line y=y0 to the first free cell, walking +y at X_PROBE."""
    coord = GridCoord(step)
    gx, gy0 = coord.to_grid(X_PROBE, y0)
    n = 0
    while obs.is_blocked(gx, gy0 + n, 0) and n < 100000:
        n += 1
    return n * step


def main():
    fails = []

    def check(name, cond, detail=''):
        print(('PASS' if cond else 'FAIL') + f': {name}'
              + (f' -- {detail}' if detail else ''))
        if not cond:
            fails.append(name)

    # ---- 1. the foreign-track stamp: mm, at the pair value -----------------
    # (label, classes, layer rules, the pair clearance check_drc grades)
    cases = [
        ('flat', None, None, CLEARANCE),
        ('class 0.6 on the foreign net', {FOREIGN_NET: 0.6}, None, 0.6),
        ('class 0.5 on the plane net', {PLANE_NET: 0.5}, None, 0.5),
        ('layer rule 0.3 on the plane layer', None, {LAYER: 0.3}, 0.3),
        ('layer rule 0.3 replaces a 0.6 class', {FOREIGN_NET: 0.6},
         {LAYER: 0.3}, 0.3),
        ('layer rule on another layer is inert', None, {'In2.Cu': 0.5},
         CLEARANCE),
    ]
    for step in STEPS:
        for label, classes, rules, pair in cases:
            name = f'track stamp, grid {step}, {label}'
            try:
                cfg = _config(step, classes, rules)
                obs = route_planes.build_plane_base_obstacles(
                    LAYER, PLANE_NET, {}, cfg, _pcb([_track()]))
                reach = _reach(obs, step)
                want = TRACK_W / 2 + SEG_W / 2 + pair
                # The capsule is exact and a tie reads OPEN, so the first free
                # cell is the first one at or beyond the keep-out: [want,
                # want + step). One-sided, so a 0.1 mm error at a 0.1 mm grid
                # is still caught.
                check(name, want - 1e-6 <= reach < want + step - 1e-6,
                      f'reaches {reach:.2f} mm, expected {want:.2f}')
            except Exception as e:     # a crash is a failure, with its reason
                check(name, False, f'{type(e).__name__}: {e}')

    # ---- 2. the previous-route stamp: grid cells, at the pair value --------
    # A path of the foreign net along y=5; its disc template blocks every cell
    # whose centre is within ceil(want / step) cells, so the first free cell
    # lies (0, step] beyond `want`.
    for step in STEPS:
        for label, classes, pair in (('flat', None, CLEARANCE),
                                     ('class 0.6 on the route net',
                                      {FOREIGN_NET: 0.6}, 0.6)):
            name = f'previous-route stamp, grid {step}, {label}'
            try:
                cfg = _config(step, classes)
                obs = route_planes.build_plane_base_obstacles(
                    LAYER, PLANE_NET, {}, cfg, _pcb(),
                    previous_routes=[(FOREIGN_NET, [(0.0, 5.0), (40.0, 5.0)])])
                reach = _reach(obs, step, y0=5.0)
                want = TRACK_W + pair
                check(name, want + 1e-6 < reach <= want + step + 1e-6,
                      f'reaches {reach:.2f} mm, expected ({want:.2f}, '
                      f'{want + step:.2f}]')
            except Exception as e:
                check(name, False, f'{type(e).__name__}: {e}')

    # ---- 3. the consequence: a region connection that fits is routed -------
    # Two plane vias 1 mm off a foreign track, 20 mm apart. The keep-out is
    # 0.4 mm, so the straight path is open; a keep-out of whole millimetres
    # walls both vias in.
    for step in STEPS:
        name = f'region connection 1 mm from a foreign track routes, grid {step}'
        try:
            cfg = _config(step)
            base = route_planes.build_plane_base_obstacles(
                LAYER, PLANE_NET, {}, cfg, _pcb([_track()]))
            router = route_planes.GridRouter(
                via_cost=cfg.via_cost_units(), h_weight=cfg.heuristic_weight,
                turn_cost=cfg.turn_cost, via_proximity_cost=0,
                layer_costs=cfg.get_layer_costs(),
                proximity_heuristic_cost=cfg.get_proximity_heuristic_cost())
            path = route_planes.route_plane_connection(
                (10.0, 1.0), (30.0, 1.0), base, router, cfg,
                max_iterations=200000)
            ok = bool(path)
            low = min(p[1] for p in path) if path else None
            check(name, ok and low >= TRACK_W / 2 + SEG_W / 2 + CLEARANCE - 1e-9,
                  'no path' if not ok else f'path stays >= {low:.2f} mm '
                                           f'from the track')
        except Exception as e:
            check(name, False, f'{type(e).__name__}: {e}')

    if fails:
        print(f'\nFAILED {len(fails)} check(s)')
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

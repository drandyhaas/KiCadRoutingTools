#!/usr/bin/env python3
"""A forced oracle link never ships copper inside foreign clearance.

A forced link (#572) routes between KiCad's nearest-approach anchors of two
fill islands. Those anchors stand where the foreign copper that cut the
islands apart is closest, and their grid cells are obstacle-exempt, so when
the two cells are neighbours the router lays the step between them whatever
stands beside it -- and a terminal neck already at the floor width leaves the
graze for the DRC report. `route_oracle_links` now measures the laid copper
(`terminal_graze`) and, on a graze:

  1. SHIFT -- relaunches the link from free cells of each anchor's own
     island (`island_launch_rows`), so the copper it lays keeps its
     clearance; or
  2. DECLINE -- with no clear route from there the link fails, as a link
     that cannot route does, and names the link it failed on.

The board: two foreign vias 0.25 mm apart (edge to edge) are the gap the
islands meet across; the anchors sit either side of it, one grid step apart,
50 um inside one via's clearance. A band of blocked cells around them stands
for the dense foreign copper of a ball field: every cell the oracle's own
end-stepping walks is taken, so the anchors keep their exempt cells. When the
band ends within reach, the link is relaunched around it; when it spans the
board, the link is declined.

    python3 tests/test_oracle_link_graze_declined.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, TESTS_DIR, os.path.join(ROOT, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)

from synth import RoutingBoard                                 # noqa: E402
from run_utils import evidence                                 # noqa: E402

FAILS = []

CLEARANCE = 0.1
TRACK = 0.1
WALL_X = 10.0
VIA_R = 0.2
VIAS = [(WALL_X, 10.0), (WALL_X, 10.65)]
#: The two anchors: either side of the gap, in neighbouring grid cells
#: (10.0, 10.3) and (10.1, 10.3); the first is 0.3 mm from the via at
#: (10, 10), so a track there sits 50 um inside its clearance.
LINK = ((9.96, 10.33, 'F.Cu', 'zone'), (10.06, 10.33, 'F.Cu', 'zone'))
#: The blocked band's x extent: wider than the oracle end-stepping's 0.5 mm
#: walk on either side, narrower than the 1 mm an island launch looks.
BAND_X = (9.4, 10.7)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name
          + (f'  {detail}' if detail else ''))


class Fill:
    """GND's F.Cu fill: island 1 left of the wall, island 2 right of it,
    carved back from every wall via by the clearance."""
    def __init__(self, vias):
        self.vias = vias

    def query_component(self, x, y, size=0.0):
        if not (1.0 < x < 19.0 and 1.0 < y < 19.0):
            return 0
        if any(math.hypot(x - vx, y - vy) < VIA_R + CLEARANCE
               for vx, vy in self.vias):
            return 0
        return 1 if x < WALL_X else 2


def route(td, gap):
    import numpy as np
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig, GridCoord
    from obstacle_map import build_base_obstacle_map
    from obstacle_cache import (precompute_all_net_obstacles,
                                add_net_obstacles_from_cache)
    import plane_fill_model
    import single_ended_routing as ser
    vias = VIAS
    b = RoutingBoard(w=20, h=20)
    b.pad('J1', 3, 3, 'GND')
    b.pad('J2', 17, 17, 'GND')
    b.pad('J3', 3, 17, 'X')
    for vx, vy in vias:
        b.via(vx, vy, 'X', size=2 * VIA_R, drill=0.2)
    path = os.path.join(td, f'gap_{int(gap)}.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(b.text())
    evidence(path)
    pcb = parse_kicad_pcb(path)
    ids = {n.name: i for i, n in pcb.nets.items()}
    gnd, x = ids['GND'], ids['X']
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TRACK,
                          clearance=CLEARANCE, grid_step=0.1, via_size=0.4,
                          via_drill=0.2)
    real = plane_fill_model.get_fill_models
    plane_fill_model.get_fill_models = \
        lambda pcb_, nid: {'F.Cu': [Fill(vias)]} if nid == gnd else {}
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            base = build_base_obstacle_map(pcb, cfg, [gnd, x],
                                           static_base=True)
            cache = precompute_all_net_obstacles(pcb, [gnd, x], cfg)
        work = base.clone_fresh()
        add_net_obstacles_from_cache(work, cache[x])
        coord = GridCoord(cfg.grid_step)
        (gx0, gy0), (gx1, gy1) = (coord.to_grid(BAND_X[0], 8.0 if gap else 0.0),
                                  coord.to_grid(BAND_X[1], 12.5 if gap else 20.0))
        work.add_blocked_cells_batch(np.array(
            [(gx, gy, li) for gx in range(gx0, gx1 + 1)
             for gy in range(gy0, gy1 + 1) for li in (0, 1)], dtype=np.int32))
        log = io.StringIO()
        with contextlib.redirect_stdout(log):
            res = ser.route_oracle_links(pcb, gnd, cfg, work, [LINK])
    finally:
        plane_fill_model.get_fill_models = real
    return res, vias, log.getvalue()


def worst_gap(res, vias):
    """Smallest edge gap (mm) from the result's copper to a wall via."""
    best = math.inf
    for s in res.get('new_segments') or ():
        for vx, vy in vias:
            dx, dy = s.end_x - s.start_x, s.end_y - s.start_y
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, (
                (vx - s.start_x) * dx + (vy - s.start_y) * dy) / L2))
            d = math.hypot(s.start_x + t * dx - vx, s.start_y + t * dy - vy)
            best = min(best, d - VIA_R - s.width / 2.0)
    for v in res.get('new_vias') or ():
        for vx, vy in vias:
            best = min(best, math.hypot(v.x - vx, v.y - vy) - VIA_R - v.size / 2)
    return best


def main():
    with tempfile.TemporaryDirectory() as td:
        print('1. the band ends within reach: the link is relaunched clear')
        res, vias, log = route(td, gap=True)
        ok = bool(res) and not res.get('failed')
        check('the link is laid', ok, log.strip().splitlines()[-1:] if log else '')
        if ok:
            g = worst_gap(res, vias)
            check('its copper keeps the clearance from every foreign via',
                  g >= CLEARANCE - 1e-6, f'worst gap {g:.4f} mm')
            check('the relaunch is disclosed', 'relaunched clear' in log)

        print('2. the band spans the board: the link is declined, not laid '
              'grazing')
        res, vias, log = route(td, gap=False)
        laid = bool(res) and not res.get('failed')
        check('the link is not laid', not laid,
              f'worst gap {worst_gap(res, vias):.4f} mm' if laid else '')
        check('the decline is disclosed', 'forced link DECLINED' in log,
              log.strip().splitlines()[-1:] if log else '')
        check('the failure names the link it is about',
              bool(res) and res.get('failed_link') == (LINK[0][:2], LINK[1][:2]),
              str((res or {}).get('failed_link')))
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

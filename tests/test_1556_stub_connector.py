#!/usr/bin/env python3
"""A route from a fanout stub lays no second grid-snap connector (#1556).

A fanout stub ends in a short grid-snap connector: from the stub's off-grid
tip P to the grid point G of P's own cell. A route launching from the stub
welds to the ORIGINAL point of the endpoint row on the cell it starts on, by
a connector from the grid point. The connector's two ends share one cell; the
endpoint rows kept its start only (P), and the router took the first row on
the cell (P, the end of the segment before the connector). So the route
re-laid P->G at its own width on top of the stub's own P->G -- two same-net
segments over one 23 um span, which a later strip could take both of
(test_1556_stack_strip.py). The rows now offer the end ON the grid point, the
route welds to it (no connector), and a tap cell is owned by it.

Checks:
  A. route_net_with_obstacles from a stub with a grid-snap connector lays no
     segment touching P, with the connector drawn either way round
  B. get_all_segment_tap_points gives G's cell the owner G, not P (an
     off-grid cell keeps its first sample, #1291)

    python3 tests/test_1556_stub_connector.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from synth import RoutingBoard                          # noqa: E402
from kicad_parser import Segment, parse_kicad_pcb       # noqa: E402
from routing_config import GridCoord, GridRouteConfig   # noqa: E402

FAILS = []
P = (10.0792, 5.0092)       # the stub's off-grid tip
G = (10.1, 5.0)             # the grid point of P's cell (0.1 mm grid)


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def near(a, b, tol=1e-6):
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def board(path, connector_reversed=False, twin=False, third_pad=False):
    b = RoutingBoard(w=30, h=10)
    b.pad('J1', 5, 5, 'N')
    b.pad('J2', 20, 5, 'N')
    if third_pad:
        b.pad('J3', 25, 8, 'N')
    b.seg(5, 5, P[0], P[1], 'N', w=0.1)
    a, z = (G, P) if connector_reversed else (P, G)
    b.seg(a[0], a[1], z[0], z[1], 'N', w=0.0762, uid='keep-connector')
    if twin:
        b.seg(P[0], P[1], G[0], G[1], 'N', w=0.1, uid='twin-connector')
        b.seg(G[0], G[1], 20, 5, 'N', w=0.1)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    return path


def check_route_from_stub(tmp):
    print('A. a route from the stub welds at G')
    from obstacle_map import build_base_obstacle_map
    from single_ended_routing import route_net_with_obstacles
    for rev in (False, True):
        pcb = parse_kicad_pcb(board(os.path.join(tmp, f'a{int(rev)}.kicad_pcb'),
                                    connector_reversed=rev))
        nid = next(i for i, n in pcb.nets.items() if n.name == 'N')
        cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.15,
                              clearance=0.1, grid_step=0.1)
        with contextlib.redirect_stdout(io.StringIO()):
            obs = build_base_obstacle_map(pcb, cfg, [nid])
            res = route_net_with_obstacles(pcb, nid, cfg, obs)
        segs = (res or {}).get('new_segments') or []
        at_p = [s for s in segs
                if near((s.start_x, s.start_y), P) or near((s.end_x, s.end_y), P)]
        at_g = [s for s in segs
                if near((s.start_x, s.start_y), G) or near((s.end_x, s.end_y), G)]
        tag = 'connector G->P' if rev else 'connector P->G'
        check(f'{tag}: the net routes', bool(segs))
        check(f'{tag}: no new copper at the stub tip P (no twin connector)',
              not at_p, f'{len(at_p)} segment(s) at P')
        check(f'{tag}: the route launches from G', bool(at_g))


def check_tap_owner():
    print('B. a cell whose copper reaches its grid point is owned by that point')
    from single_ended_routing import get_all_segment_tap_points
    coord = GridCoord(0.1)
    for rev in (False, True):
        conn = (Segment(G[0], G[1], P[0], P[1], 0.0762, 'F.Cu', 1) if rev
                else Segment(P[0], P[1], G[0], G[1], 0.0762, 'F.Cu', 1))
        segs = [Segment(5, 5, P[0], P[1], 0.1, 'F.Cu', 1), conn]
        pts = get_all_segment_tap_points(segs, coord, ['F.Cu', 'B.Cu'])
        cell = coord.to_grid(*G)
        own = [(x, y) for gx, gy, li, x, y in pts if (gx, gy, li) == (cell[0], cell[1], 0)]
        check(f"{'G->P' if rev else 'P->G'}: G's cell is owned by G",
              len(own) == 1 and near(own[0], G), f'{own}')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        check_route_from_stub(tmp)
        check_tap_owner()
    if FAILS:
        print(f'FAIL: {len(FAILS)} check(s): {FAILS}')
        sys.exit(1)
    print('PASS: #1556 stub connector')


if __name__ == '__main__':
    main()

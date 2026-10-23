#!/usr/bin/env python3
"""The channel escape's collision fixes act on the ROUTE that collides, not on its net.

  python3 tests/test_bga_collision_route_target.py

A multi-ball net (a power rail) has one FanoutRoute per ball, and two passes
treated it as one:

* `resolve_collisions` handled a colliding single-ended net through the
  net's FIRST route. That route usually collided with nothing; moving it
  "resolved" the net, and the colliding escapes shipped. daisho U1's single
  pass reported "After resolution: 20 collisions remaining": V1P2 K19, V2P5
  J21 and VCCD_PLL J20 stacked in one F.Cu channel and V1P2 V17 on /ddr2/VREF
  Y17, 20 copper contacts on the pre-route board, none of the five escapes
  ever tried.
* `repair_pad_crossings`, giving up on one route, removed every track of its
  NET (#508 finding 6's net-scoped removal): with the resolver fixed, one V1P2
  ball given up there took the escapes of 32 of the rail's 50 balls with it.

Pinned on synthetic routes, through the real functions:

1. resolve_collisions: net 1's first route collides with nothing, its second
   rides a channel another net's earlier copper already holds. The collision
   is resolved (by the second route), and the first is left where it was;
2. repair_pad_crossings: a net's route that crosses a foreign pad on every
   alternative is given up; that route's tracks go, the net's other route
   keeps its own, and the net is reported.
"""
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from synth import make_pad  # noqa: E402
from bga_fanout.types import BGAGrid, FanoutRoute, route_uid  # noqa: E402
from bga_fanout.grid import calculate_channels  # noqa: E402
from bga_fanout.geometry import create_45_stub, calculate_exit_point  # noqa: E402
from bga_fanout.tracks import generate_tracks_from_routes  # noqa: E402
from bga_fanout.collision import find_colliding_pairs  # noqa: E402
from bga_fanout import reroute  # noqa: E402

TW, CL = 0.2, 0.1
GRID = BGAGrid(pitch_x=1.0, pitch_y=1.0,
               rows=[-2.5, -1.5, -0.5, 0.5, 1.5, 2.5],
               cols=[-2.5, -1.5, -0.5, 0.5, 1.5, 2.5],
               center_x=0.0, center_y=0.0, min_x=-2.5, max_x=2.5, min_y=-2.5, max_y=2.5)
CHANNELS = calculate_channels(GRID)


def _route(net, x, y, direction, ch_pos, num):
    ch = next(c for c in CHANNELS if c.orientation == 'horizontal'
              and abs(c.position - ch_pos) < 1e-9)
    pad = make_pad(net, x, y, ref='U1', num=num, net_name=f'N{net}',
                   shape='circle', size_x=0.5, size_y=0.5)
    stub = create_45_stub(x, y, ch, direction)
    return FanoutRoute(pad=pad, pad_pos=(x, y), stub_end=stub,
                       exit_pos=calculate_exit_point(stub, ch, direction, GRID, 0.5),
                       channel=ch, escape_dir=direction, layer='F.Cu')


def _geom(r):
    return (r.stub_end, r.exit_pos, r.channel.position, r.layer)


def _quiet(fn):
    buf, old = io.StringIO(), sys.stdout
    sys.stdout = buf
    try:
        return fn()
    finally:
        sys.stdout = old


def check_resolver(fails):
    a1 = _route(1, -1.5, -1.5, 'left', -1.0, 'B2')    # the net's FIRST route: clear
    a2 = _route(1, 0.5, 1.5, 'right', 1.0, 'E4')      # rides channel y=1.0 ...
    routes = [a1, a2]
    tracks, _, _ = _quiet(lambda: generate_tracks_from_routes(routes, TW, 'F.Cu'))
    # ... which net 2's copper from an earlier step already holds
    existing = [{'start': (2.0, 1.0), 'end': (3.5, 1.0), 'width': TW, 'layer': 'F.Cu',
                 'net_id': 2, 'is_existing': True}]
    if not find_colliding_pairs(tracks + existing, TW + CL):
        fails.append("fixture: net 1's second route does not collide with net 2")
        return
    a1_before = _geom(a1)
    _, failed = _quiet(lambda: reroute.resolve_collisions(
        routes, tracks, ['F.Cu'], TW, CL, 0.2, existing, GRID, CHANNELS, 0.5,
        {1: 'N1', 2: 'N2'}))
    left = find_colliding_pairs(tracks + existing, TW + CL)
    if left:
        fails.append(f"resolve_collisions left {sorted(left)} colliding: it did not "
                     f"act on the route that collides")
    first = next((r for r in routes if r.pad.pad_number == 'B2'), None)
    if first is None or _geom(first) != a1_before:
        fails.append("resolve_collisions moved the net's first route, which collided "
                     "with nothing")
    if failed:
        fails.append(f"resolve_collisions dropped {failed}, though the colliding "
                     f"route had a free alternate channel")


def check_pad_repair(fails):
    a1 = _route(1, -1.5, -1.5, 'left', -1.0, 'B2')
    a2 = _route(1, 0.5, 1.5, 'right', 1.0, 'E4')
    routes = [a1, a2]
    tracks, _, _ = _quiet(lambda: generate_tracks_from_routes(routes, TW, 'F.Cu'))
    # a big foreign through-hole pad over every way out of the second ball
    block = make_pad(3, 2.0, 1.0, ref='J1', num='1', net_name='N3', shape='rect',
                     size_x=1.5, size_y=4.0, drill=1.0, pad_type='thru_hole',
                     layers=('*.Cu',))
    u1, u2 = route_uid(a1), route_uid(a2)
    n1 = sum(1 for t in tracks if t.get('route_uid') == u1)
    if not n1 or reroute.route_clear_of_foreign_pads(a2, [block], {'F.Cu': 0}):
        fails.append("fixture: the second route does not cross the foreign pad, or "
                     "the first has no tracks")
        return
    _, failed = _quiet(lambda: reroute.repair_pad_crossings(
        routes, tracks, ['F.Cu'], TW, CL, 0.2, [], GRID, CHANNELS, 0.5,
        {1: 'N1'}, False, object(), object(), {'F.Cu': 0}, [block]))
    if failed != ['N1']:
        fails.append(f"the unrepairable route was not given up and reported "
                     f"(failed {failed})")
    if any(t.get('route_uid') == u2 for t in tracks):
        fails.append("the given-up route's tracks are still there")
    kept = sum(1 for t in tracks if t.get('route_uid') == u1)
    if kept != n1:
        fails.append(f"giving up one route of net 1 removed {n1 - kept} of the "
                     f"{n1} track(s) of its OTHER route")


def main():
    print('=' * 60)
    print("BGA channel escape: collision and pad repairs act on the route, not the net")
    print('=' * 60)
    fails = []
    check_resolver(fails)
    check_pad_repair(fails)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print("PASS: resolve_collisions moved the colliding route of a two-ball net and left "
          "its first route alone; repair_pad_crossings gave up one route and kept the "
          "net's other route's tracks")
    return 0


if __name__ == '__main__':
    sys.exit(main())

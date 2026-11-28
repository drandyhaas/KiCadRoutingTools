"""An ordered bus (--bus-nets): members declared by net list keep the list's order side by side.

Each member is attracted to its own lane, the planned corridor moved sideways by its place in the order, one
track + clearance apart; the lane points along each route's own direction of travel, because the router's
attraction rewards only moves along its path. Checks:
  1. lanes one pitch apart round the representative's lane, on the corridor's left-hand normal;
  2. a straight corridor offset exactly; an L-shaped one keeps every lane on its own side through the bend;
  3. the lanes mirrored when the members' copper runs the other way across the corridor;
  4. a declared bus keeps the given order; an unknown or repeated net, or a one-net bus, refused;
  5. the representative is the farthest-reaching member;
  6. a lane turned to the route's direction (forward and backward probes alike), the router left alone otherwise;
  7. the attraction lookup takes a member's lane before the shared centreline.

    python3 -m pytest tests/test_ordered_bus.py
"""
import os
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))
import bus_detection as bd  # noqa: E402


def straight(n=50, layer=1):
    return [(x, 0, layer) for x in range(n)]


def test_lane_offsets_round_the_representative():
    assert bd.lane_offsets(3, 4.0) == [-4.0, 0.0, 4.0]
    assert bd.lane_offsets(4, 2.0) == [-3.0, -1.0, 1.0, 3.0]
    assert bd.lane_offsets(4, 2.0, centre=0) == [0.0, 2.0, 4.0, 6.0]


def test_offset_path_straight_and_bend():
    # +x travel: the left-hand normal is +y
    assert bd.offset_path(straight(), 3) == [(x, 3, 1) for x in range(50)]
    assert bd.offset_path(straight(), 0) == straight()
    # an L: east then north; a lane on the left stays left (inside the bend), one on the right stays right
    ell = [(x, 0, 0) for x in range(40)] + [(39, y, 0) for y in range(1, 40)]
    left, right = bd.offset_path(ell, 4), bd.offset_path(ell, -4)
    assert all(y == 4 for x, y, _ in left[:20]) and all(x == 35 for x, y, _ in left[-15:])
    assert all(y == -4 for x, y, _ in right[:20]) and all(x == 43 for x, y, _ in right[-15:])
    assert all(layer == 0 for *_, layer in left + right)


def test_lanes_follow_the_members_copper():
    path = straight()
    up = [[(0, -4)], [(0, 0)], [(0, 4)]]        # member 0 below, member 2 above: along the normal
    down = [[(0, 4)], [(0, 0)], [(0, -4)]]
    lanes = bd.lane_paths(path, [10, 11, 12], 4.0, up)
    assert lanes[10][0][1] == -4 and lanes[12][0][1] == 4
    lanes = bd.lane_paths(path, [10, 11, 12], 4.0, down)
    assert lanes[10][0][1] == 4 and lanes[12][0][1] == -4
    # the representative's lane is the corridor itself
    lanes = bd.lane_paths(path, [10, 11, 12], 4.0, up, centre=0)
    assert lanes[10] == path and lanes[12][0][1] == 8


def board(nets, pads=None, segments=()):
    return NS(nets={nid: NS(name=name) for nid, name in nets.items()},
              pads_by_net=pads or {}, segments=list(segments), vias=[])


def pad(x, y):
    return NS(global_x=x, global_y=y)


def test_declared_bus_keeps_the_given_order(monkeypatch):
    monkeypatch.setattr(bd, 'get_net_routing_endpoints', lambda pcb, nid: [(0.0, nid), (10.0, nid)])
    pcb = board({1: 'A', 2: 'B', 3: 'C'})
    (g,) = bd.declared_bus_groups(pcb, [['C', 'A', 'B']])
    assert g.ordered and g.net_ids == [3, 1, 2] and g.name == 'declared_bus_0'
    assert g.source_positions == [(0.0, 3), (0.0, 1), (0.0, 2)]
    with pytest.raises(ValueError, match='no net named'):
        bd.declared_bus_groups(pcb, [['A', 'Z']])
    with pytest.raises(ValueError, match='twice'):
        bd.declared_bus_groups(pcb, [['A', 'B'], ['B', 'C']])
    with pytest.raises(ValueError, match='two nets'):
        bd.declared_bus_groups(pcb, [['A']])


def test_representative_is_the_farthest_reaching_member():
    pcb = board({1: 'A', 2: 'B', 3: 'C'},
                pads={1: [pad(0, 0), pad(30, 0)], 2: [pad(0, 1), pad(5, 1), pad(80, 1)], 3: [pad(0, 2), pad(50, 2)]},
                segments=[NS(net_id=3, start_x=50, start_y=2, end_x=60, end_y=2)])
    assert bd.net_span(pcb, 2) == (80.0, ((0, 1), (80, 1)))
    assert bd.net_span(pcb, 3)[0] == 60.0      # its track's far end counts
    g = bd.BusGroup(name='b', net_ids=[1, 2, 3], ordered=True)
    assert bd.ordered_bus_representative(pcb, g) == 2


class FakeRouter:
    def __init__(self):
        self.paths, self.calls = [], []

    def set_attraction_path(self, path):
        self.paths.append(list(path))

    def route_with_frontier(self, obstacles, sources, targets, *a, **k):
        self.calls.append(('frontier', self.paths[-1][0]))
        return 'routed'

    def route_multi(self, obstacles, sources, targets, *a, **k):
        self.calls.append(('multi', self.paths[-1][0]))
        return 'multi'

    other = 'kept'


def test_lane_turned_to_each_route():
    lane = bd.Lane(straight())
    r = FakeRouter()
    w = bd.LaneOrientedRouter(r, lane)
    west, east = [(0, 0, 1)], [(49, 0, 1)]
    assert w.route_with_frontier(None, west, east, 100) == 'routed'
    assert w.route_with_frontier(None, east, west, 100) == 'routed'   # the backward probe
    assert w.route_multi(None, west, east) == 'multi'
    assert [c[1] for c in r.calls] == [(0, 0, 1), (49, 0, 1), (0, 0, 1)]
    assert len(r.paths) == 3                     # set once, then only when the way changes
    assert w.other == 'kept'


def test_attraction_takes_the_members_lane_first():
    g = bd.BusGroup(name='b', net_ids=[1, 2], ordered=True)
    centre, lane = [(0, 0, 0)], bd.Lane([(0, 4, 0)])
    attr, rev = bd.bus_attraction_context(2, {1: g, 2: g}, {'b': centre, ('b', 2): lane})
    assert attr is lane and rev is False
    attr, _ = bd.bus_attraction_context(1, {1: g, 2: g}, {'b': centre, ('b', 2): lane})
    assert attr is centre
    assert bd.bus_attraction_context(3, {1: g}, {'b': centre}) == (None, False)

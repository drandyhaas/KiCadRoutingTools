"""Laid lanes (--bus-lanes): a declared ordered bus laid as copper along a drawn centreline, its members joining it.

Checks of the geometry (bus_lanes.py):
  1. a straight centreline: the lanes one pitch apart in the declared order, round the middle;
  2. an L-shaped centreline: every lane keeps the pitch through the bend (mitred), on its own side;
  3. the fan: the last vertex's spacing at the fan pitch, the others at the pitch;
  4. the outlet decides the side: members whose outlet copper runs against the normal get mirrored lanes, whichever
     side their far ends are on;
  5. a drop point at the lane's point nearest the member's far end, or the first legal point downstream, or none;
  6. the automatic pitch leaves room for a via beside the next lane;
  7. a via clears the other lanes and vias, and not its own.

    python3 tests/test_bus_lanes.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router'))
import bus_lanes as bl  # noqa: E402


class approx:
    """A number or a sequence of numbers, equal to within `abs` (the pytest.approx shape, without pytest)."""
    def __init__(self, value, abs=1e-6):
        self.value, self.tol = value, abs

    def __eq__(self, other):
        def same(a, b):
            if isinstance(a, (list, tuple)):
                return isinstance(b, (list, tuple)) and len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
            return abs(a - b) <= self.tol
        return same(self.value, other)

    def __repr__(self):
        return f'approx({self.value!r})'


def test_straight_lanes_one_pitch_apart_round_the_middle():
    lanes = bl.lane_polylines([(0, 0), (10, 0)], 4, 0.5)
    ys = [lane[0][1] for lane in lanes]
    # the left-hand normal of +x in board coordinates (y down) is +y
    assert ys == approx([-0.75, -0.25, 0.25, 0.75])
    assert all(lane[1][1] == approx(lane[0][1]) for lane in lanes)


def test_lanes_keep_the_pitch_through_a_bend():
    lanes = bl.lane_polylines([(0, 0), (10, 0), (10, 10)], 3, 1.0)
    for a, b in zip(lanes, lanes[1:]):
        for (p, q), (r, s) in zip(zip(a, a[1:]), zip(b, b[1:])):
            # the distance between neighbouring lanes' matching segments, at their midpoints
            mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
            assert bl.point_segment_distance(mid, r, s) == approx(1.0, abs=1e-6)


def test_the_fan_spreads_the_last_vertex_only():
    lanes = bl.lane_polylines([(0, 0), (10, 0), (20, 0)], 3, 0.5, fan_pitch=1.5)
    assert [lane[1][1] for lane in lanes] == approx([-0.5, 0, 0.5])
    assert [lane[2][1] for lane in lanes] == approx([-1.5, 0, 1.5])


def test_the_outlet_decides_the_side():
    centre = [(0, 0), (10, 0)]
    in_order = [(10, -1), (10, 0), (10, 1)]           # the members in order along the +y normal at the outlet
    assert bl.outlet_sign(centre, in_order) == 1
    assert bl.outlet_sign(centre, list(reversed(in_order))) == -1
    lanes = bl.lane_polylines(centre, 3, 1.0, sign=bl.outlet_sign(centre, in_order))
    assert [lane[-1][1] for lane in lanes] == approx([-1, 0, 1])
    assert bl.outlet_sign(centre, [None, None, None]) == 1


def test_a_drop_point_where_the_member_joins_else_downstream():
    lane = [(0, 0), (10, 0)]
    p, rest = bl.drop_point(lane, (3, 5), lambda q: True)
    assert p == approx((3, 0)) and rest[-1] == (10, 0)
    p, rest = bl.drop_point(lane, (3, 5), lambda q: q[0] >= 4.0, step=0.25)
    assert p[0] == approx(4.0) and rest[0] == approx(p)
    assert bl.drop_point(lane, (3, 5), lambda q: False, reach=2.0) is None
    # walked from the nearest point (4.9, 0) round the bend, 0.5 mm a step: 0.4, 0.9, then 1.4
    p, rest = bl.drop_point([(0, 0), (5, 0), (5, 5)], (4.9, -3), lambda q: q[1] >= 1.0, step=0.5)
    assert p == approx((5, 1.4)) and rest == [p, (5, 5)]


def test_the_automatic_pitch_leaves_room_for_a_via():
    assert bl.auto_pitch(0.2, 0.2, 0.6) == approx(0.6)
    assert bl.auto_pitch(0.25, 0.2, 0.4) == approx(0.525)     # 0.2 + 0.125 + 0.2


def test_a_via_clears_the_other_lanes_and_vias_not_its_own():
    lanes = {0: [(0, 0), (10, 0)], 1: [(0, 0.6), (10, 0.6)]}
    assert bl.clear_of_lanes((5, 0), 0.3, lanes, 0, 0.1, 0.2, [], 0.6)
    assert not bl.clear_of_lanes((5, 0), 0.3, lanes, 0, 0.1, 0.25, [], 0.6)
    assert not bl.clear_of_lanes((5, 0), 0.3, lanes, 0, 0.1, 0.2, [((5.5, 0.6), 1)], 0.6)
    assert bl.clear_of_lanes((5, 0), 0.3, lanes, 0, 0.1, 0.2, [((5.5, 0.0), 0)], 0.6)


def test_an_unused_drop_via_goes_and_the_lane_starts_at_the_ramps():
    from types import SimpleNamespace as NS
    lane = NS(net_id=1, layer='In2.Cu', start_x=0.0, start_y=0.0, end_x=10.0, end_y=0.0)
    drop = NS(net_id=1, x=0.0, y=0.0, size=0.6)
    ramp_via = NS(net_id=1, x=1.5, y=0.0, size=0.6)
    laid = {'laid': {1: {'drop': (0.0, 0.0), 'layer': 'In2.Cu'}}}
    removed = bl.trim_unused_drops(laid, [lane], [drop, ramp_via], {})
    assert removed == [drop] and (lane.start_x, lane.start_y) == (1.5, 0.0)
    assert laid['laid'][1]['drop'] == (1.5, 0.0)
    # a drop its ramp reaches on another layer stays
    lane2 = NS(net_id=1, layer='In2.Cu', start_x=0.0, start_y=0.0, end_x=10.0, end_y=0.0)
    ramp = NS(net_id=1, layer='F.Cu', start_x=-3.0, start_y=4.0, end_x=0.0, end_y=0.0)
    laid2 = {'laid': {1: {'drop': (0.0, 0.0), 'layer': 'In2.Cu'}}}
    assert bl.trim_unused_drops(laid2, [lane2, ramp], [NS(net_id=1, x=0.0, y=0.0, size=0.6)], {}) == []


def test_nothing_laid_or_banned_without_bus_lanes():
    from types import SimpleNamespace as NS
    config = NS(bus_nets=[['A', 'B', 'C']], bus_lanes=[])
    assert bl.laid_specs(config) == [] and bl.layer_bans(config) == []
    config = NS(bus_nets=[['A', 'B'], ['C', 'D']], bus_lanes=[None, ('User.2', 'In2.Cu')])
    assert bl.laid_specs(config) == [(['C', 'D'], 'User.2', 'In2.Cu')]
    ban = bl.layer_bans(config)[0]
    assert ban['layers'] == {'In2.Cu'} and ban['nets'] == frozenset({'C', 'D'}) and ban['bus_lanes']


def test_the_writers_unchanged_unless_locked():
    from kicad_writer import generate_segment_sexpr, generate_via_sexpr
    seg = generate_segment_sexpr((1, 2), (3, 4), 0.2, 'In2.Cu', 5)
    via = generate_via_sexpr(1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5)
    assert '(locked' not in seg and '(locked' not in via
    assert '(width 0.2)\n\t\t(locked yes)\n\t\t(layer "In2.Cu")' in generate_segment_sexpr(
        (1, 2), (3, 4), 0.2, 'In2.Cu', 5, locked=True)
    assert '(layers "F.Cu" "B.Cu")\n\t\t(locked yes)' in generate_via_sexpr(
        1, 2, 0.6, 0.3, ['F.Cu', 'B.Cu'], 5, locked=True)


def test_two_members_vote_by_their_outlet_pads():
    from types import SimpleNamespace as NS
    centre = [(0, 0), (10, 0)]                 # the normal at the end: +y
    pcb = NS(pads_by_net={1: [NS(global_x=10, global_y=-1), NS(global_x=2, global_y=5)],
                          2: [NS(global_x=10, global_y=1), NS(global_x=3, global_y=-5)]}, segments=[])
    outlets = [bl.outlet_point(pcb, 1, (10, 0)), bl.outlet_point(pcb, 2, (10, 0))]
    assert outlets == [(10, -1), (10, 1)]    # the pads at the outlet, not their far ends across the run
    assert bl.outlet_sign(centre, outlets) == 1
    pcb.pads_by_net = {}                       # no pad: its copper nearest the end
    pcb.segments = [NS(net_id=1, start_x=2, start_y=5, end_x=9, end_y=1)]
    assert bl.outlet_point(pcb, 1, (10, 0)) == (9, 1)


TESTS = [
    test_straight_lanes_one_pitch_apart_round_the_middle,
    test_lanes_keep_the_pitch_through_a_bend,
    test_the_fan_spreads_the_last_vertex_only,
    test_the_outlet_decides_the_side,
    test_a_drop_point_where_the_member_joins_else_downstream,
    test_the_automatic_pitch_leaves_room_for_a_via,
    test_a_via_clears_the_other_lanes_and_vias_not_its_own,
    test_an_unused_drop_via_goes_and_the_lane_starts_at_the_ramps,
    test_nothing_laid_or_banned_without_bus_lanes,
    test_the_writers_unchanged_unless_locked,
    test_two_members_vote_by_their_outlet_pads,
]


def main():
    failed = []
    for fn in TESTS:
        try:
            fn()
            print(f"PASS  {fn.__name__}")
        except Exception as e:                       # noqa: BLE001 -- a failing check, reported and counted
            failed.append(fn.__name__)
            print(f"FAIL  {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(TESTS) - len(failed)}/{len(TESTS)} passed")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

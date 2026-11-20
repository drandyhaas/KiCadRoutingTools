"""Laid lanes (--bus-lanes) end to end: tests/fixtures/bus_lanes/demo.kicad_pcb routed through route.py.

The demo (tests/make_bus_lanes_demo.py): six nets A0..A5 declared as an ordered bus, their far ends alternating
between the top and the bottom edge (tributaries from both sides), their outlet a column of pads in the bus's order,
the bus's centreline drawn on User.2, a pad of another net and then one of another member where A3's drop via would
go. Checks:
  1. every net routed; the run says it laid six lanes;
  2. each member has its lane on In2, locked, and the lanes are in the declared order at two sections;
  3. no member has any other track on In2 (its ramps keep off the bus layer);
  4. A3 joins downstream of both pads in its way (the other member's included);
  4b. a far-side member (A2: its far end on the top edge, two lanes between it and its own) crosses them on another
     layer, with nothing on the bus layer;
  5. two runs lay and route the same copper;
  6. without --bus-lanes nothing is laid (the attraction bus as before).

    python3 tests/test_bus_lanes_e2e.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'bus_lanes', 'demo.kicad_pcb')
BUS = ['A0', 'A1', 'A2', 'A3', 'A4', 'A5']
RUN_ALL_TIMEOUT = 900


def route(out, lanes=True):
    cmd = [sys.executable, os.path.join(ROOT, 'py_router', 'route.py'), BOARD, '--output', out,
           '--nets', *BUS, 'OBST', '--layers', 'F.Cu', 'In2.Cu', 'B.Cu', '--bus-nets', ','.join(BUS)]
    if lanes:
        cmd += ['--bus-lanes', 'User.2:In2.Cu', '--bus-fan-pitch', '1.5']
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT, timeout=600)
    summary = next((json.loads(line.split(':', 1)[1]) for line in r.stdout.splitlines()
                    if line.startswith('JSON_SUMMARY:')), None)
    assert summary is not None, r.stdout[-2000:] + r.stderr[-2000:]
    return r.stdout, summary, parse_kicad_pcb(out)


def copper(board):
    name = {i: n.name for i, n in board.nets.items()}
    segs = sorted((name.get(s.net_id), s.layer, round(s.start_x, 3), round(s.start_y, 3), round(s.end_x, 3),
                   round(s.end_y, 3), s.locked) for s in board.segments)
    vias = sorted((name.get(v.net_id), round(v.x, 3), round(v.y, 3), v.locked) for v in board.vias)
    return segs, vias


_LAID = []


def laid():
    """Two runs with --bus-lanes, routed once for every check below."""
    if not _LAID:
        tmp = tempfile.mkdtemp()
        _LAID.append((route(os.path.join(tmp, 'a.kicad_pcb')), route(os.path.join(tmp, 'b.kicad_pcb'))))
    return _LAID[0]


def test_every_net_routed_and_six_lanes_laid():
    (log, summary, _), _ = laid()
    assert summary['failed'] == 0 and not summary['failed_single']
    assert 'bus lanes on In2.Cu along User.2: 6 laid' in log


def test_the_lanes_in_order_on_the_bus_layer():
    (_, _, board), _ = laid()
    segs, _ = copper(board)
    for x in (30.0, 44.0):
        at = {}
        for n, layer, x0, y0, x1, y1, locked in segs:
            if n in BUS and layer == 'In2.Cu' and locked and min(x0, x1) <= x <= max(x0, x1) and x0 != x1:
                at[n] = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
        present = [n for n in BUS if n in at]
        assert present == BUS[:len(present)] or len(present) >= 4
        assert sorted(present, key=lambda n: at[n]) == present, (x, at)


def test_no_ramp_on_the_bus_layer():
    (_, _, board), _ = laid()
    segs, _ = copper(board)
    assert not [s for s in segs if s[0] in BUS and s[1] == 'In2.Cu' and not s[6]]


def test_a_member_joins_downstream_of_what_is_in_its_way():
    (_, _, board), _ = laid()
    segs, _ = copper(board)
    a3 = [s for s in segs if s[0] == 'A3' and s[1] == 'In2.Cu' and s[6]]
    start = min(min(s[2], s[4]) for s in a3)
    assert start > 21.6 + 0.4, start          # past OB1 (x 20.4) and A5's pad (x 21.6), not at its tap's x 20


def test_a_far_side_member_crosses_the_river_off_the_bus_layer():
    (_, _, board), _ = laid()
    segs, _ = copper(board)
    lanes = {s[0]: s[3] for s in segs if s[1] == 'In2.Cu' and s[6] and s[2] < 30}      # (net) -> its lane's y
    top_of_a2 = min(lanes['A0'], lanes['A1']) - 0.2                                    # A0's and A1's lanes lie
    assert lanes['A2'] > max(lanes['A0'], lanes['A1'])                                 # between A2's tap and lane
    crossing = [s for s in segs if s[0] == 'A2' and s[1] != 'In2.Cu' and
                min(s[3], s[5]) <= top_of_a2 and max(s[3], s[5]) >= lanes['A2'] - 0.1]
    assert crossing, 'A2 reaches its lane across A0 and A1 on another layer'


def test_two_runs_lay_and_route_the_same_copper():
    (_, _, a), (_, _, b) = laid()
    assert copper(a) == copper(b)


def test_without_bus_lanes_nothing_is_laid():
    with tempfile.TemporaryDirectory() as tmp:
        log, summary, board = route(os.path.join(tmp, 'c.kicad_pcb'), lanes=False)
    assert 'bus lanes' not in log
    segs, vias = copper(board)
    assert not [s for s in segs if s[6]] and not [v for v in vias if v[3]]


TESTS = [
    test_every_net_routed_and_six_lanes_laid,
    test_the_lanes_in_order_on_the_bus_layer,
    test_no_ramp_on_the_bus_layer,
    test_a_member_joins_downstream_of_what_is_in_its_way,
    test_a_far_side_member_crosses_the_river_off_the_bus_layer,
    test_two_runs_lay_and_route_the_same_copper,
    test_without_bus_lanes_nothing_is_laid,
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

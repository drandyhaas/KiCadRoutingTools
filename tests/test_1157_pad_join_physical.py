#!/usr/bin/env python3
"""#1157: two same-net pads are one terminal only where their copper meets.

The connectivity graph joined two pads when their edge gap was within the
caller's endpoint tolerance (0.02 in check_net_connectivity, 0.05 for the
no-copper fast path, 0.06 in the oracle's exact_clusters). On a placed
StickHub, C24.1 and C34.1 (+5V) sat 15 um apart: our graph called +5V
connected, KiCad reported a link open, and the multipoint router -- which
groups terminals on that graph (#317) -- never planned the link. The
improvement gate read the same graph and reverted a run for a net that was
already open on its input.

Checks:
  1. check_drc.pad_copper_gap is EXACT: never above, and within 1e-6 of, a
     dense perimeter sampling, over random rect/roundrect/circle/oval pads at
     several rotations; a plus-shaped crossing and a contained pad read 0.
  2. Two pads 15 um apart are two components in check_net_connectivity,
     _net_pads_connected_by_overlap, the multipoint terminal grouping, the
     improvement gate's map and exact_clusters; overlapping or exactly
     touching pads stay one.
  3. A point-like terminal (no outline) still joins a pad within the caller's
     tolerance -- that question is endpoint coincidence, not copper.

    python3 tests/test_1157_pad_join_physical.py
"""
import os
import random
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from kicad_parser import BoardInfo                          # noqa: E402
from synth import make_pad, make_pcb, make_net              # noqa: E402
from check_drc import (pad_copper_gap, point_to_pad_distance,  # noqa: E402
                       _pad_perimeter_points)
from check_connected import (check_net_connectivity,        # noqa: E402
                             _net_pads_connected_by_overlap, _pads_join)
from connectivity import get_copper_connected_terminal_groups  # noqa: E402
from improvement_gate import net_connectivity_map           # noqa: E402
from kicad_exact_fill import exact_clusters                 # noqa: E402

failures = []
NET = 1


def check(label, got, want):
    ok = got == want
    print(f"  [{'ok' if ok else 'FAIL'}] {label}: got {got!r}, want {want!r}")
    if not ok:
        failures.append(label)


def rr(x, y, sx=0.55, sy=0.8, ref='C1', shape='roundrect', rratio=0.25, rot=0):
    p = make_pad(NET, x, y, ref=ref, size_x=sx, size_y=sy, shape=shape,
                 net_name='+5V')
    p.roundrect_rratio = rratio
    p.rect_rotation = rot
    return p


def board(pads):
    bi = BoardInfo(layers={}, board_bounds=(-10, -10, 10, 10),
                   copper_layers=['F.Cu', 'B.Cu'])
    return make_pcb(nets={NET: make_net(NET, '+5V')}, board_info=bi,
                    pads_by_net={NET: list(pads)})


print("1. pad_copper_gap is exact")
random.seed(1157)


def rand_pad():
    shape = random.choice(['rect', 'roundrect', 'circle', 'oval'])
    sx, sy = random.uniform(0.2, 2.0), random.uniform(0.2, 2.0)
    if shape == 'circle':
        sy = sx
    return rr(random.uniform(-1, 1), random.uniform(-1, 1), sx, sy,
              shape=shape, rratio=random.uniform(0, 0.5),
              rot=random.choice([0, 30, 45, -60, 17.5]))


def dense_gap(a, b, n=400):
    d = min(point_to_pad_distance(x, y, b) for x, y in _pad_perimeter_points(a, n))
    return min(d, min(point_to_pad_distance(x, y, a)
                      for x, y in _pad_perimeter_points(b, n)))


above, worst, apart = 0, 0.0, 0
for _ in range(150):
    a, b = rand_pad(), rand_pad()
    e, d = pad_copper_gap(a, b), dense_gap(a, b)
    apart += e > 0
    above += e > d + 1e-9
    worst = max(worst, d - e)
check("exact gap above the sampled one", above, 0)
check("sampled - exact within 1e-6", worst < 1e-6, True)
check("population has separated pairs", apart > 30, True)
check("15 um rect gap", round(pad_copper_gap(rr(0, 0, rratio=0, shape='rect'),
                                             rr(0.565, 0.11, rratio=0, shape='rect')), 9),
      0.015)
check("plus-shaped crossing", pad_copper_gap(rr(0, 0, 4, 0.1, shape='rect'),
                                             rr(0, 0, 0.1, 4, shape='rect')), 0.0)
check("contained pad", pad_copper_gap(rr(0, 0, 3, 3, shape='rect'),
                                      rr(0.2, 0.1, 0.3, 0.3, shape='rect')), 0.0)
tri = rr(0, 0, 1, 1, shape='custom')
tri.polygons = [[(-0.5, -0.5), (0.5, -0.5), (0, 0.5)]]
check("custom polygon vs circle", round(
    pad_copper_gap(tri, rr(0, 1.0, 0.5, 0.5, shape='circle')), 9), 0.25)

print("2. a positive gap is two terminals; touching copper is one")
# The StickHub pair's geometry: 0.55 x 0.8 pads, 15 um apart in x and
# overlapping 0.11 mm in y, so the gap is between two straight edges.
straight = [rr(0, 0, ref='C24', rratio=0, shape='rect'),
            rr(0.565, 0.69, ref='C34', rratio=0, shape='rect')]
overlap = [rr(0, 0, ref='C24'), rr(0.50, 0.0, ref='C34')]
touch = [rr(0, 0, ref='C24', rratio=0, shape='rect'),
         rr(0.55, 0.3, ref='C34', rratio=0, shape='rect')]
for label, pads, want in (("15 um apart", straight, False),
                          ("overlapping", overlap, True),
                          ("exactly touching", touch, True)):
    pcb = board(pads)
    r = check_net_connectivity(NET, [], [], pads, [], tolerance=0.02, pcb_data=pcb)
    check(f"check_net_connectivity, {label}", bool(r.get('connected')), want)
    check(f"_net_pads_connected_by_overlap, {label}",
          _net_pads_connected_by_overlap(pads, ['F.Cu', 'B.Cu']), want)
    # pad_info rows as the router builds them: coords at [3]/[4], the pad at [5]
    info = [(0, 0, 0, p.global_x, p.global_y, p) for p in pads]
    groups = get_copper_connected_terminal_groups(pcb, NET, info)
    check(f"terminal groups, {label}", len(set(groups.values())), 1 if want else 2)
    m = net_connectivity_map(pcb)
    check(f"improvement-gate map, {label}", m[NET][0], want)
    cl = exact_clusters(pcb, NET, [])
    check(f"exact_clusters, {label}", len(cl), 1 if want else 2)

print("3. a point-like terminal keeps the coincidence tolerance")
stub = make_pad(NET, 0.275 + 0.015, 0.0, size_x=0, size_y=0, shape='')
check("stub 15 um from a pad edge joins at 0.02",
      _pads_join(stub, straight[0], 0.02), True)
check("stub 30 um from a pad edge does not",
      _pads_join(make_pad(NET, 0.275 + 0.03, 0.0, size_x=0, size_y=0, shape=''),
                 straight[0], 0.02), False)

print()
if failures:
    print(f"FAIL: {len(failures)} check(s): {failures}")
    sys.exit(1)
print("PASS: test_1157_pad_join_physical")

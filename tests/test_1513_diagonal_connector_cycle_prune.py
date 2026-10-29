#!/usr/bin/env python3
"""Issue #1513: the cycle prune must not read a short DIAGONAL connector as a
self-loop.

``cluster_coincident_points`` joined two ends when |dx| < tol and |dy| < tol,
a box that reaches tol * sqrt(2) along a diagonal. check_drc measures a soft
joint's gap with hypot and counts anything above COINCIDENCE_TOL, so a
0.0238 mm diagonal connector had both ends in ONE cycle node and
``prune_redundant_cycles`` dropped it as redundant. A stacked duplicate of
the neighbouring segment kept the end's degree at 2, so the soft-joint
restore saw nothing; the later stacked-copper dedup removed the duplicate
and the board shipped a counted ``segment-endpoint-gap``.

Pinned here:
  1. the primitive clusters by distance: a 0.0238 mm diagonal pair is two
     clusters, a 0.0184 mm one (inside the band check_drc only warns on) and
     an axis-aligned 0.019 mm one are still one;
  2. the prune keeps the diagonal connector and drops the stacked duplicate
     instead, so the surviving copper meets at shared vertices end to end;
  3. control: inside the warning band (0.0184 mm) the connector is still
     pruned, as #320 defines it.

    python3 tests/test_1513_diagonal_connector_cycle_prune.py
"""
import math
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
for _d in ('py_router', 'rust_router', os.path.join('tests', 'oracle')):
    sys.path.insert(0, os.path.join(_ROOT, _d))

from constraint_agreement import write_board  # noqa: E402
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from connectivity import COINCIDENCE_TOL, cluster_coincident_points  # noqa: E402
from pcb_modification import prune_redundant_cycles  # noqa: E402

FAILS = []


def check(cond, msg):
    print(('  ok   ' if cond else '  FAIL ') + msg)
    if not cond:
        FAILS.append(msg)


def n_clusters(p, q):
    return len(set(cluster_coincident_points([(p[0], p[1], 'F.Cu'),
                                              (q[0], q[1], 'F.Cu')],
                                             COINCIDENCE_TOL)))


def prune(board, d):
    """Chain pad -> B (twice, stacked) -> C=B+(d,d) -> pad; returns
    (removed, kept) as coordinate tuples."""
    pcb = parse_kicad_pcb(board)

    def seg(a, b):
        return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                       width=0.1, layer='F.Cu', net_id=1)
    B = (14.0, 10.0)
    C = (B[0] + d, B[1] + d)
    pcb.segments = [seg((10, 10), B), seg((10, 10), B), seg(B, C),
                    seg(C, (20, 10.2))]
    _n, _nets, removed = prune_redundant_cycles([], pcb, {1}, clearance=0.1)
    key = lambda s: ((s.start_x, s.start_y), (s.end_x, s.end_y))  # noqa: E731
    return [key(s) for s in removed], [key(s) for s in pcb.segments], (B, C)


def main():
    print('1. the coincidence primitive measures a distance')
    check(n_clusters((14.0, 10.0), (14.0168, 10.0168)) == 2,
          'a 0.0238 mm diagonal pair is two clusters (check_drc counts it)')
    check(n_clusters((14.0, 10.0), (14.013, 10.013)) == 1,
          'a 0.0184 mm diagonal pair is one cluster (inside the band)')
    check(n_clusters((14.0, 10.0), (14.019, 10.0)) == 1,
          'an axis-aligned 0.019 mm pair is one cluster (unchanged)')

    td = tempfile.mkdtemp()
    board = os.path.join(td, 'p.kicad_pcb')
    write_board(board, footprints=[
        {'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A',
         'layer': 'F.Cu'},
        {'ref': 'U2', 'x': 20, 'y': 10.2, 'net_id': 1, 'net_name': 'A',
         'layer': 'F.Cu'}])

    print('2. a 0.0238 mm diagonal connector survives the cycle prune')
    d = 0.0168
    removed, kept, (B, C) = prune(board, d)
    print(f'     connector {math.hypot(d, d):.4f} mm, removed {removed}')
    check((B, C) not in removed, 'the connector is not removed')
    check(removed == [((10, 10), B)],
          'the stacked duplicate is the one redundant segment removed')
    ends = {}
    for a, b in kept:
        for p in (a, b):
            ends[p] = ends.get(p, 0) + 1
    check(ends.get(B) == 2 and ends.get(C) == 2,
          'the kept chain meets at shared vertices at both connector ends')

    print('3. control: inside the warning band the connector is still pruned')
    d = 0.013
    removed, _kept, (B, C) = prune(board, d)
    check((B, C) in removed,
          f'a {math.hypot(d, d):.4f} mm connector is pruned as coincident')

    if FAILS:
        print(f'\nFAILED ({len(FAILS)})')
        sys.exit(1)
    print('\nPASS')


if __name__ == '__main__':
    main()

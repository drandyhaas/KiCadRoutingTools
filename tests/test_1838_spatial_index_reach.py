#!/usr/bin/env python3
"""#1838: check_drc compares every same-layer track pair within reach,
wherever the 2 mm grid lines of its spatial index fall.

`SpatialIndex` filed a track under the cells its centreline was SAMPLED in
(every half cell) and `get_nearby_segments` read those cells alone. Two
tracks either side of an even-millimetre line shared no cell and were never
compared, and a diagonal could clip a cell's corner between two samples and
miss a track filed there. Now a track is filed in every cell its centreline
touches, the query reads the eight cells around each too, and the cell is
sized past the largest clearance plus track width or via diameter, so every
pair within reach is found (the class docstring has the argument).

Checks (no routing):
  1. a board with three N/M pairs 0.05 mm apart under a 0.2 mm class -- one
     inside a cell, one straddling y = 102, one a diagonal clipping a cell's
     corner between two of its old samples -- grades three segment-segment
     rows, each of which kicad-cli 10.0.6 reports on the same board;
  2. on 20000 random segment pairs closer than their clearance, the query
     (cell and reach sized as run_drc sizes them) finds the other segment,
     both ways;
  3. the cell grows past a reach larger than the default cell.

    python3 tests/test_1838_spatial_index_reach.py
"""
import json
import math
import os
import random
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []
_U = [0]


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def _seg(x0, y0, x1, y1, net):
    _U[0] += 1
    return (f' (segment (start {x0} {y0}) (end {x1} {y1}) (width 0.15) (layer "F.Cu") '
            f'(net {net}) (uuid "b0000000-0000-0000-0000-{_U[0]:012d}"))\n')


#: N/M pairs, each 0.05 mm apart edge to edge (0.062 for the diagonal);
#: name -> (the N track's start, the M track's start), as the rows give them
PAIRS = {
    'in_cell': ((109, 100.9), (109, 101.1)),
    'straddle': ((99, 101.9), (99, 102.1)),         # either side of y = 102
    # x + y = 204.5 crosses the cell (102..104, 102..104) only near its
    # corner, between the old samples at t = 1/3 and 2/3
    'corner': ((101, 103.5), (102.4, 102.4)),
}
BOARD = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6) (legacy_teardrops no))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "") (net 1 "N") (net 2 "M")\n'
         ' (gr_rect (start 90 90) (end 130 110) (stroke (width 0.05) (type default)) '
         '(fill no) (layer "Edge.Cuts") (uuid "e0000000-0000-0000-0000-000000000001"))\n'
         + _seg(109, 100.9, 111, 100.9, 1) + _seg(109, 101.1, 111, 101.1, 2)
         + _seg(99, 101.9, 101, 101.9, 1) + _seg(99, 102.1, 101, 102.1, 2)
         + _seg(101, 103.5, 103.5, 101, 1) + _seg(102.4, 102.4, 103.5, 102.4, 2)
         + ')\n')
PRO = {'board': {'design_settings': {'rules': {'min_clearance': 0.0, 'min_track_width': 0.1}}},
       'net_settings': {'meta': {'version': 4},
                        'classes': [{'name': 'Default', 'clearance': 0.2, 'track_width': 0.15}],
                        'netclass_patterns': []}}


def _pt(x, y):
    return (round(float(x), 3), round(float(y), 3))


def board(td):
    print("1. three close pairs, one per way the grid can part them")
    pcb = os.path.join(td, 'b.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    with open(os.path.join(td, 'b.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    out = os.path.join(td, 'drc.json')
    check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'), evidence(pcb, 'the board'),
                '--json', out, '--max-print', '0'], code=1, refuse='DRC VIOLATIONS')
    with open(evidence(out, 'the --json document'), encoding='utf-8') as fh:
        doc = json.load(fh)
    names = {frozenset(_pt(*p) for p in v): k for k, v in PAIRS.items()}
    got = sorted(names.get(frozenset((_pt(*v['loc1'][:2]), _pt(*v['loc2'][:2]))), '?')
                 for v in doc['items'] if v['type'] == 'segment-segment')
    # kicad-cli 10.0.6 on this board: three `clearance` errors, one per pair
    ok(got == sorted(PAIRS), f"one segment-segment row per pair: {got}")


def random_pairs():
    print("2. random pairs within reach are found both ways")
    import check_drc
    from kicad_parser import Segment
    from geometry_utils import segment_to_segment_distance as dist
    rnd = random.Random(1838)
    if not hasattr(check_drc.SpatialIndex, 'reach_cell_size'):
        ok(False, "SpatialIndex sizes its cell from the reach (reach_cell_size)")
        return
    idx_cell = check_drc.SpatialIndex.DEFAULT_CELL
    misses = tried = 0
    while tried < 20000:
        def seg(net):
            x, y = rnd.uniform(-6, 6), rnd.uniform(-6, 6)
            a, ln = rnd.uniform(0, math.pi), rnd.uniform(0, 5)
            return Segment(start_x=x, start_y=y, end_x=x + ln * math.cos(a),
                           end_y=y + ln * math.sin(a), width=rnd.uniform(0.1, 0.6),
                           layer='F.Cu', net_id=net)
        a, b = seg(1), seg(2)
        clr = rnd.uniform(0.0, 0.8)
        gap = dist(a.start_x, a.start_y, a.end_x, a.end_y,
                   b.start_x, b.start_y, b.end_x, b.end_y) - (a.width + b.width) / 2
        if gap >= clr:
            continue
        tried += 1
        sx = check_drc.SpatialIndex
        ix = sx(cell_size=sx.reach_cell_size(clr, max(a.width, b.width), 0.0),
                reach=sx.pair_reach(clr, max(a.width, b.width), 0.0))
        ix.add_segment(a, 1)
        ix.add_segment(b, 2)
        if b not in [s for s, _n in ix.get_nearby_segments(a)] \
                or a not in [s for s, _n in ix.get_nearby_segments(b)]:
            misses += 1
    ok(misses == 0, f"{tried} pairs closer than their clearance, {misses} missed")

    print("3. the cell outgrows a reach past the default")
    ok(check_drc.SpatialIndex.reach_cell_size(0.2, 0.25, 0.6) == idx_cell,
       "an ordinary board keeps the 2 mm cell")
    big = check_drc.SpatialIndex.reach_cell_size(1.5, 0.3, 0.8)
    ok(big > 1.5 + 0.8, f"1.5 mm clearance and a 0.8 mm via: cell {big:.3f} mm")


def main():
    with tempfile.TemporaryDirectory() as td:
        board(td)
    random_pairs()
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1659: the body-through-the-board gate reads the BODY, not the 0.25 mm cell
it enters.

`body_in_board` counted a cell of a fixed grid in the part's own frame, whole,
once the model entered it and all four of its corners were on the board, so the
verdict and the printed depth depended on where the body's face fell in its
cell: a body 0.24 mm inside the edge passed and one 0.02 mm inside was NOT
BUILDABLE, and an end-launch jack whose pads end level with its board slot had
no pose both check_assembly and KiCad pass. Now one real depth has one verdict
at any phase, and a body within `DEPTH_EDGE_TOL_MM` (0.1 mm) of the edge does
not gate on its length alone.

With #1603's synthetic body measurement (no KiCad needed), each real depth known
exactly:

  1. over every phase of the grid, at all four turns, a face along either of
     the part's axes: more than 0.1 mm inside gates and at most 0.1 mm does
     not, and `inside_mm` is the body's depth;
  2. a body on the edge reads nothing;
  3. a sloped face is read where it is at board depth, not over its height;
  4. what gates on length is the slice past the tolerance: a short tab past it
     in front of a long wall within it does not, a long one does;
  5. `depth_past_edge`, which sets the band's floor, is the same reading: a
     part moved in by exactly that passes, a little more gates, at every turn;
  6. check_assembly: the verdict and the printed depth.

    python3 tests/test_1659_body_depth_reads_body.py
"""
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import numpy as np  # noqa: E402

import run_utils  # noqa: E402
from placement import heights  # noqa: E402
from test_1603_body_in_board import (DESIGNED, FIXTURE, T, assembly, at,  # noqa: E402
                                     box, low, quiet, synthetic)

#: the outline of the fixture
WEST, NORTH, EAST, SOUTH = 5.0, 5.0, 31.0, 25.0
TOL = 0.1
#: real depths, mm: the issue's, the coordinator's probe, the stock jack's
DEPTHS = (0.0, 0.02, 0.05, 0.06, 0.1, 0.104, 0.12, 0.15, 0.24, 0.25, 0.5, 0.7)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def face_x_pose(rot, f, d):
    """The pose putting a wall at local x = f, body toward +x, `d` inside the
    edge that +x points at when turned `rot`."""
    return {0: (EAST - d - f, 15.0, 0.0), 90: (18.0, NORTH + d + f, 90.0),
            180: (WEST + d + f, 15.0, 180.0), 270: (18.0, SOUTH - d - f, 270.0)}[rot]


def face_y_pose(rot, f, d):
    """The same for a wall at local y = f, body toward +y."""
    return {0: (18.0, SOUTH - d - f, 0.0), 90: (EAST - d - f, 15.0, 90.0),
            180: (18.0, NORTH + d + f, 180.0), 270: (WEST + d + f, 15.0, 270.0)}[rot]


def sweep(pcb, ref, faces, body, pose):
    """Every (face phase, turn, depth): the verdict, `inside_mm`; a list of what
    was wrong, and how many were read."""
    wrong, n = [], 0
    for f in faces:
        h = synthetic(pcb, extra={ref: heights._depth_raster(low(body(f)), T)})
        for rot in (0, 90, 180, 270):
            for d in DEPTHS:
                n += 1
                c = heights.body_in_board(h, pcb, {ref: pose(rot, f, d)})
                row = next((r for r in c['rows'] if r['ref'] == ref), None)
                gates = ref in c['gating_refs']
                got = None if row is None else row['inside_mm']
                if gates != (d > TOL + 0.002) or (d == 0.0 and row is not None) \
                        or (d > 0.0 and (got is None or abs(got - d) > 0.0015)):
                    wrong.append(f'face {f} turn {rot} depth {d}: gates {gates}, reads {got}')
    return wrong, n


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, FIXTURE)

    print('one real depth, one verdict, at any phase:')
    faces = (2.51, 2.55, 2.6, 2.625, 2.7, 2.74, 2.75, 2.76, 2.9, 3.0, 3.1, 3.2)
    wrong, n = sweep(pcb, 'J3', faces, lambda f: box(f, f + 5.0, -3.25, 3.25, -1.8, 0.5),
                     face_x_pose)
    check(f'a face along the part\'s x, {n} readings', not wrong,
          f'{len(wrong)} wrong, e.g. {wrong[:3]}')
    faces = (1.51, 1.6, 1.625, 1.7, 1.74, 1.75, 1.76, 1.9, 1.99, 2.0)
    wrong, n = sweep(pcb, 'C1', faces, lambda f: box(-3.25, 3.25, f, f + 5.0, -1.8, 0.5),
                     face_y_pose)
    check(f'a face along the part\'s y, {n} readings', not wrong,
          f'{len(wrong)} wrong, e.g. {wrong[:3]}')

    print('a sloped face:')
    # x = 2.6 at height 1 to x = 3.6 at height -3: at the top of the slab
    # (-0.2 mm) its wall stands at 2.9, not at the 2.6 its triangles start at
    v = np.array([[2.6, -3.25, 1.0], [2.6, 3.25, 1.0], [3.6, 3.25, -3.0], [3.6, -3.25, -3.0],
                  [8.0, -3.25, 1.0], [8.0, 3.25, 1.0], [8.0, 3.25, -3.0], [8.0, -3.25, -3.0]])
    quads = ((0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (3, 2, 6, 7), (0, 3, 7, 4), (1, 2, 6, 5))
    wedge = np.array([t for a, b, c, d in quads for t in ((v[a], v[b], v[c]), (v[a], v[c], v[d]))])
    h = synthetic(pcb, extra={'J3': heights._depth_raster(low(wedge), T)})
    c = heights.body_in_board(h, pcb, {'J3': (EAST - 2.9 - 0.3, 15.0, 0.0)})
    row = next((r for r in c['rows'] if r['ref'] == 'J3'), {})
    check('it reads where it is at board depth', abs(row.get('inside_mm', 0) - 0.3) < 0.002,
          str(row))

    print('what gates on length is the slice past the tolerance:')
    wall = box(3.2, 8.0, -3.25, 3.25, -1.8, 0.5)
    for long_mm, gate in ((1.0, False), (3.5, True)):
        # beside the pad at the part's centre, whose own copper is left out
        tab = box(2.95, 3.2, 1.2 - (long_mm - 1.0), 1.2 + 1.0, -1.8, 0.5)
        h = synthetic(pcb, extra={'J3': heights._depth_raster(low(np.concatenate([wall, tab])), T)})
        c = heights.body_in_board(h, pcb, {'J3': (EAST - 0.05 - 3.2, 15.0, 0.0)})
        row = next((r for r in c['rows'] if r['ref'] == 'J3'), {})
        check(f'a wall 0.05 mm in, a {long_mm} mm tab 0.3 mm in: '
              + ('gates' if gate else 'does not'),
              ('J3' in c['gating_refs']) == gate and abs(row.get('inside_mm', 0) - 0.3) < 0.002,
              str(row))

    print('depth_past_edge reads the same:')
    body = box(2.74, 8.0, -3.25, 3.25, -1.8, 0.5)
    h = synthetic(pcb, extra={'J3': heights._depth_raster(low(body), T)})
    for rot, edge, axis, sign in ((0, 'east', 0, -1.0), (90, 'north', 1, 1.0),
                                  (180, 'west', 0, 1.0), (270, 'south', 1, -1.0)):
        p = list(face_x_pose(rot, 2.74, -1.0))        # 1 mm past the edge
        d = heights.depth_past_edge(h, pcb, 'J3', edge, tuple(p))
        check(f'{edge}: it is the distance to the edge plus the tolerance',
              d is not None and abs(d - (1.0 + TOL)) < 0.002, str(d))
        if d is None:
            continue

        def moved(by):
            q = list(p)
            q[axis] += sign * by
            return heights.body_in_board(h, pcb, {'J3': tuple(q)})['gating_refs']
        check(f'{edge}: moved in by exactly that it passes, 0.02 mm further it gates',
              not moved(d) and moved(d + 0.02) == ['J3'], f'{moved(d)} {moved(d + 0.02)}')

    print('check_assembly:')
    tmp = tempfile.mkdtemp(prefix='t1659_')
    try:
        for inb, rc_want in ((0.05, 0), (0.25, 4), (0.7, 4)):
            path = at(os.path.join(tmp, f'in{inb}.kicad_pcb'), (DESIGNED[0] - inb, DESIGNED[1], 0.0))
            rc, doc, text = assembly(path, synthetic(quiet(parse_kicad_pcb, path)), tmp)
            line = next((ln.strip() for ln in text.splitlines() if ln.strip().startswith('J3 ')), '')
            row = next((r for r in doc['body_in_board'] if r['ref'] == 'J3'), {})
            check(f'the jack {inb} mm in: rc {rc_want}, and the row prints its depth',
                  rc == rc_want and abs(row.get('inside_mm', 0) - inb) < 0.002
                  and f"{inb:g}mm inside the outline" in line,
                  f'rc {rc}: {line}')
            if rc_want == 0:
                check('...which does not gate, and says why',
                      doc['buildable'] is True and 'does not gate' in line and '0.1mm' in line, line)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

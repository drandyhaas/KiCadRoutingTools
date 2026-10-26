#!/usr/bin/env python3
"""#1412: the edge seat reads its depth along the whole edge, not at the middle.

`seats()` took ONE seat coordinate per turn, read at the edge's midpoint, and
`_free_stretches` scanned every position along the edge at that single depth.
When the midpoint fell inside a notch the depth was the notch's bottom, so the
part was seated that far inside the straight stretches either side of it (and
the constraint that refused it there named whatever stood in the way instead
of the notch). The depth is now read at several places along the span and the
one with the most room is kept.

Fixture, built here: a 60 x 40 board whose west edge has an 8 x 6 mm notch over
its midpoint (y 17..23). A connector with its mating face to the west.

    python3 tests/test_1412_edge_seat_notched_edge.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from kicad_parser import parse_kicad_pcb                       # noqa: E402
import pose_score                                              # noqa: E402
from placement import edge_seat, floorplan                     # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def outline(points):
    return ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   f'(layer "Edge.Cuts"))\n' for a, b in zip(points, points[1:] + points[:1]))


CONNECTOR = ('  (footprint "Connector:J" (layer "F.Cu") (at 30 20)\n'
             '    (property "Reference" "J1" (at 0 0 0) (layer "F.SilkS"))\n'
             '    (fp_rect (start -3 -4) (end 3 4) (layer "F.CrtYd"))\n'
             '    (pad "1" smd rect (at -2 -2.5) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
             '    (pad "2" smd rect (at -2 2.5) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
             '  )\n')


def seat_on(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=0.25, board_edge_clearance=0.55)
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
            'edge_connectors': [{'ref': 'J1', 'edge': 'west', 'mating_face': '-x'}]})
        return edge_seat.seats(state, pcb, path, intent, set(state.parts))


def main():
    tmp = tempfile.mkdtemp(prefix='t1412_')
    try:
        notched = [(0, 0), (60, 0), (60, 40), (0, 40), (0, 23), (8, 23), (8, 17), (0, 17)]
        plain = [(0, 0), (60, 0), (60, 40), (0, 40)]
        coords = {}
        for name, pts in (('plain', plain), ('notched', notched)):
            path = os.path.join(tmp, name + '.kicad_pcb')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write('(kicad_pcb (version 20241229) (generator "t1412")\n  (net 0 "")\n'
                         + outline(pts) + CONNECTOR + ')\n')
            evidence(path, 'synthetic board')
            out, _notes, unseated = seat_on(path)
            check(f'{name}: J1 seats', 'J1' in out and not unseated, str(unseated))
            if 'J1' in out:
                line = next(iter(out['J1'].lines.values()))
                coords[name] = line.coord
                check(f'{name}: it has room along the edge',
                      sum(f1 - f0 for f0, f1 in line.free) > 10.0, str(line.free))
        if len(coords) == 2:
            check('the notch does not set the depth of the straight edge',
                  abs(coords['notched'] - coords['plain']) < 0.05, str(coords))
            path = os.path.join(tmp, 'notched.kicad_pcb')
            out, _n, _u = seat_on(path)
            line = next(iter(out['J1'].lines.values()))
            # every free position keeps the connector's courtyard (y +-4) out
            # of the notch (y 17..23): its centre is below 13 or above 27
            check('no free position lies in front of the notch',
                  all(f1 <= 13.0 + 0.6 or f0 >= 27.0 - 0.6 for f0, f1 in line.free),
                  str(line.free))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

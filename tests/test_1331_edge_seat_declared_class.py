#!/usr/bin/env python3
"""#1331: place_simple's edge seat reads an intent's declared
`edge_connectors[].class`, as the grade does.

`edge_seat.seats` chose the parts it seats by `part_class.classify_part`
alone, so a declared class was ignored while `rule_edge_connector` graded
it. CM5_MINIMA's HDMI receptacle J501 is named for its part number
(`EDAC 690-019-298-412`, gp14 M16): its row said `"class":
"edge_receptacle"`, place_simple said "not an edge part (no part class)" and
placed it anywhere, and check_floorplan failed it as an edge part seated
7.23 mm from its edge. The seat now takes the declared class, else the
inferred one -- the order `floorplan.edge_facing_policy` reads them in.

Checks, on a small board built here:
  1. a part-number-named receptacle with a declared `edge_receptacle` is
     seated on its edge;
  2. the same row without a class is not, and the note says why;
  3. a declared class outranks the inferred one the other way too: a
     `USB_C_Receptacle` declared `fiducial` is not seated, and the note
     names the declared class.

    python3 tests/test_1331_edge_seat_declared_class.py
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


def part(ref, name, at):
    return (f'  (footprint "{name}" (layer "F.Cu") (at {at})\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS"))\n'
            f'    (fp_rect (start -4 -3) (end 4 3) (layer "F.CrtYd"))\n'
            f'    (pad "1" smd rect (at -2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
            f'    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
            '  )\n')


def seat(path, rows):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=0.25,
                                      board_edge_clearance=0.55)
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': rows})
        return edge_seat.seats(state, pcb, path, intent, set(state.parts))


def main():
    tmp = tempfile.mkdtemp(prefix='t1331s_')
    try:
        path = os.path.join(tmp, 'classes.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20241229) (generator "t1331")\n  (net 0 "")\n'
                     '  (gr_rect (start 0 0) (end 60 40) (layer "Edge.Cuts"))\n'
                     + part('J501', 'CM5IO:EDAC 690-019-298-412', '20 20 0')
                     + part('J2', 'Connector_USB:USB_C_Receptacle_GCT_USB4105',
                            '40 20 0')
                     + ')\n')
        evidence(path, 'synthetic board')

        out, notes, unseated = seat(path, [
            {'ref': 'J501', 'edge': 'south', 'class': 'edge_receptacle'}])
        check('a declared edge_receptacle is seated on its edge',
              'J501' in out and out['J501'].edge == 'south' and not unseated,
              f'{sorted(out)} {unseated}')

        out, notes, _unseated = seat(path, [{'ref': 'J501', 'edge': 'south'}])
        check('without the class it is not',
              'J501' not in out
              and any('J501' in n and 'no part class' in n for n in notes),
              str(notes))

        out, notes, _unseated = seat(path, [
            {'ref': 'J2', 'edge': 'north', 'class': 'fiducial'}])
        check('a declared class outranks an inferred receptacle',
              'J2' not in out
              and any('J2' in n and 'declared class fiducial' in n for n in notes),
              str(notes))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

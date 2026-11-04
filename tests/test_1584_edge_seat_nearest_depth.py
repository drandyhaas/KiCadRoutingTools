#!/usr/bin/env python3
"""#1584: the edge seat keeps the NEAREST depth with room, not the roomiest.

`seats()` reads a part's seat depth at nine places along its edge (#1412)
and kept whichever depth had the most free length. A line pushed inboard
past an outline feature is not interrupted by it, so it nearly always won:
one 20 x 10 mm notch on a 160 mm north edge, or one M3 hole drawn on
Edge.Cuts 6 mm from it, seated a declared 2x8 header 10.00 / 7.60 mm inside
the edge on every seed -- at x positions where the edge ahead was straight --
and `check_floorplan` printed a bare `overhang 0.0000mm ... pass`.

Pinned here, on boards built here (the issue's repro geometry):
  1. a notch and a hole no longer set the depth: J1's line is the plain
     board's flush line, and the note says nothing about an inboard line;
  2. where a nearer line has no free position (fixed parts along it), the
     inboard line is kept and the seat's note says so, with both distances;
  3. where every position read is held off the edge by the outline (a hole
     every flush position covers), the note says so with the distance;
  4. the grade discloses how far inside a classless row stands: the CLI's
     evidence line carries `setback 10.00mm [courtyard; no limit declared]`
     and the JSON row `setback_mm`, a disclosure and not a finding.

    python3 tests/test_1584_edge_seat_nearest_depth.py
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
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


LINE = '(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n'
PADS = ''.join(f'    (pad "{2 * i + r + 1}" thru_hole circle (at {-10.5 + 3 * i} {4 + 3 * r}) '
               f'(size 2 2) (drill 1.1) (layers "*.Cu" "*.Mask") (net 1 "A"))\n'
               for i in range(8) for r in (0, 1))
J1 = ('  (footprint "Connector_Molex:Molex_Micro-Fit_3.0_2x08_P3.00mm_Horizontal" '
      '(layer "F.Cu") (at {at})\n'
      '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
      '    (fp_rect (start -18 -2) (end 18 10) (stroke (width 0.05) (type default)) '
      '(layer "F.CrtYd"))\n' + PADS + '  )\n')


def chip(ref, x, y, half_w=3.0, half_h=3.0):
    return (f'  (footprint "Package_SO:SOIC-8" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            f'    (fp_rect (start {-half_w} {-half_h}) (end {half_w} {half_h}) '
            '(stroke (width 0.05) (type default)) (layer "F.CrtYd"))\n'
            '    (pad "1" smd rect (at -1 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A"))\n'
            '    (pad "2" smd rect (at 1 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A"))\n'
            '  )\n')


def board(path, points, extra='', at='60 40'):
    cuts = ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) {LINE}'
                   for a, b in zip(points, points[1:] + points[:1]))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1584")\n  (net 0 "")\n'
                 '  (net 1 "A")\n' + cuts + extra + J1.replace('{at}', at) + ')\n')
    evidence(path, 'synthetic board')
    return path


def intent():
    return floorplan.intent_from_dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
        'edge_connectors': [{'ref': 'J1', 'edge': 'north',
                             'overhang_mm': {'min': 0.0, 'max': 0.0}}]}, source_path='t')


def seat_on(path):
    """`(courtyard mm inside the north edge, J1's note, unseated)`."""
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=0.25, board_edge_clearance=0.2)
        out, notes, unseated = edge_seat.seats(state, pcb, path, intent(), {'J1'})
    note = next((n for n in notes if n.startswith('edge connector J1')), '')
    if 'J1' not in out:
        return None, note, unseated
    line = next(iter(out['J1'].lines.values()))
    top = line.coord + state.parts['J1'].rect(0.0, 0.0, 0.0)[1]
    return round(top - pcb.board_info.board_bounds[1], 2) + 0.0, note, unseated


def main():
    tmp = tempfile.mkdtemp(prefix='t1584_')
    try:
        rect = [(0, 0), (160, 0), (160, 80), (0, 80)]
        notched = [(0, 0), (30, 0), (30, 10), (50, 10), (50, 0), (160, 0), (160, 80), (0, 80)]
        hole = f'  (gr_circle (center 40 6) (end 41.6 6) {LINE}'
        plain = board(os.path.join(tmp, 'plain.kicad_pcb'), rect, chip('U1', 100, 50))
        inside, note, _u = seat_on(plain)
        check('plain board: J1 seats flush', inside == 0.0, f'{inside}')
        for name, pts, extra in (('notched', notched, ''), ('holed', rect, hole)):
            path = board(os.path.join(tmp, name + '.kicad_pcb'), pts, extra + chip('U1', 100, 50))
            inside, note, unseated = seat_on(path)
            check(f'{name}: J1 seats', inside is not None, str(unseated))
            check(f'{name}: the outline feature does not set the depth (flush, '
                  f'as on the plain board)', inside == 0.0, f'{inside} mm inside')
            check(f'{name}: the note names no inboard line',
                  'only an inboard line' not in note, note)

        # 2. the flush line is blocked along its whole length by fixed strips
        # (their courtyards reach y 9, the flush courtyard y 0..12); the line
        # 10 mm inside, in front of the notch's floor, is free.
        strips = (chip('L1', 7, 4.75, 7, 4.25) + chip('L2', 110, 4.75, 50, 4.25))
        path = board(os.path.join(tmp, 'blocked.kicad_pcb'), notched, strips)
        inside, note, unseated = seat_on(path)
        check('blocked: J1 is seated on the inboard line', inside == 10.0,
              f'{inside} {unseated}')
        check('blocked: the note says only an inboard line has room, with both distances',
              'only an inboard line has room: seated 10.00mm inside the north edge '
              '(courtyard) at turn 0, the line 0.00mm inside having no free position' in note,
              note)

        # 3. a 60 mm board: every flush position covers the hole
        narrow = [(0, 0), (60, 0), (60, 80), (0, 80)]
        path = board(os.path.join(tmp, 'narrow.kicad_pcb'), narrow,
                     f'  (gr_circle (center 30 6) (end 31.6 6) {LINE}' + chip('U1', 30, 50), at='30 40')
        inside, note, unseated = seat_on(path)
        check('narrow holed: J1 is seated on the line below the hole', inside == 7.6,
              f'{inside} {unseated}')
        check('narrow holed: the note says the outline holds it there, with the distance',
              'only an inboard line has room: seated 7.60mm inside the north edge '
              '(courtyard) at turn 0, the outline' in note, note)

        # 4. the grade discloses the setback of a classless row: J1 written
        # with its courtyard 10 mm inside a straight north edge
        path = board(os.path.join(tmp, 'inboard.kicad_pcb'), rect, chip('U1', 100, 50), at='60 12')
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(path)
            res = floorplan.grade(intent(), pcb, path)
            text = floorplan.format_text(res)
        row = next((e for e in res.edge_connector_evidence if e['ref'] == 'J1'), {})
        check('grade: the JSON row carries setback_mm on the courtyard',
              row.get('setback_mm') is not None and abs(row['setback_mm'] - 10.0) < 0.01
              and row.get('setback_basis') == 'courtyard' and row.get('setback_limit_mm') is None,
              str({k: row.get(k) for k in ('setback_mm', 'setback_basis', 'setback_limit_mm')}))
        line = next((ln for ln in text.splitlines() if ln.strip().startswith('J1 north: overhang')), '')
        check('grade: the evidence line discloses it, naming that no limit is declared',
              '; setback 10.00mm [courtyard; no limit declared]' in line, line)
        check('grade: a disclosure, not a finding',
              not [v for v in res.violations if v.ref == 'J1' and 'seated' in v.message],
              str([v.message for v in res.violations if v.ref == 'J1']))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

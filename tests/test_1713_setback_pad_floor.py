#!/usr/bin/env python3
"""#1713: where the board's edge clearance on a part's pads holds the shape its
setback is graded on further from the edge than its setback limit, the seat's
reason says so, and offers `overhang_mm.max` only when the band's maximum is
what stopped it.

The part is a side-entry receptacle (mouth -y) declared `edge_receptacle` on
the north edge of a 40 x 30 board: two mounting pads stand 0.15 mm in front of
its F.Fab body, its courtyard 0.5 mm in front of the pads. At a 0.4 mm board
edge clearance the pads hold the body 0.555 mm from the edge, past the 0.50 mm
setback limit an edge receptacle is held to; the message named only the
setback and advised an `overhang_mm.max` that changes nothing (none of this
asks KiCad):

  1. edge clearance 0.4, band 0..0.5: no seat, and the reason names the pad
     floor, its value and source, where the pads would stand at the limit, and
     the way out (the vendor's drawing, a `.kicad_dru` rule, a larger
     `max_setback_mm`); it does not offer `overhang_mm.max`;
  2. band 0..1.0, the remedy the old message advised: the same reason;
  3. `max_setback_mm` 0.7 seats it, and so does an edge clearance of 0.2;
  4. on #1653's receptacle, whose band reads a courtyard drawn 1.00 mm past
     its body, a band maximum (0.2) that holds the body back still offers
     `overhang_mm.max`, and says nothing of the pad floor.

    python3 tests/test_1713_setback_pad_floor.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

from kicad_parser import parse_kicad_pcb  # noqa: E402
import pose_score  # noqa: E402
from placement import edge_seat, floorplan  # noqa: E402
from run_utils import evidence  # noqa: E402
from test_1653_edge_seat_setback_datum import ARMS  # noqa: E402
from test_1653_edge_seat_setback_datum import board as board_1653  # noqa: E402
from test_1653_edge_seat_setback_datum import intent as intent_1653  # noqa: E402
from test_1653_edge_seat_setback_datum import seat as seat_1653  # noqa: E402

#: no subprocess: `run_utils` is imported for `evidence` alone
RUN_ALL_FAST_OK = True

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def rect(x0, y0, x1, y1, lay, w):
    return (f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) '
            f'(stroke (width {w}) (type default)) (layer "{lay}"))\n')


def board(path):
    """The receptacle J1 at (20, 15): F.Fab front y -2.55, its mounting pads'
    front -2.7, its courtyard's front -3.2, in its own frame."""
    fp = ('  (footprint "Connector:SideEntry_Receptacle" (layer "F.Cu") (at 20 15)\n'
          '    (property "Reference" "J1" (at 0 -5) (layer "F.SilkS"))\n'
          + rect(-3, -2.55, 3, 2, 'F.Fab', 0.1) + rect(-4.5, -3.2, 4.5, 2.5, 'F.CrtYd', 0.05)
          + ''.join(f'    (pad "MP" smd rect (at {x} -2.2) (size 1.0 1.0) '
                    f'(layers "F.Cu" "F.Mask"))\n' for x in (-3.8, 3.8))
          + ''.join(f'    (pad "{i + 1}" smd rect (at {-1.5 + i} 1.5) (size 0.6 1.2) '
                    f'(layers "F.Cu" "F.Mask"))\n' for i in range(4))
          + '  )\n')
    cuts = ''.join(
        f'  (gr_line (start {a} {b}) (end {c} {d}) (stroke (width 0.05) (type default)) '
        f'(layer "Edge.Cuts"))\n'
        for a, b, c, d in ((0, 0, 40, 0), (40, 0, 40, 30), (40, 30, 0, 30), (0, 30, 0, 0)))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1713")\n  (net 0 "")\n'
                 + cuts + fp + ')\n')
    evidence(path, 'synthetic board')
    return path


def seat(path, edge_clr, **row):
    """`(seated, its pads' front from the edge or the reason)`."""
    r = dict({'ref': 'J1', 'edge': 'north', 'mating_face': '-y', 'class': 'edge_receptacle',
              'overhang_mm': {'min': 0.0, 'max': 0.5}}, **row)
    it = floorplan.intent_from_dict({'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                                     'units': 'mm', 'edge_connectors': [r]}, source_path='t')
    pcb = quiet(parse_kicad_pcb, path)
    st = quiet(pose_score.make_state, pcb, path, clearance=0.2, board_edge_clearance=edge_clr)
    out, _notes, unseated = quiet(edge_seat.seats, st, pcb, path, it, {'J1'})
    if 'J1' in out:
        s = out['J1']
        return True, s.lines[sorted(s.lines)[0]].coord - 2.7
    return False, unseated.get('J1', '')


def main():
    tmp = tempfile.mkdtemp(prefix='t1713_')
    try:
        p = board(os.path.join(tmp, 'b.kicad_pcb'))
        for band in (0.5, 1.0):
            ok, why = seat(p, 0.4, overhang_mm={'min': 0.0, 'max': band})
            check(f'edge clearance 0.4, band 0..{band}: not seated', not ok, f'seated, pads {why}')
            if ok:
                continue
            print(f'    {why}')
            check('...the reason names the setback limit',
                  'past its setback limit 0.50mm' in why, why)
            check("...and that its pad copper's edge clearance holds it, with the floor and "
                  "its source",
                  "its pad copper's edge clearance holds it there, 0.40mm from "
                  "the edge clearance given to this run" in why, why)
            check('...and where its pads would stand at the limit (0.405 - 0.055)',
                  'at its setback limit its pads would stand 0.35mm from the edge' in why, why)
            check("...and the way out: the vendor's drawing, a .kicad_dru rule, a larger "
                  "max_setback_mm",
                  "check the vendor's drawing" in why and '.kicad_dru edge_clearance rule' in why
                  and 'a larger max_setback_mm if the drawing allows' in why, why)
            check('...and it does not offer overhang_mm.max, which changes nothing',
                  'overhang_mm.max' not in why, why)
        ok, front = seat(p, 0.4, max_setback_mm=0.7)
        check('max_setback_mm 0.7: seated, its pads 0.405 mm from the edge',
              ok and abs(front - 0.405) < 0.002, str(front))
        ok, front = seat(p, 0.2)
        check('edge clearance 0.2: seated, its pads 0.205 mm from the edge',
              ok and abs(front - 0.205) < 0.002, str(front))
        # 4. #1653's receptacle (its band read on its courtyard, drawn 1.00 mm
        # past its body), whose band maximum holds the body back
        body = ARMS['B'][0]
        y, why = seat_1653(board_1653(os.path.join(tmp, 'b1653.kicad_pcb'), body),
                           intent_1653(**{'class': 'edge_receptacle',
                                          'overhang_mm': {'min': 0.0, 'max': 0.2}}))
        check('band max 0.2 on a courtyard drawn 1.00 mm past the body: not seated',
              y is None, f'seated at origin y {y}')
        if y is None:
            print(f'    {why}')
            check('...the band maximum holds it: the reason offers overhang_mm.max',
                  'past its setback limit 0.50mm' in why
                  and 'its declared overhang_mm.max 0.20mm holds it there' in why
                  and 'raise overhang_mm.max' in why, why)
            check("...and says nothing of the pad floor",
                  "pad copper" not in why and '.kicad_dru' not in why, why)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

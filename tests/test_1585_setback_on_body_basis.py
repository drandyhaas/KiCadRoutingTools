#!/usr/bin/env python3
"""#1585: the setback is asked on the shape it is measured on, and a hole
elsewhere on Edge.Cuts no longer changes what "flush" means.

Two gaps, measured on tigard's stock USB-C J1 (courtyard 0.50 mm past its
F.Fab body at the mouth):
  * `rule_edge_connector` asked `max_setback_mm` only while the COURTYARD did
    not overhang, but measured the setback on the drawn BODY
    (`edge_seat_rect`), so with `max_setback_mm: 0.1` a body 0.40 mm inside
    (courtyard 0.10 mm past) passed, and 0.10 mm further it failed;
  * one M3 hole drawn on Edge.Cuts mid-board made the outline "not
    rectangular", so every body read unmeasured, and the band and the seat
    fell back to the courtyard for every connector: J1 went from body flush
    to body 0.50 mm inside, still reading `overhang 0.0000mm ... pass`.

Pinned here, on boards built here (a USB-C-like receptacle, 0.5 mm of
courtyard past its body at the mouth, on a 60 x 40 board):
  1. a body 0.40 mm inside with its courtyard past the edge fails a declared
     `max_setback_mm` 0.1 (and the text line names the body's setback);
  2. with a hole on Edge.Cuts elsewhere the body is measured where the
     declared edge is straight under it, and the seat puts it flush, as on
     the plain rectangle;
  3. with a notch in the edge under the body it stays unmeasured, with the
     reason; where no position along the edge is straight (a kinked edge),
     the seat's note says the seat is on the courtyard and how far inside
     the body stands;
  4. on the holed board, pad copper past the outline is still named (the
     sampled path carries no gap; the pads are read by their boxes).

    python3 tests/test_1585_setback_on_body_basis.py
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
from placement.connector_geometry import geometry_for          # noqa: E402
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


LINE = '(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n'


def usb(x, y, pad_x=2.5):
    """Mouth at local -x: the body (F.Fab) x -4..3.5, the courtyard 0.5 mm
    past it all round, the pads a row at local x `pad_x`."""
    pads = ''.join(f'    (pad "{i + 1}" smd rect (at {pad_x} {-3 + i}) (size 1 0.6) '
                   f'(layers "F.Cu" "F.Mask") (net 1 "A"))\n' for i in range(7))
    return (f'  (footprint "Connector_USB:USB_C_Receptacle_Test" (layer "F.Cu") (at {x} {y})\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -4 -4.5) (end 3.5 4.5) (stroke (width 0.1) (type default)) '
            '(layer "F.Fab"))\n'
            '    (fp_rect (start -4.5 -5) (end 4 5) (stroke (width 0.05) (type default)) '
            '(layer "F.CrtYd"))\n' + pads + '  )\n')


CHIP = ('  (footprint "Package_SO:SOIC-8" (layer "F.Cu") (at 40 20)\n'
        '    (property "Reference" "U1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
        '    (fp_rect (start -3 -3) (end 3 3) (stroke (width 0.05) (type default)) '
        '(layer "F.CrtYd"))\n'
        '    (pad "1" smd rect (at -2 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A"))\n'
        '    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu" "F.Mask") (net 1 "A"))\n  )\n')
RECT = [(0, 0), (60, 0), (60, 40), (0, 40)]
HOLE = f'  (gr_circle (center 45 32) (end 46.6 32) {LINE}'
# a 2 mm notch in the west edge at y 18..22, under a body standing at y 20
NOTCHED = [(0, 0), (60, 0), (60, 40), (0, 40), (0, 22), (2, 22), (2, 18), (0, 18)]


def board(path, points, part, extra=''):
    cuts = ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) {LINE}'
                   for a, b in zip(points, points[1:] + points[:1]))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1585")\n  (net 0 "")\n'
                 '  (net 1 "A")\n' + cuts + extra + part + CHIP + ')\n')
    evidence(path, 'synthetic board')
    return path


def intent(**extra):
    return floorplan.intent_from_dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
        'edge_connectors': [dict({'ref': 'J1', 'edge': 'west',
                                  'overhang_mm': {'min': 0.0, 'max': 1.0}}, **extra)]},
        source_path='t')


TIGHT = {'class': 'edge_receptacle', 'max_setback_mm': 0.1}


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def grade(path, it):
    pcb = quiet(parse_kicad_pcb, path)
    res = quiet(floorplan.grade, it, pcb, path)
    text = floorplan.format_text(res)
    row = next((e for e in res.edge_connector_evidence if e['ref'] == 'J1'), {})
    line = next((ln for ln in text.splitlines() if ln.strip().startswith('J1 west: overhang')), '')
    return res, row, line


class Holder:
    pass


def body_row(path, pose=None):
    pcb = quiet(parse_kicad_pcb, path)
    return geometry_for(Holder(), pcb, path).measure('J1', 'west', pose)


def seat(path):
    """`(the drawn body's mm inside the west edge at the seat, note)`."""
    pcb = quiet(parse_kicad_pcb, path)
    state = quiet(pose_score.make_state, pcb, path, clearance=0.25, board_edge_clearance=0.2)
    out, notes, unseated = quiet(edge_seat.seats, state, pcb, path, intent(), {'J1'})
    note = next((n for n in notes if n.startswith('edge connector J1')), str(unseated))
    if 'J1' not in out:
        return None, note
    p = state.parts['J1']
    body = geometry_for(Holder(), pcb, path).rect('J1', (p.x, p.y, p.rot))[0]
    return round(body[0] - pcb.board_info.board_bounds[0], 2) + 0.0, note


def main():
    tmp = tempfile.mkdtemp(prefix='t1585_')
    try:
        # 1. body 0.40 mm inside, courtyard 0.10 mm past the west edge
        path = board(os.path.join(tmp, 'at440.kicad_pcb'), RECT, usb(4.4, 20))
        res, row, line = grade(path, intent(**TIGHT))
        hits = [v for v in res.violations if v.ref == 'J1' and 'seated' in v.message]
        check('a body 0.40 mm inside, courtyard past the edge, fails max_setback_mm 0.1',
              any(v.severity == 'error' and 'seated 0.40mm' in v.message for v in hits),
              str([v.message for v in res.violations if v.ref == 'J1']))
        check('...and the text line names the body\'s setback and the limit',
              '; setback 0.40mm [body:fab; limit 0.10mm]' in line, line)
        res, row, line = grade(path, intent())
        check('...while a row with no limit only discloses it',
              not [v for v in res.violations if v.ref == 'J1' and 'seated' in v.message]
              and row.get('setback_mm') == 0.0 and row.get('body_setback_mm') == 0.4
              and '; setback 0.00mm [courtyard; body 0.40mm; no limit declared]' in line,
              line)
        flush = board(os.path.join(tmp, 'at400.kicad_pcb'), RECT, usb(4.0, 20))
        res, row, line = grade(flush, intent(**TIGHT))
        check('a flush body passes the same declaration',
              not [v for v in res.violations if v.ref == 'J1' and 'seated' in v.message],
              str([v.message for v in res.violations if v.ref == 'J1']))

        # 2. one hole on Edge.Cuts elsewhere
        holed = board(os.path.join(tmp, 'holed.kicad_pcb'), RECT, usb(20, 20), HOLE)
        row = body_row(holed, (4.4, 20, 0.0))
        check('holed: the body is measured where the west edge is straight under it',
              row['body_measured'] and abs(row['body_setback_mm'] - 0.4) < 1e-6,
              str(row.get('body_unmeasured_reason')))
        inside, note = seat(holed)
        plain_inside, _n = seat(board(os.path.join(tmp, 'plain.kicad_pcb'), RECT, usb(20, 20)))
        check('holed: the seat puts the body flush, as on the plain rectangle',
              inside == 0.0 and plain_inside == 0.0, f'holed {inside}, plain {plain_inside}')
        check('holed: the note says nothing of a courtyard seat',
              'seated on its courtyard' not in note, note)

        # 3. a notch under the body
        notched = board(os.path.join(tmp, 'notched.kicad_pcb'), NOTCHED, usb(20, 20))
        row = body_row(notched, (4.0, 20, 0.0))
        check('notched under the body: unmeasured, with the reason',
              not row['body_measured']
              and 'not one straight Edge.Cuts run' in str(row.get('body_unmeasured_reason')),
              str(row))
        # ...and where no position has a straight edge under it (a west edge
        # kinked 0.3 mm at its middle), the seat is on the courtyard and says so
        kinked = board(os.path.join(tmp, 'kinked.kicad_pcb'),
                       [(0, 0), (60, 0), (60, 40), (0, 40), (0.3, 20)], usb(20, 20))
        inside, note = seat(kinked)
        check('kinked: the seat note says it is seated on its courtyard and how far '
              'inside the body stands',
              inside is not None and f'its body stands {inside:.2f}mm inside the west edge' in note
              and 'seated on its courtyard, not its drawn body' in note, f'{inside} {note}')

        # 4. pad copper past the outline on the holed board (pads at local x -4.6)
        off = board(os.path.join(tmp, 'copper.kicad_pcb'), RECT, usb(4.0, 20, pad_x=-3.6), HOLE)
        res, row, line = grade(off, intent())
        cu = [v for v in res.violations if v.ref == 'J1' and 'pad copper' in v.message]
        check('holed: pad copper past the outline is named on the body path',
              row.get('body_measured') and cu
              and abs(cu[0].measured['pad_copper_outside_mm'] - 0.1) < 0.02,
              f"{row.get('body_measured')} {[v.message for v in cu]}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

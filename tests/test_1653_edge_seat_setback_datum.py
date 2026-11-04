#!/usr/bin/env python3
"""#1653: the edge seat puts a receptacle on the shape its setback is graded
on, and the grade says where the part's own 3D model ends.

The part is the pad layout of a horizontal two-barrel power receptacle (its
body drawn in FRONT of its pins, so `connector_geometry` refuses the drawn
envelope as a marker and the band reads the courtyard), declared on the
north edge, on a 80 x 60 board. Two outline sets: arm A draws the courtyard
0.50 mm past the F.Fab body at the mouth, arm B 1.00 mm (what an import may
draw).

  1. class `edge_receptacle`: the seat puts the F.Fab body flush (it put the
     courtyard flush, leaving arm B's body 1.00 mm inside, which the grade
     then refused at its 0.50 mm seat tolerance), the grade at that pose
     names nothing, and under `legality_budget {oob_count: 0}` the part's
     courtyard past the edge is exempt; with no class the seat is unchanged
     (courtyard flush);
  2. a declared `overhang_mm.max` that keeps the body further in than the
     tolerance leaves the part unseated with the setback named, rather than
     seated where its grade fails it; one that keeps it within the
     tolerance seats it and the note says what holds the body inside;
  3. with the part's 3D model (synthetic, no KiCad: its raised body as
     raster cells over the F.Fab extent) the grade's line says
     `model end -1.00mm` / `-0.50mm`, and WARNs when the courtyard reads
     flush and the model stops short by more than the seat tolerance;
     without a model, or with the export's frame a guess, nothing is said.

    python3 tests/test_1653_edge_seat_setback_datum.py
"""
import contextlib
import io
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import numpy as np  # noqa: E402

from kicad_parser import parse_kicad_pcb  # noqa: E402
import pose_score  # noqa: E402
from placement import edge_seat, floorplan, heights  # noqa: E402
from run_utils import evidence  # noqa: E402

#: no subprocess: `run_utils` is imported for `evidence` alone (about 1 s)
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


def pad(n, x, y, s, d):
    return (f'    (pad "{n}" thru_hole circle (at {x} {y}) (size {s} {s}) '
            f'(drill {d}) (layers "*.Cu" "*.Mask"))\n')


PADS = (pad('', -3, -5, 1.5, 1.0) + pad('', 8, -5, 1.5, 1.0)
        + pad('1', 0, 0, 2.55, 1.7) + pad('2', 5, 0, 2.55, 1.7))
#: arm: (outlines, the F.Fab box, the courtyard's front), the part's frame
ARMS = {
    'A': (rect(-3.15, -14.05, 8.15, -4.0, 'F.Fab', 0.1)
          + rect(-4.65, -14.55, 9.65, 2.25, 'F.CrtYd', 0.05), (-3.15, -14.05, 8.15, -4.0), -14.55),
    'B': (rect(-4.15, -15.0, 9.15, -3.0, 'F.Fab', 0.1)
          + rect(-5.15, -16.0, 10.15, 2.27, 'F.CrtYd', 0.05), (-4.15, -15.0, 9.15, -3.0), -16.0),
}


def board(path, body, y=30.0):
    cuts = ''.join(
        f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
        f'(stroke (width 0.05) (type default)) (layer "Edge.Cuts"))\n'
        for a, b in (((0, 0), (80, 0)), ((80, 0), (80, 60)),
                     ((80, 60), (0, 60)), ((0, 60), (0, 0))))
    chips = ''.join(
        f'  (footprint "P:C" (layer "F.Cu") (at {cx} 50)\n'
        f'    (property "Reference" "{r}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
        + rect(-3, -3, 3, 3, 'F.CrtYd', 0.05)
        + '    (pad "1" smd rect (at -2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n'
          '    (pad "2" smd rect (at 2 0) (size 1 1) (layers "F.Cu" "F.Mask"))\n  )\n'
        for r, cx in (('U1', 20), ('U2', 60)))
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1653")\n  (net 0 "")\n' + cuts
                 + f'  (footprint "Connector_Power:Receptacle_2Barrel_Horizontal" '
                   f'(layer "F.Cu") (at 30 {y})\n'
                   '    (property "Reference" "J1" (at 0 -18) (layer "F.SilkS"))\n'
                 + body + PADS + '  )\n' + chips + ')\n')
    evidence(path, 'synthetic board')
    return path


def intent(**extra):
    row = dict({'ref': 'J1', 'edge': 'north', 'mating_face': '-y'}, **extra)
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': [row]}
    budget = row.pop('budget', None)
    if budget is not None:
        doc['legality_budget'] = budget
    return floorplan.intent_from_dict(doc, source_path='t')


def seat(path, it):
    """`(the J1 origin's y at its seat or None, the note or the reason)`."""
    pcb = quiet(parse_kicad_pcb, path)
    st = quiet(pose_score.make_state, pcb, path, clearance=0.25, board_edge_clearance=0.2)
    out, notes, unseated = quiet(edge_seat.seats, st, pcb, path, it, {'J1'})
    if 'J1' not in out:
        return None, unseated.get('J1', '')
    s = out['J1']
    rot = sorted(s.lines)[0]
    return s.lines[rot].coord, next((n for n in notes if n.startswith('edge connector J1')), '')


def grade(path, it):
    pcb = quiet(parse_kicad_pcb, path)
    res = quiet(floorplan.grade, it, pcb, path)
    line = next((ln.strip() for ln in floorplan.format_text(res).splitlines()
                 if ln.strip().startswith('J1 north: overhang')), '')
    return res, line


def model(fab, frame_sure=True):
    """J1's raised body as `Heights.bodies` holds it: raster cell centres
    over the F.Fab box (the barrels' extent), 3.6 mm tall; `frame_sure`
    False is an export whose frame is a guess (one modelled part fits a
    mirrored frame too)."""
    c = heights.CELL_MM
    xs = np.arange(fab[0] + c / 2, fab[2], c)
    ys = np.arange(fab[1] + c / 2, fab[3], c)
    pts = np.array([(x, y) for x in xs for y in ys], float)
    return heights.Heights(top={'J1': (3.6, 'model')},
                           bodies={'J1': (pts, np.full(len(pts), 3.6))},
                           frame_sure=frame_sure)


def main():
    tmp = tempfile.mkdtemp(prefix='t1653_')
    real = heights.for_board
    try:
        # 1. the seat's datum is the setback's
        for arm, (body, fab, cy_front) in ARMS.items():
            p = board(os.path.join(tmp, f's{arm}.kicad_pcb'), body)
            y, note = seat(p, intent(**{'class': 'edge_receptacle'}))
            check(f'{arm} receptacle: seated with its F.Fab body flush',
                  y is not None and abs(y + fab[1]) < 0.01, f'origin y {y}, note {note}')
            if y is None:
                continue
            g = board(os.path.join(tmp, f'g{arm}.kicad_pcb'), body, y=round(y, 4))
            res, line = grade(g, intent(**{'class': 'edge_receptacle',
                                           'budget': {'oob_count': 0}}))
            named = [(v.severity, v.message) for v in res.violations
                     if v.ref == 'J1' or 'leaving the board' in v.message]
            check(f'{arm} receptacle: its grade names nothing at that pose, oob_count 0 '
                  f'included (its courtyard past the edge is exempt)',
                  not named, str(named))
            y0, _n = seat(p, intent())
            check(f'{arm} no class: still seated with its courtyard flush',
                  y0 is not None and abs(y0 + cy_front) < 0.01, f'origin y {y0}')

        body, fab, _cy = ARMS['B']
        p = board(os.path.join(tmp, 'sB.kicad_pcb'), body)
        # 2. a band max the body cannot reach its tolerance under: unseated, named
        y, why = seat(p, intent(**{'class': 'edge_receptacle',
                                   'overhang_mm': {'min': 0.0, 'max': 0.2}}))
        check('B receptacle, overhang_mm.max 0.2: not seated where its grade fails it',
              y is None, f'seated at origin y {y}')
        check('...and the reason names the setback',
              'past its setback limit 0.50mm' in why and 'body:fab' in why, why)
        y, note = seat(p, intent(**{'class': 'edge_receptacle',
                                    'overhang_mm': {'min': 0.0, 'max': 0.6}}))
        check('B receptacle, overhang_mm.max 0.6: seated with its body within the tolerance',
              y is not None and 0.39 < (y + fab[1]) < 0.5, f'origin y {y}')
        # 0.405 mm: the band's max less the seat's BAND_SLACK_MM, 1.00 - 0.595
        check('...and the note says what holds the body inside (not an inboard line)',
              re.search(r'its drawn body \(body:fab\), the shape its setback is graded '
                        r'on, stands 0\.4[01]mm inside the north edge: its declared '
                        r'overhang_mm\.max 0\.60mm holds it there', note) is not None
              and 'only an inboard line' not in note, note)
        if y is not None:
            g = board(os.path.join(tmp, 'gB6.kicad_pcb'), body, y=round(y, 4))
            res, line = grade(g, intent(**{'class': 'edge_receptacle',
                                           'overhang_mm': {'min': 0.0, 'max': 0.6}}))
            check('...and its grade passes it',
                  not [v for v in res.violations if v.ref == 'J1'],
                  str([v.message for v in res.violations if v.ref == 'J1']))

        # 3. the model's end, beside the overhang
        for arm, end, warns in (('A', -0.50, False), ('B', -1.00, True)):
            body, fab, _cy = ARMS[arm]
            y, _n = seat(board(os.path.join(tmp, f'm{arm}.kicad_pcb'), body), intent())
            g = board(os.path.join(tmp, f'gm{arm}.kicad_pcb'), body, y=round(y, 4))
            heights.for_board = lambda *_a, _f=fab, **_k: model(_f)
            res, line = grade(g, intent())
            heights.for_board = real
            check(f'{arm} no class, courtyard flush: the line says the model end',
                  f'; model end {end:+.2f}mm' in line, line)
            row = next(e for e in res.edge_connector_evidence if e['ref'] == 'J1')
            check(f'{arm} ...and the evidence row carries it',
                  row.get('model_end_mm') is not None
                  and abs(row['model_end_mm'] - end) < 1e-6, str(row.get('model_end_mm')))
            warn = [v for v in res.violations if v.ref == 'J1' and '3D model stops' in v.message]
            check(f'{arm} ...and it {"WARNs" if warns else "does not warn"} (model '
                  f'{-end:.2f}mm short, seat tolerance 0.50mm)',
                  bool(warn) == warns and all(v.severity == 'warn' for v in warn),
                  str([(v.severity, v.message) for v in warn]))
            if arm == 'B':
                res, line = grade(g, intent())
                check('B without a model: no model end and no warning',
                      'model end' not in line
                      and not [v for v in res.violations if '3D model' in v.message], line)
                heights.for_board = lambda *_a, _f=fab, **_k: model(_f, frame_sure=False)
                res, line = grade(g, intent())
                heights.for_board = real
                check('B with the export frame a guess: no model end and no warning',
                      'model end' not in line
                      and not [v for v in res.violations if '3D model' in v.message], line)
    finally:
        heights.for_board = real
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

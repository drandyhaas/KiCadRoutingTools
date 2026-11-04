"""#1539: the `mountable` row counts "under" and prices the room on the shape
the verdict judges.

`mount_report` read a part as under a declared body when its OCCUPANCY box met
the body's rect, and priced its room from its whole height; the verdict
(`Heights.pair` -> `Cover.declared_area`) judges a part with a 3D model by its
model's cells inside the rect. A part whose courtyard reaches under the cover
while its body stays beside it read `YES` next to `least room -0.2mm`. The row
now asks `Heights.under_declared`, which shares the verdict's cell test
(`heights.cells_in_rect`), so a negative room never sits beside YES.

A synthetic board (no 3D library needed): J1 a socket whose declared body runs
along its +x axis, C1 a part whose courtyard is 2.0 mm long and whose model
body (injected into the board's heights, as a measurement would give it) is
0.8 mm long, 2.4 mm tall, under a 2.0 mm underside; R1 a part with no model,
0.5 mm tall, judged on its occupancy as before.

  1. the body's far edge over C1's courtyard but short of its model: YES, C1
     not under it (before: under it, least room -0.4 mm, beside YES);
  2. the edge over C1's first model cell: NO, C1 under it with that room;
  3. a part with no model is still under it on its occupancy, with its room;
  4. swept across the band, a YES never carries a room under the margin and
     every part refused is counted under it.

    python3 tests/test_1539_mountable_under_cells.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

import numpy as np  # noqa: E402

from placement import floorplan as fp  # noqa: E402
from placement import heights as hm  # noqa: E402

failures = []

#: C1 at x 16: courtyard x 15.0..17.0, model cells centred at 15.625..16.375
C1_X = 16.0
UNDERSIDE = 2.0


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _fp(ref, x, y, crt, pads, name='t:PART'):
    out = [f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))',
           f'    (fp_rect (start {crt[0]} {crt[1]}) (end {crt[2]} {crt[3]}) (stroke (width 0.05)'
           f' (type default)) (layer "F.CrtYd"))']
    for num, px, py, w, h in pads:
        out.append(f'    (pad "{num}" smd rect (at {px} {py}) (size {w} {h}) '
                   f'(layers "F.Cu") (net 0 ""))')
    out.append('  )')
    return '\n'.join(out) + '\n'


def board(td):
    sock = [(str(k + 1), 0.0, -2.5 + k, 0.6, 0.3) for k in range(6)]
    two = [('1', -0.4, 0, 0.3, 0.4), ('2', 0.4, 0, 0.3, 0.4)]
    parts = (_fp('J1', 10.0, 30.0, (-1.0, -3.0, 1.0, 3.0), sock, 't:SOCKET')
             + _fp('C1', C1_X, 30.0, (-1.0, -0.6, 1.0, 0.6), two)
             + _fp('R1', 13.0, 31.5, (-1.0, -0.6, 1.0, 0.6), two))
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n'
            '  (gr_rect (start 0 0) (end 40 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n' + parts + ')\n')
    path = os.path.join(td, 'b.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def report(path, length):
    from kicad_parser import parse_kicad_pcb
    doc = {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
           'min_reader': fp.MOUNT_READER,
           'covers': [{'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 6.0,
                       'length_mm': length, 'underside_mm': UNDERSIDE, 'removable': True,
                       'source': 'synthetic: a card'}],
           'heights': [{'ref': 'R1', 'height_mm': 0.5, 'source': 'synthetic'}]}
    it = fp.intent_from_dict(doc, '')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    base = hm.for_board(pcb, path)
    # C1's model, as a measurement gives it: cells on the 0.25 mm grid, its
    # body 0.8 x 0.5 mm inside a 2.0 x 1.2 mm courtyard, 2.4 mm tall
    xs = np.arange(-0.375, 0.376, hm.CELL_MM)
    ys = np.arange(-0.125, 0.126, hm.CELL_MM)
    pts = np.array([(x, y) for x in xs for y in ys], float)
    base.bodies['C1'] = hm._with_extent((pts, np.full(len(pts), 2.4)))
    base.top['C1'] = (2.4, 'model')
    h = hm.with_declared(base, it.height_declarations(), pcb, path)
    return fp.mount_report(it, pcb, path, heights=h)[0]


def main():
    td = tempfile.mkdtemp(prefix='krt1539_')
    try:
        path = board(td)
        first_cell = C1_X - 0.375 - 10.0         # along J1's axis: 5.625 mm

        print("1. the far edge over C1's courtyard, short of its model")
        m = report(path, first_cell - 0.2)
        lr = m.get('least_room') or {}
        check('mountable YES', m['mountable'] is True, json.dumps(m['why']))
        check('C1 is not refused', not [b for b in m['blocked'] if b['ref'] == 'C1'],
              json.dumps(m['blocked']))
        check('C1 is not counted under it, so no negative room sits beside YES',
              lr.get('ref') != 'C1' and (lr.get('room_mm') is None or lr['room_mm'] >= 0),
              json.dumps(lr))

        print("2. the far edge over C1's first model cell")
        m = report(path, first_cell + 0.05)
        lr = m.get('least_room') or {}
        check('mountable NO, C1 refused', m['mountable'] is False
              and [b for b in m['blocked'] if b['ref'] == 'C1'], json.dumps(m['blocked']))
        check('C1 under it with the room the verdict judged: 2.0 - 2.4 on its model',
              lr.get('ref') == 'C1' and lr.get('room_mm') == -0.4 and lr.get('on') == 'model',
              json.dumps(lr))

        print('3. a part with no model: under it on its occupancy, with its room')
        m = report(path, 4.0)
        lr = m.get('least_room') or {}
        check('R1 under it (its courtyard reaches y 30.9..32.1, the body 27..33)',
              m['under'] == 1 and lr.get('ref') == 'R1' and lr.get('room_mm') == 1.5
              and lr.get('on') == 'occupancy', json.dumps({'under': m['under'], 'lr': lr}))
        check('...and it fits: YES', m['mountable'] is True, json.dumps(m['why']))

        print('4. swept across the band: the row and the verdict agree')
        bad = []
        for k in range(0, 31):
            L = round(first_cell - 0.75 + 0.05 * k, 3)
            m = report(path, L)
            lr = m.get('least_room') or {}
            refused = {b['ref'] for b in m['blocked']}
            if m['mountable'] and lr.get('room_mm') is not None and lr['room_mm'] < hm.MARGIN_MM:
                bad.append(f'L {L}: YES with least room {lr["room_mm"]} ({lr.get("ref")})')
            if 'C1' in refused and lr.get('ref') != 'C1':
                bad.append(f'L {L}: C1 refused but not the part with the least room')
        check('no YES with a room under the margin; every refusal of C1 counted under it',
              not bad, '; '.join(bad[:4]))
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

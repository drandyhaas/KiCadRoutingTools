"""`pad_contacts` skips a candidate pad whose copper disc lies beyond the
contact distance (a board's largest pad sizes its grid, so one shield or
thermal slug fills every cell with pads the exact checks reject). The skip
must never drop a contact:

1. The disc holds the copper: for rect, roundrect, oval, circle, a tilted
   (`rect_rotation`) and a custom-polygon pad, every point just outside the
   disc is outside the pad's copper as `point_to_pad_distance` measures it.
2. On a routed corpus board, the rows (and their order) at clearance 0, 0.1
   and 0.2 mm equal the rows with the skip disabled.

Run:
    python3 tests/test_pad_contact_skip.py
"""
import json
import math
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('', 'py_router'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from check_drc import point_to_pad_distance  # noqa: E402
from kicad_parser import Pad, parse_kicad_pcb  # noqa: E402
import pad_contact  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def _pad(shape, sx, sy, rratio=0.0, tilt=0.0, polygons=None):
    return Pad(component_ref='U1', pad_number='1', global_x=10.0, global_y=5.0,
               local_x=0.0, local_y=0.0, size_x=sx, size_y=sy, shape=shape,
               layers=['F.Cu'], net_id=1, net_name='A',
               roundrect_rratio=rratio, rect_rotation=tilt, polygons=polygons)


def test_disc_holds_copper():
    print('disc holds the copper')
    pads = {
        'rect': _pad('rect', 3.0, 1.0),
        'roundrect': _pad('roundrect', 1.2, 0.6, rratio=0.25),
        'oval': _pad('oval', 2.0, 0.8),
        'circle': _pad('circle', 1.5, 1.5),
        'tilted rect': _pad('rect', 3.0, 1.0, tilt=30.0),
        'custom polygon': _pad('custom', 1.0, 1.0, polygons=[
            [(9.0, 4.0), (13.5, 4.2), (13.0, 6.5), (9.5, 7.0)]]),
    }
    for name, p in pads.items():
        cx, cy, r = pad_contact._pad_disc(p)
        worst = min(point_to_pad_distance(cx + (r + 1e-4) * math.cos(a),
                                          cy + (r + 1e-4) * math.sin(a), p)
                    for a in (k * math.pi / 180.0 for k in range(360)))
        check(f'{name}: no copper outside its disc', worst > 0.0,
              f'min distance just outside the disc {worst}')


def _rows_key(rows):
    return json.dumps([{k: (id(v) if k == 'item' else v) for k, v in r.items()}
                       for r in rows])


def test_rows_unchanged():
    print('rows equal the rows without the skip (routed_output)')
    pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb'))
    real = pad_contact._pad_disc
    for cl in (0.0, 0.1, 0.2):
        fast = pad_contact.pad_contacts(pcb, clearance=cl)
        pad_contact._pad_disc = lambda p: (p.global_x, p.global_y, math.inf)
        try:
            full = pad_contact.pad_contacts(pcb, clearance=cl)
        finally:
            pad_contact._pad_disc = real
        check(f'clearance {cl}: {len(fast)} rows, same rows in the same order',
              _rows_key(fast) == _rows_key(full), f'{len(fast)} vs {len(full)}')
    check('the board has contact rows at 0.2 mm (the test measures something)',
          len(pad_contact.pad_contacts(pcb, clearance=0.2)) > 0)


if __name__ == '__main__':
    test_disc_holds_copper()
    test_rows_unchanged()
    if FAILS:
        print(f'\nFAILED: {len(FAILS)} failure(s)')
        sys.exit(1)
    print('\nALL CHECKS PASSED')

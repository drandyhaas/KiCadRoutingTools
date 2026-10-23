#!/usr/bin/env python3
"""#1325: place_simple's edge seat finds the seats that exist, and says why
when there is none.

`edge_seat.seats` answered "no room along the <edge> edge" for three
different faults (gp14 M08), two of them seats that exist:

  (a) `_pad_box` counted a pad on no copper layer as copper. daplink's Harwin
      M20 headers draw each pin as a `pad "" smd` on F.SilkS, 6 mm long, so
      the copper limit held J2/J3 0.55 mm inside their edge and their 7.0 mm
      band never came within reach: every place_simple seed carried the same
      four edge_connector errors. The box is now the copper the seat's own
      `copper_clear` grades (`connector_geometry.pad_boxes`).
  (b) `_seat_coord` took the edge from the board's bounding box. On
      CM5_MINIMA (east bounds 136.25 at its corner ears, its straight edge at
      132.90) the copper limit never bound, and every position failed the
      copper check 5 um short. The limit is now found against the outline AT
      THE SEAT, by the same check.
  (c) A courtyard longer than its edge (sonde_xilinx's DB25 J1, 54.1 mm on
      43.18 mm) reads past the outline at both ends wherever it stands, so the
      band never came down to its target. That is a real fault, and the reason
      now says so with the reading; any other refusal names the conjuncts the
      tested positions failed.

A positive band minimum is aimed 5 um inside, as the copper limit is: seated
exactly at it, J3 wrote back as 7.0000 under its declared 7.0.

Checks, on small boards built here:
  1. (a) a header whose silk pins reach past its copper seats at its
     7.0-7.4 band, and reads inside it.
  2. (b) a board with ears: a courtyard-less switch seats on the straight
     edge, its copper the edge clearance (plus slack) inside it.
  3. (c) a 40 mm courtyard on a 30 mm edge: unseated, the reason naming the
     reading it never leaves and the two lengths.
  4. An edge a locked part fills: unseated, the reason naming the part.
  5. A declared along-edge window too narrow for the pads: unseated, the
     reason naming the window.

    python3 tests/test_1325_edge_seat_refusals.py
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
CLEARANCE, EDGE = 0.25, 0.55


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def pad(num, x, y, w, h, layers='"F.Cu" "F.Mask"'):
    return f'    (pad "{num}" smd rect (at {x} {y}) (size {w} {h}) (layers {layers}))\n'


def footprint(ref, name, at, body, locked=False):
    return (f'  (footprint "{name}"{" (locked yes)" if locked else ""} (layer "F.Cu") '
            f'(at {at})\n'
            f'    (property "Reference" "{ref}" (at 0 0 0) (layer "F.SilkS"))\n'
            + body + '  )\n')


def courtyard(x0, y0, x1, y1):
    return f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (layer "F.CrtYd"))\n'


def board(path, outline, footprints):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('(kicad_pcb (version 20241229) (generator "t1325")\n  (net 0 "")\n'
                 + outline + ''.join(footprints) + ')\n')
    return evidence(path, 'synthetic board')


def rect_outline(w, h):
    return f'  (gr_rect (start 0 0) (end {w} {h}) (layer "Edge.Cuts"))\n'


def poly_outline(points):
    return ''.join(f'  (gr_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) '
                   f'(layer "Edge.Cuts"))\n'
                   for a, b in zip(points, points[1:] + points[:1]))


def seat(path, rows):
    """`(state, seats, notes, unseated)` for `rows` (edge_connectors[])."""
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        state = pose_score.make_state(pcb, path, clearance=CLEARANCE,
                                      board_edge_clearance=EDGE)
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
            'units': 'mm', 'edge_connectors': rows})
        movable = {r for r, p in state.parts.items() if not p.locked}
        out, notes, unseated = edge_seat.seats(state, pcb, path, intent, movable)
    return state, out, notes, unseated


def main():
    tmp = tempfile.mkdtemp(prefix='t1325_')
    try:
        # 1. (a) silk pins past the copper.
        body = (courtyard(-7.0, -6.5, 7.3, 6.5)
                + ''.join(pad(str(i + 1), -5.5, -5.08 + 2.54 * i, 2.5, 1.0)
                          for i in range(5))
                + pad('', 3.9, -5.08, 6.0, 0.76, '"F.SilkS"'))
        path = board(os.path.join(tmp, 'silk.kicad_pcb'), rect_outline(40, 30),
                     [footprint('J3', 'Connector_Harwin:Harwin_M20-89005xx_1x05_'
                                'P2.54mm_Horizontal', '20 15 0', body)])
        state, out, _notes, unseated = seat(path, [
            {'ref': 'J3', 'edge': 'east',
             'overhang_mm': {'min': 7.0, 'max': 7.4}}])
        check('(a) J3 seats on its edge', 'J3' in out and not unseated,
              str(unseated))
        if 'J3' in out:
            p = state.parts['J3']
            from placement.legality import BoardOutlineGate
            band = BoardOutlineGate(state.pcb_data.board_info,
                                    0.0).rect_outside_amount(p.rect())
            check('(a) at its band, read on the courtyard at zero margin '
                  '(#1301)', 7.0 < band <= 7.4, f'{band:.4f}')

        # 2. (b) ears: bounds east at 43, the straight edge at 40.
        ears = [(0, 0), (43, 0), (43, 4), (40, 5), (40, 25), (43, 26), (43, 30), (0, 30)]
        body = pad('1', -1.0, -1.5, 1.0, 1.0) + pad('2', -1.0, 1.5, 1.0, 1.0)
        path = board(os.path.join(tmp, 'ears.kicad_pcb'), poly_outline(ears),
                     [footprint('SW1', 'Lib:SW_TS-1010A-B', '20 15 0', body)])
        state, out, _notes, unseated = seat(path, [
            {'ref': 'SW1', 'edge': 'east', 'mating_face': '+x'}])
        check('(b) the switch seats on the straight edge, not the ears',
              'SW1' in out and not unseated, str(unseated))
        if 'SW1' in out:
            pb = state.parts['SW1'].padbox()
            gap = 40.0 - pb[2]
            check('(b) its copper the edge clearance inside the straight edge',
                  EDGE + edge_seat.COPPER_SLACK_MM - 1e-6 <= gap
                  <= EDGE + edge_seat.COPPER_SLACK_MM + 0.02, f'{gap:.4f}')

        # 3. (c) a courtyard longer than its edge.
        body = (courtyard(-3, -20, 3, 20) + pad('1', 0, -10, 1, 1)
                + pad('2', 0, 10, 1, 1))
        path = board(os.path.join(tmp, 'long.kicad_pcb'), rect_outline(60, 30),
                     [footprint('J1', 'Connector_Dsub:DSUB-25_Male_EdgeMount',
                                '30 15 0', body)])
        _state, out, _notes, unseated = seat(path, [
            {'ref': 'J1', 'edge': 'west', 'mating_face': '-x'}])
        why = unseated.get('J1', '')
        check('(c) unseated', 'J1' not in out and bool(why), why)
        check('(c) the reason gives the reading it never leaves',
              'reads at least 10.00mm wherever it stands' in why, why)
        check('(c) and the two lengths',
              'its courtyard is 40.00mm along a 30.00mm edge' in why, why)

        # 4. An edge a locked part fills.
        wall = footprint('H1', 'Lib:Wall', '38 15 0', courtyard(-2, -15, 2, 15)
                         + pad('1', 0, 0, 1, 1), locked=True)
        conn = footprint('J2', 'Connector_USB:USB_C_Receptacle', '15 15 0',
                         courtyard(-4, -4, 4, 4) + pad('1', -2, 0, 1, 1)
                         + pad('2', 2, 0, 1, 1))
        path = board(os.path.join(tmp, 'wall.kicad_pcb'), rect_outline(40, 30),
                     [wall, conn])
        _state, out, _notes, unseated = seat(path, [{'ref': 'J2', 'edge': 'east'}])
        why = unseated.get('J2', '')
        check('a filled edge: unseated', 'J2' not in out and bool(why), why)
        check('and the reason names the part in the way',
              'refused by the declared-pose check against H1' in why, why)

        # 5. A declared along-edge window its pads cannot sit in: the first
        # 2% of a 40 mm edge, pads 2.5 mm each side of the centre.
        path = board(os.path.join(tmp, 'window.kicad_pcb'), rect_outline(40, 30),
                     [conn.replace('J2', 'J4')])
        _state, out, _notes, unseated = seat(path, [
            {'ref': 'J4', 'edge': 'north', 'along_edge_band': {'from': 0.0, 'to': 0.02}}])
        why = unseated.get('J4', '')
        check('a window too narrow for its pads: unseated, and the reason says so',
              'J4' not in out and 'no position inside its declared along_edge_band' in why,
              why)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

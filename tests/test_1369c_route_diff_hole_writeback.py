#!/usr/bin/env python3
"""route_diff writes back the copper-to-hole floor it HELD (#1369 part c).

route_diff holds NPTH walls at the board's own min_hole_clearance
(obstacle_map.resolve_hole_clearance), but its writeback passed no hole
clearance, so fix_kicad_drc_settings set rules.min_hole_clearance to the
copper clearance -- 0.25 -> 0.175 on a board whose route_diff copper KiCad
grades clean at 0.25 -- and the delivered fill then followed the lower value.

It now writes the held value, lowered only to what the step's OWN copper
needs (fix_kicad_drc_settings.step_copper_to_hole: a P via's drill beside the
N track can sit under the held value, since a plated hole is held at copper
clearance), and never below the copper clearance it wrote before.

Rows:
  A. a route_diff run on a board declaring 0.25 keeps min_hole_clearance 0.25
  B. step_copper_to_hole measures own-track-to-foreign-hole and
     own-via-drill-to-foreign-copper, skips same-net pairs, and skips a pair
     whose copper already violates the copper clearance (KiCad reports that
     one as a clearance violation, not a hole one)
  C. held_hole_clearance: the held value, lowered to the step's own need,
     never below the copper clearance; None when nothing is declared

    python3 tests/test_1369c_route_diff_hole_writeback.py
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

import run_utils                                                   # noqa: E402

BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run32', 'placed_v2.kicad_pcb')
FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def near(a, b, tol=1e-6):
    return a is not None and b is not None and abs(float(a) - float(b)) <= tol


def rules_of(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro', encoding='utf-8') as f:
        return json.load(f)['board']['design_settings']['rules']


def test_route_diff_keeps_the_held_hole_floor():
    print('\n-- A. route_diff on a board declaring min_hole_clearance 0.25 --')
    run_utils.evidence(BOARD)
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        subprocess.run([sys.executable, '-X', 'utf8',
                        run_utils.tool('copy_board.py'), BOARD, src],
                       check=True, capture_output=True, text=True)
        held = rules_of(src).get('min_hole_clearance')
        check('the fixture declares a hole clearance above its copper clearance',
              near(held, 0.25), held)
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            run_utils.tool('route_diff.py'), src, out,
                            '--nets', '/USB_*'],
                           capture_output=True, text=True, timeout=600)
        log = r.stdout + r.stderr
        ok = r.returncode == 0 and os.path.isfile(out)
        check('route_diff ran', ok, '' if ok else f'rc={r.returncode} {log[-400:]}')
        if not os.path.isfile(out):
            return
        got = rules_of(out).get('min_hole_clearance')
        check('min_hole_clearance is the held 0.25, not the copper clearance',
              near(got, 0.25), got)
        check('the writeback says what it wrote and why',
              'Copper-to-hole floor written at 0.25 mm' in log)
        check('no min_hole_clearance write is listed',
              'rules.min_hole_clearance:' not in log)


def _via(x, y, net, size=0.6, drill=0.3):
    from kicad_parser import Via
    return Via(x=x, y=y, size=size, drill=drill, layers=['F.Cu', 'B.Cu'], net_id=net)


def _seg(x1, y1, x2, y2, net, layer='F.Cu', width=0.2):
    from kicad_parser import Segment
    return Segment(start_x=x1, start_y=y1, end_x=x2, end_y=y2, width=width,
                   layer=layer, net_id=net)


def _npth(x, y, drill):
    return NS(drill=drill, pad_type='np_thru_hole', layers=['*.Mask'],
              global_x=x, global_y=y, rotation=0.0, net_id=0, size_x=drill,
              size_y=drill, shape='circle')


def _board(segs=(), vias=(), pads=()):
    return NS(segments=list(segs), vias=list(vias),
              footprints={'H1': NS(pads=list(pads))},
              board_info=NS(copper_layers=['F.Cu', 'B.Cu']))


def test_step_copper_to_hole():
    print('\n-- B. step_copper_to_hole --')
    try:
        from fix_kicad_drc_settings import step_copper_to_hole
    except ImportError as e:
        check('step_copper_to_hole exists', False, e)
        return
    # P via (net 1, drill 0.3, copper 0.6) at the origin; N track (net 2,
    # width 0.2) whose centreline is 0.52 from the via centre: copper-to-
    # copper 0.12, copper-to-hole 0.27.
    v = _via(0.0, 0.0, 1)
    n = _seg(-1.0, 0.52, 1.0, 0.52, 2)
    m = step_copper_to_hole(_board(), [n], [v], 0.3, clearance=0.1)
    check('a P via drill beside the N track is measured (0.27)', near(m, 0.27), m)
    m = step_copper_to_hole(_board(vias=[v]), [n], [], 0.3, clearance=0.1)
    check('...from the track side too, the via already on the board',
          near(m, 0.27), m)
    m = step_copper_to_hole(_board(), [n], [v], 0.25, clearance=0.1)
    check('nothing below the held value -> None', m is None, m)
    m = step_copper_to_hole(_board(), [n], [v], 0.3, clearance=0.15)
    check('a pair already under the copper clearance is not a hole pair',
          m is None, m)
    same = _seg(-1.0, 0.52, 1.0, 0.52, 1)
    m = step_copper_to_hole(_board(), [same], [v], 0.3, clearance=0.1)
    check('a same-net pair is not graded', m is None, m)
    other_layer = _via(0.0, 0.0, 1)
    other_layer.via_type = 'micro'
    other_layer.layers = ['B.Cu', 'In1.Cu']
    m = step_copper_to_hole(_board(vias=[other_layer]), [n], [], 0.3, clearance=0.1)
    check('a via that does not span the track layer is not graded', m is None, m)
    # An NPTH hole (drill 1.0 at x=5) and a foreign track 0.22 off its wall:
    # an NPTH is foreign to every net, and has no copper to violate.
    hole = _npth(5.0, 0.0, 1.0)
    t = _seg(5.0 + 0.5 + 0.22 + 0.1, -1.0, 5.0 + 0.5 + 0.22 + 0.1, 1.0, 0)
    m = step_copper_to_hole(_board(pads=[hole]), [t], [], 0.25, clearance=0.1)
    check('an own track 0.22 off an NPTH wall is measured', near(m, 0.22), m)


def test_held_hole_clearance():
    print('\n-- C. held_hole_clearance --')
    try:
        from fix_kicad_drc_settings import held_hole_clearance
    except ImportError as e:
        check('held_hole_clearance exists', False, e)
        return
    check('own copper clears it -> the held value',
          near(held_hole_clearance(0.25, 0.1, None), 0.25))
    check('own copper at 0.195 -> 0.195',
          near(held_hole_clearance(0.25, 0.1, 0.195), 0.195))
    check('never below the copper clearance',
          near(held_hole_clearance(0.25, 0.1, 0.05), 0.1))
    check('nothing declared -> None (the copper clearance, as before)',
          held_hole_clearance(0.0, 0.1, None) is None)


if __name__ == '__main__':
    test_route_diff_keeps_the_held_hole_floor()
    test_step_copper_to_hole()
    test_held_hole_clearance()
    print(f"\n{'FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL PASS'}")
    sys.exit(1 if FAILS else 0)

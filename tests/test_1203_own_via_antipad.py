#!/usr/bin/env python3
"""#1203: check_impedance does not count a net's own via antipad as a
reference-plane crossing.

A segment that ends on its own via runs over that via's antipad on the
reference layer for about via radius + clearance (0.225 + 0.2 = 0.425 mm),
so it always passed the 0.30 mm void filter: on CM5_MINIMA_3's own board 40
of 41 impedance nets read "with a crossing", 104 of 105 crossings within
0.6 mm of a via of the same net. board_score adds nets_with_crossing to
`blocking`, so no board whose impedance nets change layers could reach 0.

Synthetic 4-layer board, GND planes on In1 and In2:
  /SIG     F.Cu -> its own via -> B.Cu (a layer change, nothing else);
  /D_P,/D_N side-by-side vias, the two antipads merged;
  /E_P,/E_N  /E_N passes 0.35 mm from /E_P's via on its way to its own: the
           PARTNER's antipad, exempt as well;
  /SLOT    F.Cu across a 0.8 mm gap between two In1 GND zones, far from any
           via -- a real slot, which must still be reported.

Checks:
  1. /SIG and the pair report no crossing, and own_via_antipad_runs > 0.
  2. /SLOT still reports its void crossing on In1.
  3. A via of ANOTHER net is not exempted: /SLOT2 runs over /SIG's via
     antipad on In1 and keeps that crossing.

    python3 tests/test_1203_own_via_antipad.py
"""
import contextlib
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import check_impedance                                         # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402


def zone(net, layer, x0, y0, x1, y1):
    return f"""  (zone (net "{net}") (layer "{layer}") (uuid "z-{layer}-{x0}")
    (hatch edge 0.5) (connect_pads yes (clearance 0.2)) (min_thickness 0.1)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) (island_removal_mode 0))
    (polygon (pts (xy {x0} {y0}) (xy {x1} {y0}) (xy {x1} {y1}) (xy {x0} {y1}))))
"""


def seg(net, layer, x0, y0, x1, y1, w=0.2):
    return (f'  (segment (start {x0} {y0}) (end {x1} {y1}) (width {w}) '
            f'(layer "{layer}") (net "{net}"))\n')


def via(net, x, y):
    return (f'  (via (at {x} {y}) (size 0.45) (drill 0.3) '
            f'(layers "F.Cu" "B.Cu") (net "{net}"))\n')


BOARD = ("""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)
          (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (net 2 "/SIG")
  (net 3 "/D_P")
  (net 4 "/D_N")
  (net 5 "/SLOT")
  (net 6 "/SLOT2")
  (net 7 "/E_P")
  (net 8 "/E_N")
  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
"""
    + zone('GND', 'In1.Cu', 0.5, 0.5, 30, 29.5)
    + zone('GND', 'In1.Cu', 30.8, 0.5, 39.5, 29.5)
    + zone('GND', 'In2.Cu', 0.5, 0.5, 39.5, 29.5)
    # /SIG: F.Cu to its via at (10, 5), then B.Cu on.
    + seg('/SIG', 'F.Cu', 4, 5, 10, 5) + via('/SIG', 10, 5)
    + seg('/SIG', 'B.Cu', 10, 5, 16, 5)
    # the pair: vias at (10, 10 -/+ 0.35), antipads merged.
    + seg('/D_P', 'F.Cu', 4, 9.65, 10, 9.65) + via('/D_P', 10, 9.65)
    + seg('/D_P', 'B.Cu', 10, 9.65, 16, 9.65)
    + seg('/D_N', 'F.Cu', 4, 10.35, 10, 10.35) + via('/D_N', 10, 10.35)
    + seg('/D_N', 'B.Cu', 10, 10.35, 16, 10.35)
    # /E_N passes 0.35 mm from its PARTNER's via on its way to its own.
    + seg('/E_P', 'F.Cu', 16, 15, 20, 15) + via('/E_P', 20, 15)
    + seg('/E_P', 'B.Cu', 20, 15, 26, 15)
    + seg('/E_N', 'F.Cu', 16, 15.35, 21, 15.35) + via('/E_N', 21, 15.35)
    + seg('/E_N', 'B.Cu', 21, 15.35, 26, 15.35)
    # /SLOT crosses the In1 gap at x = 30..30.8, nowhere near a via.
    + seg('/SLOT', 'F.Cu', 26, 20, 35, 20)
    # /SLOT2 passes 0.25 mm from /SIG's via on F.Cu: another net's antipad.
    + seg('/SLOT2', 'F.Cu', 10.25, 2, 10.25, 8)
    + ")\n")
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def main():
    import tempfile
    with tempfile.TemporaryDirectory(prefix='t1203_') as d:
        path = os.path.join(d, 'b.kicad_pcb')
        open(path, 'w').write(BOARD)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        rep = check_impedance.analyze_impedance(pcb, board_name='t1203')
    nets = {n.net_name: n for n in rep.nets}
    check('precondition: every signal net was analysed',
          set(nets) >= {'/SIG', '/D_P', '/D_N', '/E_P', '/E_N', '/SLOT', '/SLOT2'},
          str(sorted(nets)))
    for name in ('/SIG', '/D_P', '/D_N', '/E_P', '/E_N'):
        n = nets.get(name)
        check(f'{name}: its own via antipad is not a crossing',
              n is not None and not n.crossings
              and getattr(n, 'own_via_antipad_runs', 0) > 0,
              f'crossings {[c.describe() for c in n.crossings][:2]}, '
              f"antipad runs {getattr(n, 'own_via_antipad_runs', None)}"
              if n else 'missing')
    slot = nets.get('/SLOT')
    check('/SLOT: the real In1 slot is still reported',
          slot is not None and any(c.kind == 'void' and c.ref_layer == 'In1.Cu'
                                   and c.length_mm >= 0.6 for c in slot.crossings),
          str([c.describe() for c in slot.crossings]) if slot else 'missing')
    slot2 = nets.get('/SLOT2')
    check("/SLOT2: another net's via antipad keeps its crossing",
          slot2 is not None and slot2.crossings
          and not getattr(slot2, 'own_via_antipad_runs', 0),
          str([c.describe() for c in slot2.crossings]) if slot2 else 'missing')
    d = check_impedance.report_to_dict(rep)
    check('the JSON carries own_via_antipad_runs',
          d['totals'].get('own_via_antipad_runs', 0) > 0
          and all('own_via_antipad_runs' in n for n in d['nets']),
          str(d['totals'].get('own_via_antipad_runs')))
    check('nets_with_crossing counts only the slot nets',
          d['totals']['nets_with_crossing'] == 2, str(d['totals']['nets_with_crossing']))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

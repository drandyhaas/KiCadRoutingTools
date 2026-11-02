#!/usr/bin/env python3
"""#1581 end to end, graded by KiCad: route.py keeps a GND pad's only real
connection, and a KiCad-only open names its stranded pad.

The board is the issue's repro (tests/test_1581_graze_prune_pour_only_join.py
builds it): U1's GND pad ringed by four via-in-pad pins whose 45-degree gaps
are under the zone's min_thickness, with the earlier step's fanout stub out
through one gap to a via in the flood. Before the fix route.py's graze prune
removed the stub ("Graze prune: removed 1 grazing segment(s)"), the in-run
oracle could not put a connection back, and GND shipped open with
check_connected naming no pad, so board_score's `stranded_pads` was [].

1. route.py, as the issue ran it: the stub survives and check_connected,
   whose KiCad refill cross-check RAN, grades the output connected.
2. The same board with no stub at all (the open the prune used to make):
   check_connected names U1's pad under "Disconnected pads". Since #1608 the
   fill model sees that open itself, and KiCad's cross-check agrees.
3. A KiCad-only open (the fill model cannot see it: a thermal pad whose four
   spokes are all cut by foreign bars, joined in the model through the open
   diagonals) names its stranded pad, read off KiCad's own fill rather than
   the link's zone-corner anchor.

Needs kicad-cli and KiCad's bundled python (pcbnew); exits 77 (SKIP) without
them.

    python3 -X utf8 tests/test_1581_route_keeps_pour_only_join_kicad.py
"""
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from run_utils import check, evidence, tool                    # noqa: E402
from test_1581_graze_prune_pour_only_join import (             # noqa: E402
    write_board, CX, CY)

SKIP_EXIT = 77


def thermal_board(path):
    """G1, a 1 mm GND pad on a thermal-relief pour (gap 0.5, spokes 0.3),
    with a foreign 0.1 mm bar across each of its four spokes 0.9 mm out; G2
    in the open flood. KiCad leaves G1 unconnected (checked with kicad-cli
    10.0.3); the fill model, which does not model thermal relief, joins it
    to the flood through the open diagonals."""
    def fp(ref, x, y, net, nm, size=1.0):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr smd)\n    (pad "1" smd rect (at 0 0) (size {size} {size}) '
                f'(layers "F.Cu") (net {net} "{nm}"))\n  )\n')
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n (net 1 "GND")\n'
         ' (net 2 "/X")\n')
    b += fp('G1', CX, CY, 1, 'GND') + fp('G2', 105, 105, 1, 'GND')
    for x1, y1, x2, y2 in ((0.9, -0.4, 0.9, 0.4), (-0.9, -0.4, -0.9, 0.4),
                           (-0.4, 0.9, 0.4, 0.9), (-0.4, -0.9, 0.4, -0.9)):
        b += (f'  (segment (start {CX + x1} {CY + y1}) (end {CX + x2} {CY + y2}) '
              f'(width 0.1) (layer "F.Cu") (net 2))\n')
    b += fp('X1', 130, 125, 2, '/X', 0.5)
    b += ('  (zone (net 1) (net_name "GND") (layer "F.Cu") (hatch edge 0.5) '
          '(connect_pads (clearance 0.1))\n'
          '    (min_thickness 0.1) (filled_areas_thickness no) '
          '(fill yes (thermal_gap 0.5) (thermal_bridge_width 0.3))\n'
          '    (polygon (pts (xy 100.5 100.5) (xy 139.5 100.5) (xy 139.5 129.5) '
          '(xy 100.5 129.5))))\n')
    for x1, y1, x2, y2 in ((100, 100, 140, 100), (140, 100, 140, 130),
                           (140, 130, 100, 130), (100, 130, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    return path


def main():
    from kicad_oracle import find_kicad_cli
    from kicad_exact_fill import find_kicad_python
    if not find_kicad_cli() or not find_kicad_python():
        print('SKIP: needs kicad-cli and KiCad\'s bundled python')
        return SKIP_EXIT
    tmp = tempfile.mkdtemp(prefix='krt_1581_kicad_')
    py = [sys.executable, '-X', 'utf8', '-B']

    print('1. route.py keeps the stub, and KiCad grades GND connected')
    src = evidence(write_board(os.path.join(tmp, 'in.kicad_pcb'), 0.45, True, 0.15))
    out = os.path.join(tmp, 'out.kicad_pcb')
    r = check(py + [tool('route.py'), src, out, '--nets', '*',
                    '--clearance', '0.1', '--track-width', '0.1',
                    '--via-size', '0.35', '--via-drill', '0.2'], accept=True)
    assert 'Graze prune: removed' not in r.stdout, \
        'the graze prune removed copper on this board:\n' + r.stdout[-2000:]
    with open(evidence(out), encoding='utf-8') as f:
        txt = f.read()
    assert (f'(start {CX:g} {CY:g})' in txt
            or f'(start {CX} {CY})' in txt), \
        "U1's stub is not in the output"
    print("  ok    no graze-prune removal; U1's stub is in the output")
    r = check(py + [tool('check_connected.py'), out, '--nets', 'GND'],
              accept=True)
    assert 'KiCad refill cross-check: ran' in r.stdout, \
        'the KiCad cross-check did not run:\n' + r.stdout[-1500:]
    assert 'ALL NETS FULLY CONNECTED' in r.stdout, r.stdout[-1500:]
    print('  ok    check_connected (KiCad cross-check ran): GND connected')

    print("\n2. with no stub, U1's pad is graded open, and KiCad agrees")
    bare = evidence(write_board(os.path.join(tmp, 'bare.kicad_pcb'), 0.45, False, 0.15))
    r = check(py + [tool('check_connected.py'), bare, '--nets', 'GND'],
              refuse='agrees with copper grading', code=1)
    assert 'Disconnected pads:' in r.stdout and \
        f'({CX:.2f}, {CY:.2f}) on F.Cu [U1]' in r.stdout, \
        'the open names no pad:\n' + r.stdout[-1500:]
    print('  ok    "Disconnected pads: (120.00, 115.00) on F.Cu [U1]"')

    print("\n3. a KiCad-only open names its stranded pad")
    therm = evidence(thermal_board(os.path.join(tmp, 'thermal.kicad_pcb')))
    r = check(py + [tool('check_connected.py'), therm, '--nets', 'GND'],
              refuse='UNCONNECTED per KiCad refill though copper grading '
                     'passed them: GND', code=1)
    assert 'Disconnected pads:' in r.stdout and \
        f'({CX:.2f}, {CY:.2f}) on F.Cu [G1]' in r.stdout, \
        'the KiCad-only open names no pad:\n' + r.stdout[-1500:]
    print('  ok    "Disconnected pads: (120.00, 115.00) on F.Cu [G1]"')

    print('\nPASS: GND keeps its only real join, and an open names its pad, '
          'KiCad-only or not')
    return 0


if __name__ == '__main__':
    sys.exit(main())

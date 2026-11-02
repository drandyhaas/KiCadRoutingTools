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
   check_connected names U1's pad under "Disconnected pads", read off KiCad's
   own fill rather than the link's zone-corner anchor.

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

    print("\n2. a KiCad-only open names its stranded pad")
    bare = evidence(write_board(os.path.join(tmp, 'bare.kicad_pcb'), 0.45, False, 0.15))
    r = check(py + [tool('check_connected.py'), bare, '--nets', 'GND'],
              refuse='UNCONNECTED per KiCad refill though copper grading '
                     'passed them: GND', code=1)
    assert 'Disconnected pads:' in r.stdout and \
        f'({CX:.2f}, {CY:.2f}) on F.Cu [U1]' in r.stdout, \
        'the KiCad-only open names no pad:\n' + r.stdout[-1500:]
    print('  ok    "Disconnected pads: (120.00, 115.00) on F.Cu [U1]"')

    print('\nPASS: GND keeps its only real join, and a KiCad-only open names '
          'its pad')
    return 0


if __name__ == '__main__':
    sys.exit(main())

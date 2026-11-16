#!/usr/bin/env python3
"""#1757: the final reconciliation never grants itself rip authority over a
net the step's --nets negates with '!'.

The board: /A, routed by an earlier step on F.Cu in 3 pieces, runs through the
only channel in a locked wall; /X has a pad on each side of the wall, so on one
layer it can only cross through /A's channel.

`route.py --nets '*' '!/A'` excludes /A. The main pass honoured that, but the
end-of-run reconciliation's #103 escalation read the step's negations from
`net_names` -- already expanded by main(), so it found none -- granted itself
`--rip-existing-nets /A`, ripped it, could not lay it again, and shipped /A
open with the improvement gate accepting the trade. The guard now reads the
run's hands-off set (#1621, stashed from the RAW patterns).

Rows:
  1. '!/A' in --nets: the main pass names /A as a net it may not rip (the
     fixture reaches the reconciliation's hint source), no reconciliation
     grant names /A, nothing rips /A, and /A ships its 3 segments.
  2. An operator --rip-existing-nets '/A' still overrides the negation: /A is
     ripped (the documented override door).

    python3 tests/test_1757_reconcile_rip_honours_negation.py
"""
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check as run_check, evidence, tool        # noqa: E402

FAILS = []
NAMES = ['/A', '/X', '/W1', '/W2']
NID = {n: i + 1 for i, n in enumerate(NAMES)}
FLAGS = ['--layers', 'F.Cu', '--track-width', '0.2', '--clearance', '0.2',
         '--via-size', '0.6', '--via-drill', '0.3', '--grid-step', '0.05',
         '--no-fix-drc-settings']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def _fp(ref, x, y, net):
    return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
            f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") (hide yes)'
            f' (effects (font (size 1 1) (thickness 0.15))))\n'
            f'    (attr smd)\n    (pad "1" smd rect (at 0 0) (size 1.0 1.0) (layers "F.Cu")'
            f' (net {NID[net]} "{net}"))\n  )\n')


def _seg(x1, y1, x2, y2, net):
    lock = '(locked yes) ' if net.startswith('/W') else ''
    return (f'  (segment {lock}(start {x1} {y1}) (end {x2} {y2}) (width 0.2) '
            f'(layer "F.Cu") (net {NID[net]}))\n')


def write_board(path):
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n'
         ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))\n'
         ' (setup (pad_to_mask_clearance 0))\n (net 0 "")\n')
    b += ''.join(f' (net {i + 1} "{n}")\n' for i, n in enumerate(NAMES))
    b += _fp('W1A', 20, 0.8, '/W1') + _fp('W1B', 20, 14.6, '/W1') + _seg(20, 0.8, 20, 14.6, '/W1')
    b += _fp('W2A', 20, 29.2, '/W2') + _fp('W2B', 20, 16.3, '/W2') + _seg(20, 16.3, 20, 29.2, '/W2')
    b += _fp('S1', 8, 15.45, '/A') + _fp('S2', 32, 15.45, '/A')
    b += (_seg(8, 15.45, 14, 15.45, '/A') + _seg(14, 15.45, 26, 15.45, '/A')
          + _seg(26, 15.45, 32, 15.45, '/A'))
    b += _fp('X1', 10, 8, '/X') + _fp('X2', 30, 8, '/X')
    for x1, y1, x2, y2 in ((0, 0, 40, 0), (40, 0, 40, 30), (40, 30, 0, 30), (0, 30, 0, 0)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(b + ')\n')
    import json
    pro = {"board": {"design_settings": {"rules": {"min_clearance": 0.2,
                                                   "min_hole_clearance": 0.1}}},
           "net_settings": {"classes": [{"name": "Default", "clearance": 0.2,
                                         "track_width": 0.2, "via_diameter": 0.6,
                                         "via_drill": 0.3}]}}
    with open(path[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump(pro, fh)
    return evidence(path, 'synthetic input board')


def a_segments(path):
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(evidence(path, 'routed output'))
    nid = next(i for i, n in p.nets.items() if n.name == '/A')
    return sum(1 for s in p.segments if s.net_id == nid)


def route(tmp, tag, extra):
    inp = write_board(os.path.join(tmp, f'{tag}_in.kicad_pcb'))
    out = os.path.join(tmp, f'{tag}_out.kicad_pcb')
    r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'), inp, out]
                  + extra + FLAGS, accept=True)
    return r.stdout + r.stderr, a_segments(out)


def main():
    tmp = tempfile.mkdtemp(prefix='t1757_')
    try:
        print("1. --nets '*' '!/A': the reconciliation leaves /A alone")
        log, segs = route(tmp, 'neg', ['--nets', '*', '!/A'])
        if not re.search(r"may not rip\. Retry with --rip-existing-nets '/A'", log):
            print("  [BROKEN TEST] the main pass never named /A as a blocker it may "
                  "not rip: the fixture does not reach the reconciliation's hint")
            return 1
        grant = [l for l in log.splitlines()
                 if 'Reconciliation rip authority' in l or 'lap authority (#572)' in l]
        check("no reconciliation grant names /A",
              not any('/A' in l for l in grant), '; '.join(grant))
        check("nothing rips /A", "PRE-EXISTING rip: '/A'" not in log)
        check("/A ships its 3 segments", segs == 3, f"{segs} segment(s)")
        check("the summary does not name /A broken",
              'ripped and not restored): /A' not in log)

        print("2. an operator --rip-existing-nets '/A' still overrides")
        log, segs = route(tmp, 'ovr', ['--nets', '*', '!/A',
                                       '--rip-existing-nets', '/A'])
        check("/A is ripped on the operator's word", "PRE-EXISTING rip: '/A'" in log)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1758: a casualty-reconcile partial restore ships what it restored.

The board (#1757's): /A, routed by an earlier step on F.Cu in 3 pieces, runs
through the only channel in a locked wall; /X has a pad on each side.

`route.py --nets /X --rip-existing-nets /A` rips /A to route /X; /A's reroute
fails and the casualty reconcile (diff_pair_custody branch D, "route.py #134
parity") puts back the 2 pieces that do not collide. That restore was appended
to the write-list but never made the net's result in routed_results, so
route.py's #87 superseded filter dropped it and the dead-end sweep, whose
protect set (_unfinished_net_ids) reads only routed_results, trimmed the
restored pieces: /A shipped 0 segments under a line saying 2 were restored.

Rows:
  1. the casualty reconcile's partial restore fires (the fixture reaches
     branch D), restoring 2 segments;
  2. no result is dropped as superseded, and the dead-end sweep trims nothing
     of /A;
  3. /A ships the 2 restored segments, and /X routes.

    python3 tests/test_1758_casualty_partial_restore_ships.py
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


def counts(path):
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(evidence(path, 'routed output'))
    out = {}
    for nm in ('/A', '/X'):
        nid = next(i for i, n in p.nets.items() if n.name == nm)
        out[nm] = sum(1 for s in p.segments if s.net_id == nid)
    return out


def main():
    tmp = tempfile.mkdtemp(prefix='t1758_')
    try:
        inp = write_board(os.path.join(tmp, 'in.kicad_pcb'))
        out = os.path.join(tmp, 'out.kicad_pcb')
        r = run_check([sys.executable, '-B', '-X', 'utf8', tool('route.py'), inp, out,
                       '--nets', '/X', '--rip-existing-nets', '/A'] + FLAGS,
                      accept=True)
        log = r.stdout + r.stderr
        m = re.search(r'PARTIAL /A: reroute failed; restored (\d+) segment', log)
        if not m:
            print("  [BROKEN TEST] no casualty partial restore of /A: the fixture "
                  "does not reach diff_pair_custody's branch D")
            return 1
        print(f"1. the casualty reconcile restored {m.group(1)} segment(s) of /A")
        check("no result dropped as superseded", 'superseded rip-reroute result' not in log)
        sweep = re.search(r'Dead-end sweep: trimmed (\d+) dead-end segment', log)
        check("the dead-end sweep trims none of the restore",
              sweep is None or sweep.group(1) == '0',
              sweep.group(0) if sweep else 'no sweep line')
        got = counts(out)
        check("/A ships the restored segments", got['/A'] == int(m.group(1)),
              f"{got['/A']} shipped, {m.group(1)} restored")
        check("/X routes", got['/X'] > 0, f"{got['/X']} segment(s)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

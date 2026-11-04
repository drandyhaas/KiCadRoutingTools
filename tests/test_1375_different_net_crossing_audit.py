#!/usr/bin/env python3
"""#1375: route.py audits the copper it is about to write for different-net
tracks that cross on one layer, and names every pair it laid.

A leg retried after a via-in-pad unblock searched without some committed
copper and laid a 278-segment detour through another net's track; the run
reported `failed 0` over a short (KiCad tracks_crossing, check_drc
SEGMENT-CROSSING). The cause of that retry's blindness is not isolated; this
is the net under it: no shorted pair ships unnamed.

Rows:
  1. `different_net_crossings`: a crossing pair with one new segment is named
     (nets, layer, point); a T-contact is a short too; same-net crossings,
     different layers, near-misses and input-vs-input pairs are not named;
     graphic copper is skipped.
  2. route.py on a clean board writes no `different_net_crossings` key (the
     negative control), and carries a pre-existing input crossing silently
     (it is not this run's).

    python3 tests/test_1375_different_net_crossing_audit.py
"""
import json
import os
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS)

from check_weird import different_net_crossings                 # noqa: E402
from synth import RoutingBoard                                  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def seg(x1, y1, x2, y2, layer='F.Cu', graphic=False):
    return SimpleNamespace(start_x=x1, start_y=y1, end_x=x2, end_y=y2, layer=layer,
                           graphic=graphic, width=0.2)


def main():
    name = lambda n: f'N{n}'
    a = seg(0, 0, 10, 10)
    b = seg(0, 10, 10, 0)
    got = different_net_crossings({1: [a], 2: [b]}, lambda s: s is b, name)
    check('1. an X crossing with one new segment is named',
          len(got) == 1 and {got[0]['net_a'], got[0]['net_b']} == {'N1', 'N2'}
          and got[0]['layer'] == 'F.Cu', str(got))
    got = different_net_crossings({1: [seg(0, 0, 10, 0)], 2: [seg(5, 0, 5, 8)]},
                                  lambda s: True, name)
    check('1. a T-contact is a short too', len(got) == 1, str(got))
    check('1. a same-net crossing is not named',
          not different_net_crossings({1: [a, b]}, lambda s: True, name))
    check('1. a crossing on different layers is not named',
          not different_net_crossings({1: [a], 2: [seg(0, 10, 10, 0, layer='B.Cu')]},
                                      lambda s: True, name))
    check('1. a near-miss is not named',
          not different_net_crossings({1: [seg(0, 0, 10, 0)], 2: [seg(5, 0.01, 5, 8)]},
                                      lambda s: True, name))
    check('1. an input-vs-input pair is not named',
          not different_net_crossings({1: [a], 2: [b]}, lambda s: False, name))
    check('1. graphic copper is skipped',
          not different_net_crossings({1: [a], 2: [seg(0, 10, 10, 0, graphic=True)]},
                                      lambda s: True, name))
    long_a = seg(0, 0.5, 80, 0.5)
    long_b = seg(79.5, 0, 79.5, 5)
    got = different_net_crossings({1: [long_a], 2: [long_b]}, lambda s: True, name)
    check('1. a crossing far along a long segment is found', len(got) == 1, str(got))

    # 2. route.py on a small board.
    bd = RoutingBoard(w=40, h=30)
    bd.pad('A', 10, 15, 'SIG')
    bd.pad('B', 30, 15, 'SIG')
    bd.seg(5, 3, 15, 3, 'X1')          # an input crossing pair, not this run's
    bd.seg(10, 0.5, 10, 6, 'X2')
    tmp = tempfile.mkdtemp(prefix='t1375_')
    pcb = os.path.join(tmp, 'b.kicad_pcb')
    out = os.path.join(tmp, 'o.kicad_pcb')
    js = os.path.join(tmp, 'o.json')
    open(pcb, 'w', encoding='utf-8').write(bd.text())
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', pcb, out,
                        '--nets', 'SIG', '--layers', 'F.Cu', 'B.Cu', '--json-out', js],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    summ = json.load(open(js)) if os.path.isfile(js) else {}
    check('2. the net routed', summ.get('failed') == 0, str(summ.get('failed')))
    check('2. no crossing is reported for copper the run did not lay',
          'different_net_crossings' not in summ
          and 'different-net track crossing' not in (r.stdout + r.stderr))

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

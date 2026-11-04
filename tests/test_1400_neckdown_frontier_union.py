#!/usr/bin/env python3
"""#1400: a power tap that fails its neck-down retry is analysed for rip-up on
BOTH attempts' frontiers, so the wall the narrow route meets is named.

interf_u's VCC phase-3 tap to edge finger BUS1.3 fails at the 0.7 mm power
width and then at the 0.2 mm neck-down. Rip-up analysis read the wide
attempt's frontier alone -- it cannot leave the 1.8 mm finger past its static
neighbours, so it never meets /PC-RST's B.Cu track 0.42 mm above the finger
-- blamed a finger-side net with one cell, ripped and restored it, and VCC
shipped open. The same board with `--power-nets-widths 0.2 0.2` (narrow
frontier only) named /PC-RST, ripped it and routed VCC.

The chain: signal routing of everything but VCC/GND, the /PC-RST wall staged
over VCC's finger (check_join --keep-staged), VCC/GND routed once at 0.2 to
leave VCC's finger tap open, then again at 0.7 with the rescue ladder off.

Rows:
  1. `_union_frontier_cells` keeps the first list's order and adds only the
     second's new cells.
  2. At 0.7 the neck-down retry fails and the cascade names and rips /PC-RST.
  3. VCC ends connected (no open pad pairs), as at 0.2.

    python3 tests/test_1400_neckdown_frontier_union.py      (about 2 min)
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from single_ended_routing import _union_frontier_cells          # noqa: E402

FAILS = []
COMMON = ['--no-bga-zones', 'U9', '--clearance-ceiling', '0.254', '--track-width', '0.2',
          '--via-size', '0.9', '--via-drill', '0.5', '--layer-costs', '1.0', '1.0',
          '--direction-preference-cost', '4000', '--turn-cost', '2000',
          '--escalation', 'board']


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def run(*argv, env=None):
    e = dict(os.environ)
    e.update(env or {})
    r = subprocess.run([sys.executable, '-X', 'utf8'] + list(argv), cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', env=e)
    return r.stdout + r.stderr


def main():
    a = [(1, 2, 0), (3, 4, 0)]
    b = [(3, 4, 0), (5, 6, 1)]
    check('1. the union keeps the first order and adds only new cells',
          _union_frontier_cells(a, b) == [(1, 2, 0), (3, 4, 0), (5, 6, 1)]
          and _union_frontier_cells(a, []) is a and _union_frontier_cells([], b) is b)

    tmp = tempfile.mkdtemp(prefix='t1400_')
    try:
        src = os.path.join(ROOT, 'kicad_files', 'interf_u_unrouted_placed.kicad_pcb')
        sig = os.path.join(tmp, 'sig.kicad_pcb')
        wall = os.path.join(tmp, 'wall.kicad_pcb')
        opened = os.path.join(tmp, 'open.kicad_pcb')
        w07 = os.path.join(tmp, 'w07.kicad_pcb')
        run('py_router/route.py', src, sig, '--nets', '*', '!VCC', '!GND', *COMMON,
            '--via-cost', '150')
        run('py_tools/check_join.py', sig, '/PC-RST', '163.4,132.1,B.Cu',
            '162.4,133.1,B.Cu', '162.4,134.2,B.Cu', '165.6,134.2,B.Cu', '167.0,135.6,B.Cu',
            '--width', '0.2', '--keep-staged', wall, '-q')
        if not os.path.isfile(wall):
            check('0. the wall board was staged', False)
            return 1
        log = run('py_router/route.py', wall, opened, '--nets', 'VCC', 'GND', *COMMON,
                  '--via-cost', '600', '--power-nets', 'GND', 'VCC',
                  '--power-nets-widths', '0.2', '0.2')
        m = re.findall(r'"pad_pairs_open": (\{[^}]*\})', log)
        check('precondition: the narrow run leaves VCC open on the staged wall',
              bool(m) and '"VCC"' in m[-1], m[-1] if m else 'no pad_pairs_open')

        log = run('py_router/route.py', opened, w07, '--nets', 'VCC', 'GND', *COMMON,
                  '--via-cost', '600', '--power-nets', 'GND', 'VCC',
                  '--power-nets-widths', '0.7', '0.7', env={'KICAD_NET_RESCUE': '0'})
        check('2. the wide attempt fails and the neck-down retry runs',
              'Wide route blocked - retrying at default track width (neck-down)' in log)
        check('2. the cascade names and rips /PC-RST', 'Ripping /PC-RST' in log)
        m = re.findall(r'"pad_pairs_open": (\{[^}]*\})', log)
        check('3. VCC ends connected', bool(m) and '"count": 0' in m[-1],
              m[-1] if m else 'no pad_pairs_open')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

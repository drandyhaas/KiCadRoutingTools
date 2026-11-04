#!/usr/bin/env python3
"""#1394: a route that fails AFTER the rip ladder ripped nets does not say
"no rippable blockers", is not called boxed in by static obstacles, and the
box-in hint never advises a clearance or track wider than the run's own.

Rows:
  1. `static_boxin_hint` advises a clearance / track only when it is NARROWER
     than the run's: a run at 0.09 / 0.0762 gets the grid advice alone, a run
     at 0.2 / 0.2 is still offered 0.15 / 0.15.
  2. The 4-layer rp2350 chain (rip ladder to N=5 on nets that stay blocked):
     every "All rip-up attempts failed" is followed by an HONEST line -- the
     main pass's "ROUTE FAILED - ripping N blocker net(s) (...) did not free
     a path", or the phase-3 re-route's "RIP-RESTORE" of the victim -- never by
     "no rippable blockers found" or a "search exhausted" static verdict.
     Which site the board's ladders land in moves with routing changes
     elsewhere (the main pass's ladder on one tree, only the re-route's on
     another), so the row holds every site to the contract rather than
     requiring one of them.

    python3 tests/test_1394_ladder_failure_message.py     (about 1.5 min)
"""
import os
import re
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from routing_diagnostics import static_boxin_hint               # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    if not cond:
        FAILS.append(name)
    print(('  PASS ' if cond else '  FAIL ') + name + (f'  {detail}' if detail else ''))


def main():
    res = {'iterations_forward': 30, 'iterations_backward': 0}
    fine = SimpleNamespace(grid_step=0.1, clearance=0.09, track_width=0.0762)
    wide = SimpleNamespace(grid_step=0.1, clearance=0.2, track_width=0.2)
    h = static_boxin_hint(res, fine)
    check('1. a run at 0.09 / 0.0762 is not advised a wider clearance or track',
          '--clearance' not in h and '--track-width' not in h and '--grid-step 0.05' in h, h)
    h = static_boxin_hint(res, wide)
    check('1. a run at 0.2 / 0.2 is still offered 0.15 / 0.15',
          '--clearance 0.15' in h and '--track-width 0.15' in h, h)

    tmp = tempfile.mkdtemp(prefix='t1394_')
    bare = os.path.join(tmp, 'bare.kicad_pcb')
    out = os.path.join(tmp, 'out.kicad_pcb')
    subprocess.run([sys.executable, '-X', 'utf8', 'tests/stress/strip_copper_only.py',
                    'kicad_files/rp2350_fpga_eensy_prePlane.kicad_pcb', bare],
                   cwd=ROOT, capture_output=True)
    r = subprocess.run(
        [sys.executable, '-X', 'utf8', 'py_router/route.py', bare, out, '--nets', '*',
         '--track-width', '0.0762', '--clearance-ceiling', '0.09', '--via-size', '0.25',
         '--via-drill', '0.15', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
         '--layer-costs', '1.0', '6.0', '2.5', '1.0', '--no-bga-zones', '--max-ripup', '5',
         '--power-nets', 'GND', '+3V3', 'Net-(D2-K)', '/VIN', 'Net-(D2-A)', 'Net-(U7-SW)',
         '+1V1', '/RP2354A/1V1', 'Net-(U6-VREG_LX)', 'Net-(U6-VREG_AVDD)',
         '--power-nets-widths', '0.7', '0.7', '0.7', '0.7', '0.7', '0.7', '0.5', '0.5',
         '0.5', '0.3'],
        cwd=ROOT, capture_output=True, text=True, encoding='utf-8', errors='replace')
    lines = (r.stdout + r.stderr).splitlines()
    after = [i for i, l in enumerate(lines)
             if 'All rip-up attempts failed: Restoring' in l]
    nxt = [lines[j] for i in after for j in range(i + 1, min(i + 3, len(lines)))
           if 'ROUTE FAILED' in lines[j]]
    check('2. the repro hit a ladder failure', bool(after), f'{len(after)} ladder failure(s)')

    def honest(i):
        follow = lines[i + 1:i + 3]
        return any(re.search(r'ROUTE FAILED - ripping \d+ blocker net\(s\) \(.*\) '
                             r'did not free a path', l) or 'RIP-RESTORE' in l
                   for l in follow)
    dishonest = [lines[i] for i in after if not honest(i)]
    check('2. every ladder failure is followed by an honest line (the ripped '
          'nets named, or the victim\'s restore)', bool(after) and not dishonest,
          str(dishonest[:2]))
    check('2. no "no rippable blockers found" follows a ladder failure',
          not any('no rippable blockers found' in l for l in nxt), str(nxt[:2]))
    bad = [lines[j] for i in after for j in range(i + 1, min(i + 4, len(lines)))
           if 'search exhausted' in lines[j] or '--clearance 0.15' in lines[j]]
    check('2. no static box-in verdict or wider-clearance advice after a ladder failure',
          not bad, str(bad[:1])[:200])

    print('FAILED: ' + ', '.join(FAILS) if FAILS else 'ALL OK')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())

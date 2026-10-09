#!/usr/bin/env python3
"""A pair's end connector onto a pose of a heading its dozen best lack (whole_snap POSE_HEADS).

  python3 tests/test_622_pose_heads.py

The snap lays a pair pose to pose, each end's candidates ranked by their legs' length and how far each pose stands off
the plan's line, the dozen best kept. The dozen can all stand straight ahead of the tips, and a line arriving across
the escape -- a pair berthing on the destination's side face, its lane coming round the corner -- cannot turn onto any
of them: the pair was refused ("no end connector with a body between them"), open. Where no pair of the dozen has a
body between them, the best of each heading they lack are searched too.

The bench: a generated bus of six pairs on four copper layers, every one berthing on the destination's SOUTH face
(synth_bus --ring-s). One round of whole_route.py (about a minute). Pinned: some pair of the round was laid onto an end
pose of a heading its dozen lacked (the snap says so in its log) -- and the dozen were refused somewhere in the round,
or the bench does not exercise them (BROKEN TEST). The round's grade line is printed, not asserted: another machine may
lay other copper.
"""
import glob
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
AWX = os.path.join(ROOT, 'awx')
LAYERS = 'F.Cu,B.Cu,In1.Cu,In2.Cu'
ARGS = ['--k', '12', '--pairs', '6', '--ring-s', '6', '--dst-cols', '10', '--copper', '4', '--gap', '2.5']
HEADED = 'an end pose of a heading its dozen lacked'
REFUSED = 'no end connector with a body between them'


def main():
    print('=' * 60)
    print("a pair's end connector onto a pose of a heading its dozen lack")
    print('=' * 60)
    with tempfile.TemporaryDirectory() as td:
        raw, bench, run = (os.path.join(td, f) for f in ('raw.kicad_pcb', 'bench.kicad_pcb', 'run'))
        e = dict(os.environ, ROUTE_LAYERS=LAYERS)
        for cmd in (['synth_bus.py', raw] + ARGS, ['make_bench.py', raw, 'SU1', 'SD1', bench]):
            r = subprocess.run([sys.executable] + cmd, cwd=AWX, capture_output=True, text=True, timeout=600, env=e)
            if r.returncode != 0:
                raise SystemExit(f'BROKEN TEST: {cmd[0]} failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}')
        r = subprocess.run([sys.executable, 'whole_route.py', '12', run, '1'], cwd=AWX, capture_output=True, text=True,
                           timeout=1800, env=dict(e, BASE=bench, DEST='SD1'))
        logs = [f for f in glob.glob(os.path.join(run, 'r1', 'loop', '*.log'))]
        if not logs:
            raise SystemExit(f'BROKEN TEST: one round left no snap logs:\n{r.stdout[-2000:]}{r.stderr[-2000:]}')
        whole = [ln for ln in r.stdout.splitlines() if ln.startswith('WHOLE')]
        print(f'  {whole[-1] if whole else "no grade line"}')
        text = {f: open(f, errors='replace').read() for f in logs}
        headed = sorted({ln.split()[0] for t in text.values() for ln in t.splitlines() if HEADED in ln})
        refused = sorted({ln.split()[0] for t in text.values() for ln in t.splitlines() if REFUSED in ln})
        print(f'  pairs laid onto a pose of another heading: {headed or "none"}')
        print(f'  pairs whose dozen were refused somewhere in the round: {refused or "none"}')
        if not headed and not refused:
            raise SystemExit("BROKEN TEST: no pair's dozen was refused in the round -- the bench does not exercise the "
                             "poses of other headings")
        if not headed:
            print(f'  FAIL no pair laid onto a pose of another heading, {len(refused)} refused at their end connectors')
            print('1 FAILURE(S)')
            sys.exit(1)
    print('PASS')
    sys.exit(0)


if __name__ == '__main__':
    main()

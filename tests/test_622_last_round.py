#!/usr/bin/env python3
"""The whole route's loop solves again only for a round that will run.

  python3 tests/test_622_last_round.py

whole_route.loop's round i ends, when its plan does not pass, with the solve again (s{i+1}.json) -- the plan
round i+1's geometry starts from. After the LAST round there is no round i+1: a loop that did not pass is laid
from its rounds' own smooth plans (held_plan, lay_rounds), and nothing opens that solve. This drives the loop
with every stage stood in for (whole_route.run replaced: each stage writes the file the loop reads next, the
gate finds fewer findings each round so the loop never stalls) and pins:

1. the solves: one after each round but the last -- s2 and s3 for three rounds, no s4;
2. the loop's own line saying so, and its exit (1: no round passed) and its rounds' plans, all three, as before.
"""
import json
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('ROUTE_LAYERS', None)            # (two routing layers: the solve's plain path)

import whole_route as wr  # noqa: E402

ROUNDS = 3


def main():
    fails = []
    solves = []

    def fake_run(argv, env, log_path=None, err=None, to=None, own_process=False):
        if argv[0] == 'stage_cache.py':
            argv = argv[argv.index('--') + 1:]
        script, args = argv[0], list(argv[1:])
        out, rc = '', 0
        if script == 'whole_geo.py':
            json.dump({'cuts': [], 'vcuts': []}, open(args[1], 'w'))
        elif script == 'whole_polish.py':
            json.dump({'flips': [], 'vcuts': []}, open(args[1], 'w'))
        elif script == 'whole_gate.py':
            if '--hot' in args:
                json.dump({'hot': [{'at': [0.0, 0.0]}]}, open(args[args.index('--hot') + 1], 'w'))
            i = int(re.search(r'[\\/]p(\d+)\.json$', args[0]).group(1))
            # (fewer findings each round: a new best every round, the loop never NOT CONVERGING)
            out, rc = f'dive 0 static {10 - i} shape 0 swim 0, pitch 0 in the plan, band 0.0 mm', 1
        elif script == 'whole_solve.py':
            json.dump({'lanes': {}}, open(args[0], 'w'))
            solves.append(os.path.basename(args[0]))
        elif script == 'whole_feedback.py':
            rc = 1                                   # (the ends not crowded)
        if log_path is not None:
            with open(log_path, 'w') as f:
                f.write(f'whole_solve: stood in for {os.path.basename(args[0]) if args else script}\n'
                        if script == 'whole_solve.py' else '')
        return rc, out

    run0 = wr.run
    wr.run = fake_run
    try:
        with tempfile.TemporaryDirectory() as td:
            first = os.path.join(td, 'solve.json')
            json.dump({'lanes': {}}, open(first, 'w'))
            loopd, logp = os.path.join(td, 'loop'), os.path.join(td, 'loop.log')
            env = dict(os.environ, BENCH=os.path.join(td, 'bench.kicad_pcb'))    # (no sidecar: no ends feedback)
            with open(logp, 'w') as lf:
                rc = wr.loop(first, loopd, ROUNDS, env, wr.Log(lf))
            log = open(logp).read()
            want = [f's{i}.json' for i in range(2, ROUNDS + 1)]
            if solves != want:
                fails.append(f'solves {solves}, want {want} -- none after the last round')
            if os.path.exists(os.path.join(loopd, f's{ROUNDS + 1}.json')):
                fails.append(f's{ROUNDS + 1}.json written after the last round')
            if f'round {ROUNDS} is the last: no solve again' not in log:
                fails.append('the loop does not say the last round solves no more')
            if rc != 1:
                fails.append(f'loop exit {rc}, want 1 (no round passed)')
            ps = sorted(f for f in os.listdir(loopd) if re.match(r'p\d+\.json$', f))
            if ps != [f'p{i}.json' for i in range(1, ROUNDS + 1)]:
                fails.append(f"rounds' plans {ps}, want p1..p{ROUNDS}")
            if 'no round passed' not in log:
                fails.append("the loop's end line is missing")
    finally:
        wr.run = run0
    for f in fails:
        print('FAIL', f)
    print(f'{"FAIL" if fails else "PASS"}: the loop solves again after rounds 1..{ROUNDS - 1}, not after round {ROUNDS} '
          f'({len(solves)} solves)')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

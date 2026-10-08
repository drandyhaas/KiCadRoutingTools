#!/usr/bin/env python3
"""resolve_round.py ROUND_DIR OUT_DIR [--dest REF] [--set K=V ...] -- one round's FIRST SOLVE again, on that round's own
fanout board: ROUND_DIR is a whole_route.py round (OUTDIR/rN, its fo.kicad_pcb and nets.lines), the solve runs under
the chain's own stage settings (whole_route.stage_env) as the chain runs it (SOLVE_UNPROVED=1), and its result lands in
OUT_DIR/solve.json and solve.log.

What it is for: telling a solve's trouble from the ends' trouble, on the board a run actually laid -- the same board
under a change to the solve (a run from another machine, or one from before a change), or the same solve on another
machine's board. `--set K=V` gives the solve a setting of its own (repeatable).

Prints the solve's own lines (whole_solve:, the workers', the tie vias) and the wall time; exits as whole_solve does.
"""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import whole_route  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('round_dir', help='a whole_route.py round: OUTDIR/rN')
    ap.add_argument('out_dir')
    ap.add_argument('--dest', default=None, help='the destination part (default: DEST, else DU1)')
    ap.add_argument('--set', action='append', default=[], metavar='K=V', help='a setting for the solve')
    a = ap.parse_args()
    rd = os.path.abspath(a.round_dir)
    board, nets = os.path.join(rd, 'fo.kicad_pcb'), os.path.join(rd, 'nets.lines')
    for f in (board, nets):
        if not os.path.isfile(f):
            sys.exit(f'resolve_round: {f} is missing -- ROUND_DIR must be a whole_route.py round (OUTDIR/rN)')
    os.makedirs(a.out_dir, exist_ok=True)
    env = whole_route.stage_env(dict(os.environ))
    env.update(BENCH=board, NETS='@' + nets, DEST=a.dest or env.get('DEST') or 'DU1', SOLVE_UNPROVED='1')
    for kv in a.set:
        k, _, v = kv.partition('=')
        env[k] = v
    out = os.path.abspath(os.path.join(a.out_dir, 'solve.json'))
    log = os.path.abspath(os.path.join(a.out_dir, 'solve.log'))
    t0 = time.time()
    with open(log, 'w') as f:
        rc = subprocess.run([sys.executable, os.path.join(HERE, 'whole_solve.py'), out], cwd=HERE, env=env,
                            stdout=f, stderr=subprocess.STDOUT).returncode
    for ln in open(log, errors='replace'):
        if ln.lstrip().startswith(('whole_solve:', 'bound-raising', 'plan-finding', 'tie vias', 'the root')):
            print(ln.rstrip()[:260])
    print(f'resolve_round: exit {rc} in {time.time() - t0:.0f} s -> {out if os.path.isfile(out) else "(no plan)"}')
    sys.exit(rc)


if __name__ == '__main__':
    main()

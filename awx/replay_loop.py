#!/usr/bin/env python3
"""replay_loop.py WORK ROUND_DIR --sizes CLR,TRK,FTRK,VIA,DRILL PLAN OUT_DIR -- a bus's recorded round's LOOP run again
on its own from a given plan: the plan's via ends relayed on the round's fanout bench (whole_route.relayed), then
whole_route.loop(PLAN, OUT_DIR/loop) under the environment the round gave its loop (replay_stage.round_env). No fanout,
no first solve. `--set SEED_CUTS=..,..` / `SEED_HIST=` / `SEED_FLIPS=` start it from a later round's state -- the cut
(cN.json), history (hpN.json) and flip (pN.json) files the recorded loop had by then -- so a re-solve's plan (a
replay_stage.py whole_solve) can be carried through the rounds after it.

What it is for: whether a change to a round's re-solve converges, in the loop's minutes rather than a round's. Its
log OUT_DIR/loop.log; exits with the loop's code (0 passes, 3 not converging, 4 crowded ends)."""
KRT_TOOL = {'scope': [], 'kind': 'actor'}   # a research tool (awx), catalogued, shown at no door

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))


def main():
    import replay_stage
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    replay_stage.add_round_args(ap)
    ap.add_argument('plan', help='the plan the loop starts from (a solve: solve.json, loop/sN.json, a replayed one)')
    ap.add_argument('out_dir')
    a = ap.parse_args()
    plan, outd = os.path.abspath(a.plan), os.path.abspath(a.out_dir)
    env = replay_stage.round_env(a.work, a.round_dir, a.sizes, a.dest, a.layers, a.set)
    os.makedirs(outd, exist_ok=True)
    os.chdir(HERE)
    os.environ.update(env)
    import awx_settings
    import whole_route as wr
    with awx_settings.given({**awx_settings.environ(), **env}):
        with open(os.path.join(outd, 'loop.log'), 'w') as lf:
            say = wr.Log(lf)
            wr.relayed(outd, env['BENCH'], json.load(open(plan)), env, say)
            rc = wr.loop(plan, os.path.join(outd, 'loop'), wr.loop_rounds(), env, say)
    for ln in open(os.path.join(outd, 'loop.log'), errors='replace'):
        if ln.startswith('===') or 'smooth:' in ln:
            print(ln.rstrip()[:200])
    print(f'replay_loop: loop exit {rc}')
    sys.exit(rc or 0)


if __name__ == '__main__':
    main()

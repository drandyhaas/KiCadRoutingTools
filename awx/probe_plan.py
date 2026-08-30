#!/usr/bin/env python3
"""The corridor plan without routing anything: per member its tooth
and stub in (s, o), its join / exit leg s, launch and target offsets,
layer rules. usage: probe_plan.py FANOUT_BOARD K [--dest REF]"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
import braid as te  # noqa: E402
from schedule import Schedule  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('k', type=int)
    ap.add_argument('--dest', default='DU1')
    a = ap.parse_args()
    nets = subprocess.run([sys.executable, os.path.join(HERE, 'coherent_nets.py'),
                           str(a.k)], capture_output=True, text=True).stdout.strip()
    names = [n for n in nets.split(',') if n]
    quiet = lambda *x: None  # noqa: E731
    ctx, groups = te.setup(a.board, names, a.dest, quiet)
    for gi, grp in enumerate(groups):
        c = te.Corridor(gi, grp, ctx, quiet)
        c.build_spine()
        c.classify()
        c.offsets(0.35)
        c.reserve_intervals()
        sched = Schedule(c.launch, c.target, ctx.tooth_layer)
        cols, gate = c.plan_columns(sched, {d: 1 for d in sched.divers},
                                    {d: 0 for d in sched.divers})
        c.lay_lanes(cols)
        print(f'corridor {gi}: {len(grp)} lanes, s0={c.s0:.2f} s1={c.s1:.2f}, '
              f'{len(cols)} columns (gate {gate}), divers {sched.divers}')
        print(f'  launch: {c.launch}')
        print(f'  target: {c.target}')
        for nm in sorted(grp, key=lambda n: c.launch_o[n]):
            s_t, o_t = c.st[nm]
            s_e, o_e = c.se[nm]
            jl = c.join_leg_s.get(nm)
            el = c.exit_leg_s.get(nm)
            print(f'  {nm:6} tooth ({s_t:5.2f},{o_t:4.2f}){ctx.tooth_layer[nm][0]}'
                  + (f' join leg s{jl:5.2f}' if jl is not None else '            ')
                  + f' launch {c.launch_o[nm]:5.2f} | target {c.target_o[nm]:5.2f}'
                  + (f' exit leg s{el:5.2f}' if el is not None else '            ')
                  + f' stub ({s_e:5.2f},{o_e:4.2f}){ctx.dest_layer[nm][0]}'
                  + f'  req {[(round(x, 2), round(y, 2), L[0]) for x, y, L in c.req[nm]]}')


if __name__ == '__main__':
    main()

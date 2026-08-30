#!/usr/bin/env python3
"""Print every lane's layer rules (req) and flag the s-stretches where
BOTH layers are closed -- a lane the plan makes impossible before the
router sees it. usage: probe_req.py FANOUT_BOARD K [--dest REF]"""
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
        cols = sched.columns({d: 1 for d in sched.divers}, {d: 0 for d in sched.divers})
        c.lay_lanes(cols)
        print(f'corridor {gi}: {len(grp)} lanes, s0={c.s0:.2f} s1={c.s1:.2f}')
        if c.exit_block:
            print('  exit block (inner->outer): ' + ', '.join(
                f'{nm}@s{c.se[nm][0]:.1f} o{c.exit_block[nm]:.2f} '
                f'leg {c.leg_layer.get(nm, "-")[0]} tooth {ctx.tooth_layer[nm][0]} '
                f'stub {ctx.dest_layer[nm][0]}'
                for nm in sorted(c.exit_block, key=lambda n: abs(c.exit_block[n]))))
        for nm in grp:
            F = [(xa, xb) for (xa, xb, L) in c.req[nm] if L == 'F.Cu']
            B = [(xa, xb) for (xa, xb, L) in c.req[nm] if L == 'B.Cu']
            dead = []
            for (fa, fb) in F:
                for (ba, bb) in B:
                    lo, hi = max(fa, ba), min(fb, bb)
                    if hi > lo + 1e-9:
                        dead.append((lo, hi))
            if dead:
                print(f'  {nm}: CLOSED ON BOTH LAYERS at '
                      + ', '.join(f'[{lo:.2f},{hi:.2f}]' for lo, hi in dead)
                      + f'   req={[(round(x, 2), round(y, 2), L[0]) for x, y, L in c.req[nm]]}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Survey the boards on disk by their JOINT floor (see joint_floor.py).

WHY. The audits found the per-net floor is 20-40 vias loose on exactly
the boards the search walks past and rejects, and the search's whole job
is telling those apart. A board's routed via count says how well it was
REALIZED; its joint floor says how good its STRUCTURE is. A board routed
at 130 whose paths floor at 88 is a better starting point than one routed
at 80 that floors at 78 -- there is 42 vias of headroom in the first and 2
in the second, and only the floor can see it.

    python3 floor_survey.py 41 [--limit N] [--glob 'tmp/*_k%d.kicad_pcb']
"""
from __future__ import annotations

import argparse
import glob as _glob
import os
import subprocess
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from ledger_cal import Judge                      # noqa: E402
from joint_floor import joint_floor               # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('k')
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--glob', default='')
    ap.add_argument('--src', default='U1')
    ap.add_argument('--dst', default='DU1')
    a = ap.parse_args()
    pat = a.glob or f'tmp/*_k{a.k}.kicad_pcb'
    boards = [b for b in sorted(_glob.glob(pat)) if '_fo_' not in b]
    if a.limit:
        boards = boards[:a.limit]
    nets = subprocess.run(
        [sys.executable, os.path.join(HERE, 'coherent_nets.py'), a.k],
        capture_output=True, text=True).stdout.strip().split(',')
    nets = [n for n in nets if n]
    print(f'{len(boards)} board(s) at K={a.k}, {len(nets)} net(s)')
    print(f'{"board":34s} {"paths":>5s} {"routed":>6s} {"perNet":>6s} '
          f'{"JOINT":>5s} {"head":>5s}')
    out = []
    for b in boards:
        try:
            J = Judge(b, nets, a.src, a.dst)
            # A board missing paths is not comparable with one that has
            # them -- its floor is over a SMALLER problem. But refusing
            # every such board threw away all 79 at K51, where one net
            # (SZQ, which runs to R6 rather than to the destination BGA)
            # can never have a path and the rest lose one to each open
            # net. So rank WITHIN a group of boards carrying the same
            # path SET, and never across groups.
            key = tuple(sorted(J.paths))
            fl, _per, st = joint_floor(J)
            if fl is None:
                print(f'{os.path.basename(b)[:34]:34s} {len(J.paths):5d} '
                      f'{J.act_total:6d} {J.floor_total:6d} {"INF":>5s}')
                continue
            out.append((fl, J.act_total, J.floor_total, b, key))
            print(f'{os.path.basename(b)[:34]:34s} {len(J.paths):5d} '
                  f'{J.act_total:6d} {J.floor_total:6d} {fl:5d} '
                  f'{J.act_total - fl:5d}')
        except Exception:
            print(f'{os.path.basename(b)[:34]:34s}   FAILED')
            traceback.print_exc(limit=1)
    if out:
        groups = {}
        for row in out:
            groups.setdefault(row[4], []).append(row)
        # the biggest comparable group first; a group of one says nothing
        for key, rows in sorted(groups.items(), key=lambda kv: -len(kv[1])):
            if len(rows) < 2:
                continue
            rows.sort()
            print(f'\nBEST STRUCTURE at K={a.k} among the {len(rows)} board(s) '
                  f'carrying the SAME {len(key)} path(s):')
            for fl, act, pn, b, _k in rows[:10]:
                print(f'  {fl:4d} floor  {act:4d} routed  ({act - fl:3d} headroom)  '
                      f'{os.path.basename(b)}')
            br = min(rows, key=lambda t: t[1])
            print(f'  best ROUTED in this group: {br[1]} vias '
                  f'({os.path.basename(br[3])}), floor {br[0]}')


if __name__ == '__main__':
    main()

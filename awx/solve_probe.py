#!/usr/bin/env python3
"""#622 the contract-time feasibility loop: solve -> oracle -> re-solve
ANCHORED to what the engine actually delivered -> oracle again.

The solver's lattice is a model (one tooth per node, TOL rasterization)
and its contracts can over-pack a region the engine cannot serve --
measured: SDQ7-class exhaustions where NO post-hoc repair works (free
re-fan refused, bench restore grazes). The general fix is to let the
ORACLE's own outcome re-shape the plan: round 0 solves free, the
oracle grades it and dumps the ACHIEVED contract; round 1 re-solves
with `--anchor` on that achievement (movement away from the proven
pose is charged), so infeasible asks migrate toward delivered ones
while the solver still optimizes crossings/length globally.

Kept round = fewer bare nets, then fewer oracle drc pairs, then
higher side obedience -- printed per round; the chain judges later.

usage: solve_probe.py K [--rounds 2] [--anchor-w 0.6]
"""
import argparse
import json
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

ap = argparse.ArgumentParser()
ap.add_argument('k')
ap.add_argument('--rounds', type=int, default=2)
ap.add_argument('--anchor-w', type=float, default=0.6)
ap.add_argument('--frame', default='chord')
a = ap.parse_args()


def oracle(prefix):
    """plan_fanout on this round's contract; returns the metric tuple
    (bare, drc_pairs, -side_obedience) and prints the summary."""
    out = f'{prefix}.kicad_pcb'
    r = subprocess.run(
        [sys.executable, 'plan_fanout.py', a.k,
         '--contract-json', f'{prefix}_contract.json',
         '--ach-json', f'{prefix}_ach.json', '--out', out],
        capture_output=True, text=True)
    open(f'{prefix}_fanout.log', 'w').write(r.stdout + r.stderr)
    txt = r.stdout
    m = re.findall(r'PLAN OBEDIENCE.*?layer (\d+)/(\d+), side '
                   r'(\d+)/(\d+),.*?(\d+) no-crossing', txt)
    side_ok, bare = (int(m[-1][2]), int(m[-1][4])) if m else (0, 99)
    mb = re.search(r'RESTORED bench copper for net\(s\) nothing '
                   r're-laid: (\S+)', txt)
    n_bare = len(mb.group(1).split(',')) if mb else 0
    mg = re.search(r'RESTORED copper grazes: (\S+)', txt)
    n_drc = len(mg.group(1).split(',')) if mg else 0
    print(f'  oracle {prefix}: side {side_ok}, bare {n_bare}, '
          f'graze-nets {n_drc}, no-crossing {bare}')
    return (n_bare, n_drc, -side_ok)


best = None
anchor = []
for rnd in range(a.rounds):
    prefix = f'tmp/sp{a.k}_r{rnd}'
    cmd = [sys.executable, 'plan_global.py', 'solve', '--k', a.k,
           '--out-prefix', prefix, '--frame', a.frame] + anchor
    r = subprocess.run(cmd, capture_output=True, text=True)
    open(f'{prefix}_solve.log', 'w').write(r.stdout + r.stderr)
    ms = re.search(r'SOLVED: predicted ([\d.]+)', r.stdout)
    print(f'round {rnd}: solved'
          + (f' (predicted {float(ms.group(1)):.1f})' if ms else '')
          + (' [anchored]' if anchor else ''))
    met = oracle(prefix)
    if best is None or met < best[0]:
        best = (met, prefix)
        print(f'  round {rnd} kept')
    anchor = ['--anchor', f'{prefix}_ach.json',
              '--anchor-w', str(a.anchor_w)]
print(f'BEST: {best[1]} (bare {best[0][0]}, graze-nets {best[0][1]}, '
      f'side {-best[0][2]}) -- chain it with '
      f'BASE={best[1]}.kicad_pcb')

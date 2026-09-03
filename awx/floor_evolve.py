#!/usr/bin/env python3
"""#622 FLOOR EVOLUTION: judge-guided surgical descent.

The measured division of labor (0902): the improve loop's moves
harvest SLACK against a flat floor; FLOOR moves are assignment
changes -- and applied SURGICALLY (replace one net's stub, braid
that net alone against everyone else's FROZEN copper) the judged
floor is realized exactly (SRST wrap: predicted 63->60, realized
60, board 73v -> 71v 0/0 in one move). Whole-board re-braids threw
the prediction away (measured: same move re-braided = floor 65,
1 open).

Loop: sweep assignment moves (alt faces x layers both ends +
in-place flips) -> realize each cheaply (relay + single-net braid on
the frozen world) -> judge by swap_floor -> apply the best candidate
whose REALIZED board grades 0 open / 0 drc and whose full-Judge
floor confirms the drop -> repeat until flat. Then SLACK HARVEST:
per net with act > dp, re-braid it alone in place, keep strict
improvements. Every accepted board is graded and floor-verified.

usage: floor_evolve.py TAG BEST_BOARD FO_BOARD K [--rounds 4]
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
os.chdir(HERE)
from ledger_cal import Judge              # noqa: E402
import surgical as sg                     # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--rounds', type=int, default=4)
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
a = ap.parse_args()
NETS = subprocess.run(
    [sys.executable, 'coherent_nets.py', a.k],
    capture_output=True, text=True).stdout.strip()
nets = NETS.split(',')
ENV = dict(os.environ, TWO_PAGE='1')


def swap_stub(best, src_board, n, out, strip_only_lane=False):
    return sg.swap_stub(best, src_board, n, out, strip_only_lane)


def grade(board):
    return sg.grade(board, NETS)


def braid_one(board, n, out):
    return sg.braid_one(board, n, out, dst=a.dst, env=ENV)


# ---- candidate menu (the sweep's geometry): surgical.Menu
_menu = sg.Menu(a.fo, a.src, a.dst)


def cands(nm):
    return _menu.cands(nm)


best = a.best
fo = a.fo
g0 = grade(best)
J = Judge(best, nets)
print(f'start: grade {g0} floor {J.floor_total} '
      f'changes {J.act_total}')

for rnd in range(a.rounds):
    print(f'== round {rnd}: floor {J.floor_total}, sweeping')
    found = []
    for nm in sorted(J.paths):
        seen = set()
        for label, largs in cands(nm):
            out = f'tmp/{a.tag}_r{rnd}_{nm}_{label}.kicad_pcb'
            r = subprocess.run(
                [sys.executable, 'relay_net.py', fo, nm,
                 '--out', out] + largs,
                capture_output=True, text=True)
            if not os.path.exists(out) or 'MISSED' in r.stdout:
                continue
            md = re.search(r'DELIVERED (\w+)/([\w.]+)@([-\d.]+)',
                           r.stdout)
            if md:
                k2 = (md.group(1), md.group(2),
                      round(float(md.group(3)), 1))
                if k2 in seen:
                    os.remove(out)
                    continue
                seen.add(k2)
            # blocker-aware realization (surgical.realize_relay)
            rr = sg.realize_relay(best, fo, nm, out, f'tmp/{a.tag}_b1',
                                  dst=a.dst)
            if rr is None:
                continue
            b1 = rr[0]
            nf = J.swap_floor(b1, nm)
            if nf is not None and nf < J.floor_total:
                found.append((nf, nm, label, out))
                # keep the realized candidate board
                keep = f'tmp/{a.tag}_r{rnd}_{nm}_{label}_b1.kicad_pcb'
                shutil.copy(b1, keep)
    if not found:
        print(f'  round {rnd}: landscape flat, stopping')
        break
    found.sort()
    applied = False
    for nf, nm, label, rel_fo in found[:5]:
        cand = f'tmp/{a.tag}_r{rnd}_{nm}_{label}_b1.kicad_pcb'
        gc = grade(cand)
        if gc[0] > g0[0] and gc[1] <= g0[1]:
            # completion rescue before the veto: close each stranded
            # net alone on the candidate's frozen copper
            _gb, base_open = sg.grade_full(best, NETS)
            cand, gc2, _o = sg.rescue_close(cand, rel_fo, base_open,
                                            NETS, a.tag, a.dst)
            print(f'  try {nm} {label}: strands -> rescue {gc} -> {gc2}')
            gc = gc2
        Jc = Judge(cand, nets)
        print(f'  try {nm} {label}: predicted {nf}, realized '
              f'floor {Jc.floor_total}, grade {gc}')
        # lexicographic: completion/drc improvement outranks the
        # floor (closing an open ADDS a path and legitimately RAISES
        # the floor); at equal (open, drc) the floor must drop
        if (gc[0], gc[1]) < (g0[0], g0[1]) \
                or ((gc[0], gc[1]) == (g0[0], g0[1])
                    and Jc.floor_total < J.floor_total):
            best = cand
            fo = rel_fo
            J = Jc
            g0 = gc
            print(f'  APPLIED {nm} {label}: floor -> '
                  f'{J.floor_total}, grade {gc}')
            applied = True
            break
    if not applied:
        print(f'  round {rnd}: no candidate realized cleanly, '
              'stopping')
        break

# ---- surgical slack harvest: re-braid slack nets alone, in place
print(f'== harvest: floor {J.floor_total}, '
      f'changes {J.act_total}, grade {g0}')
for _pass in range(2):
    moved = False
    slack = sorted(((J.paths[m].changes - J.per[m], m)
                    for m in J.paths), reverse=True)
    for d, m in slack:
        if d <= 0:
            continue
        scr = f'tmp/{a.tag}_h_scr.kicad_pcb'
        # strip m's lane but KEEP its stubs: swap in the stubs from
        # the current fo (the stub source of record)
        swap_stub(best, fo, m, scr)
        if not braid_one(scr, m, f'tmp/{a.tag}_h1'):
            continue
        h1 = f'tmp/{a.tag}_h1.kicad_pcb'
        gh = grade(h1)
        if (gh[0], gh[1]) > (g0[0], g0[1]) or gh[2] >= g0[2]:
            continue
        Jh = Judge(h1, nets)
        if Jh.floor_total > J.floor_total:
            continue
        keep = f'tmp/{a.tag}_h_{m}.kicad_pcb'
        shutil.copy(h1, keep)
        best, J, g0 = keep, Jh, gh
        print(f'  harvested {m}: grade {gh}, '
              f'changes {J.act_total}')
        moved = True
    if not moved:
        break

out = f'tmp/{a.tag}_final.kicad_pcb'
shutil.copy(best, out)
pro = os.path.splitext(best)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(out)[0] + '.kicad_pro')
print(f'FINAL {out}: grade {g0} floor {J.floor_total} '
      f'changes {J.act_total}')

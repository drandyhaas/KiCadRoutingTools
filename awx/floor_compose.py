#!/usr/bin/env python3
"""#622 COMPOSED floor moves (item 3 of the 0902 brief), on top of the
honest single-move sweep.

Why this exists: the K35 record (67v 0/0) sits at floor 62 against
the human's 54, and the brief says its single-move droppers are
exhausted. They were exhausted under the STALE metric -- fev35 ran
at 12:28, the pad-layer-terminal judge landed at 14:45 -- so the
first thing this tool does is the honest sweep. Then, when no single
assignment change drops the floor, PAIRS: two nets' assignment
changes whose joint floor drops although neither alone does (the
crossing they share flips only when both move), judged by
swap_floor applied twice (Judge on the first candidate's realized
board, then the second net swapped into it) and realized
SURGICALLY in sequence (net 1 braided alone on the frozen record,
net 2 braided alone on that).

Round:
  A. SWEEP (parallel over nets, resumable): every menu candidate
     (faces x layers both ends, keep-pos flips) relayed on the
     fanout board, the net's stub swapped into the record, braided
     alone; the realized board kept and judged (swap_floor).
  B. SINGLES: apply the best realized dropper (completion/drc
     outrank floor; strands get the completion rescue).
  C. PAIRS when B applies nothing: per net the --top candidates by
     judged floor (<= floor + 1), every cross-net pair judged
     jointly, the joint droppers realized in order, first clean
     accept wins.
After the rounds: harvest_k (--no-faces) for the slack, which also
runs collapse_dives + nudge_grazes.

usage: floor_compose.py TAG BEST FO K [--rounds 3] [--workers 6]
                        [--top 2] [--pairs-try 8]
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)
from ledger_cal import Judge  # noqa: E402
import surgical as sg         # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('best')
ap.add_argument('fo')
ap.add_argument('k')
ap.add_argument('--rounds', type=int, default=3)
ap.add_argument('--workers', type=int, default=6)
ap.add_argument('--top', type=int, default=2)
ap.add_argument('--pairs-try', type=int, default=8)
ap.add_argument('--src', default='U1')
ap.add_argument('--dst', default='DU1')
ap.add_argument('--only', default='',
                help='comma list: sweep only these nets (smoke)')
a = ap.parse_args()
NETS = sg.k_nets(a.k)
nets = NETS.split(',')
T0 = time.time()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


def vias_of(board, nm):
    """The net's via count on a board (cheap text census)."""
    ids = sg.net_ids(board)
    nid, name = ids[nm]
    txt = open(board, encoding='utf-8').read()
    return len(sg._collect(txt, 'via', sg._matcher(nid, name)))


def sweep_net(rnd, best, fo, menu, nm):
    """All of one net's candidates, relayed + realized on the frozen
    record. Returns [(label, relfo, b1)]. Resumable: an existing b1
    is reused."""
    out = []
    seen = set()
    # PAGE FLIPS first (no relay): the net re-braided alone with its
    # ride pinned F / B / swimmer through the sidecar
    for pg, label in (('F.Cu', 'pgF'), ('B.Cu', 'pgB'), (None, 'pgS')):
        stem = f'tmp/{a.tag}_r{rnd}_{nm}_{label}'
        b1 = stem + '_b1.kicad_pcb'
        if not os.path.exists(b1):
            scr = stem + '_scr.kicad_pcb'
            sg.swap_stub(best, fo, nm, scr)
            if not sg.braid_one(scr, nm, stem + '_b1', dst=a.dst,
                                pages={nm: pg}):
                continue
        out.append((label, fo, b1))
    for label, largs in menu.cands(nm):
        stem = f'tmp/{a.tag}_r{rnd}_{nm}_{label}'
        relfo = stem + '_fo.kicad_pcb'
        b1 = stem + '_b1.kicad_pcb'
        if os.path.exists(b1) and os.path.exists(relfo):
            out.append((label, relfo, b1))
            continue
        k2 = sg.relay(fo, nm, largs, relfo)
        if k2 is None:
            continue
        if k2 in seen:
            os.remove(relfo)
            continue
        seen.add(k2)
        # blocker-aware: a stub laid on the fanout board may sit under
        # another net's frozen LANE in the record -- that net is ripped
        # and braided with the mover (surgical.realize_relay)
        r = sg.realize_relay(best, fo, nm, relfo, stem, dst=a.dst)
        if r is None:
            continue
        out.append((label, relfo, b1))
    return out


def apply_grade(cand, relfo, label, best, g0, J):
    """Grade (with completion rescue) + full judge of a realized
    candidate; floor_evolve's acceptance rule. Returns the new state
    or None."""
    gc, opens = sg.grade_full(cand, NETS)
    if gc[0] > g0[0] and gc[1] <= g0[1]:
        _gb, base_open = sg.grade_full(best, NETS)
        cand, gc2, _o = sg.rescue_close(cand, relfo, base_open, NETS,
                                        f'{a.tag}_{label}', a.dst)
        say(f'  {label}: strands -> rescue {gc} -> {gc2}')
        gc = gc2
    Jc = Judge(cand, nets, a.src, a.dst)
    say(f'  try {label}: realized floor {Jc.floor_total}, grade {gc}')
    if (gc[0], gc[1]) < (g0[0], g0[1]) \
            or ((gc[0], gc[1]) == (g0[0], g0[1])
                and Jc.floor_total < J.floor_total):
        keep = f'tmp/{a.tag}_acc_{label}.kicad_pcb'
        shutil.copy(cand, keep)
        say(f'  APPLIED {label}: floor {J.floor_total} -> '
            f'{Jc.floor_total}, grade {g0} -> {gc}')
        return keep, gc, Jc
    return None


best, fo = a.best, a.fo
g0 = sg.grade(best, NETS)
J = Judge(best, nets, a.src, a.dst)
say(f'start {os.path.basename(best)}: grade {g0} floor {J.floor_total} '
    f'changes {J.act_total}')

for rnd in range(a.rounds):
    menu = sg.Menu(fo, a.src, a.dst)
    say(f'== round {rnd}: floor {J.floor_total}, sweeping '
        f'{len(J.paths)} nets x {a.workers} workers')
    names = sorted(J.paths)
    if a.only:
        names = [n for n in names if n in a.only.split(',')]
    with ThreadPoolExecutor(max_workers=a.workers) as ex:
        res = list(ex.map(
            lambda nm: (nm, sweep_net(rnd, best, fo, menu, nm)), names))
    cands = {}
    for nm, lst in res:
        rows = []
        for label, relfo, b1 in lst:
            nf = J.swap_floor(b1, nm)
            if nf is None:
                continue
            rows.append((nf, vias_of(b1, nm), label, relfo, b1))
        rows.sort()
        cands[nm] = rows
    json.dump({nm: [(r[0], r[1], r[2]) for r in rows]
               for nm, rows in cands.items()},
              open(f'tmp/{a.tag}_r{rnd}_sweep.json', 'w'), indent=1)
    n_all = sum(len(v) for v in cands.values())
    droppers = sorted((r[0], nm, r[2], r[3], r[4])
                      for nm, rows in cands.items() for r in rows
                      if r[0] < J.floor_total)
    say(f'  sweep: {n_all} realized candidates, {len(droppers)} '
        f'judged droppers')

    # B. singles
    applied = None
    for nf, nm, label, relfo, b1 in droppers[:5]:
        say(f'  single {nm} {label}: predicted {nf}')
        applied = apply_grade(b1, relfo, f'r{rnd}_{nm}_{label}',
                              best, g0, J)
        if applied:
            best, g0, J = applied
            fo = relfo
            break
    if applied:
        continue

    # C. pairs
    near = []
    for nm, rows in cands.items():
        near.extend((nm,) + r for r in rows[:a.top]
                    if r[0] <= J.floor_total + 1)
    say(f'  pairs: {len(near)} near-droppers, judging jointly')
    J1 = {}
    for c in near:
        nm, nf, nv, label, relfo, b1 = c
        J1[(nm, label)] = Judge(b1, nets, a.src, a.dst)
    joint = []
    for c1 in near:
        for c2 in near:
            if c1[0] == c2[0]:
                continue
            jf = J1[(c1[0], c1[3])].swap_floor(c2[5], c2[0])
            # a COMPOSED move must beat the better of its two singles:
            # a pair that only carries a solo dropper along is that
            # dropper (measured: every top pair at K35 was SA5's own
            # single plus a passenger, all failing on SA5's defect)
            if jf is not None and jf < min(J.floor_total, c1[1], c2[1]):
                joint.append((jf, c1, c2))
    joint.sort(key=lambda t: (t[0], t[1][2] + t[2][2]))
    say(f'  pairs: {len(joint)} joint droppers '
        f'(best {joint[0][0] if joint else "-"})')
    for jf, c1, c2 in joint[:a.pairs_try]:
        n1, _f1, _v1, l1, relfo1, _b1 = c1
        n2, _f2, _v2, l2, relfo2, _b2 = c2
        label = f'r{rnd}_{n1}_{l1}+{n2}_{l2}'
        say(f'  pair {n1} {l1} + {n2} {l2}: predicted {jf}')
        # compose the fanout: n2's ask relayed on n1's relayed board
        largs2 = next(la for lb, la in menu.cands(n2) if lb == l2)
        relfo12 = f'tmp/{a.tag}_{label}_fo.kicad_pcb'
        if sg.relay(relfo1, n2, largs2, relfo12) is None:
            say('    relay of the second net missed on the composed fo')
            continue
        # surgical in sequence (each with its blockers): n1 on the
        # record, n2 on that
        r1 = sg.realize_relay(best, fo, n1, relfo12,
                              f'tmp/{a.tag}_{label}_1', dst=a.dst)
        if r1 is None:
            say('    first net refused')
            continue
        r2 = sg.realize_relay(r1[0], relfo12, n2, relfo12,
                              f'tmp/{a.tag}_{label}_2', dst=a.dst)
        if r2 is None:
            say('    second net refused')
            continue
        say(f'    moved {r1[1]} then {r2[1]}')
        applied = apply_grade(r2[0], relfo12, label, best, g0, J)
        if applied:
            best, g0, J = applied
            fo = relfo12
            break
    if not applied:
        say(f'  round {rnd}: no single or pair realized cleanly, '
            'stopping')
        break

say(f'== evolved: grade {g0} floor {J.floor_total} changes '
    f'{J.act_total}; harvesting')
r = subprocess.run(
    [sys.executable, 'harvest_k.py', f'{a.tag}_h', best, fo, a.k,
     '--passes', '1', '--no-faces'], capture_output=True, text=True)
sys.stdout.write(r.stdout)
hf = f'tmp/{a.tag}_h_final.kicad_pcb'
out = f'tmp/{a.tag}_final.kicad_pcb'
src = hf if os.path.exists(hf) else best
shutil.copy(src, out)
pro = os.path.splitext(fo)[0] + '.kicad_pro'
if os.path.exists(pro):
    shutil.copy(pro, os.path.splitext(out)[0] + '.kicad_pro')
gf = sg.grade(out, NETS)
Jf = Judge(out, nets, a.src, a.dst)
say(f'FINAL {out}: grade {gf} floor {Jf.floor_total} changes '
    f'{Jf.act_total} (fo {fo})')

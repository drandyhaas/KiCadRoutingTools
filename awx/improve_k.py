#!/usr/bin/env python3
"""#622 intelligent iteration on a routed result (the wobble turned
into search). One chain result is a single draw from a chaotic
system; instead of accepting the draw, DIAGNOSE it and make informed
moves through the cheapest control channel -- the pages sidecar (a
braid-only rerun, no comb/berth redraw):

  1. braid the fanout board free -> baseline; dump its own plan.
  2. pin its own attempt-1 pages (kills inter-attempt drift).
  3. LEDGER DIAGNOSIS: per net, actual vias vs the model optimum on
     the same geometry. Every net paying more than the model is a
     WASTE net with a named mechanism (needless swimmer, wrong page,
     paged-but-weaving).
  4. targeted moves, one waste net at a time: flip its sidecar entry
     toward the model's ride (swimmer -> model page, paged -> other
     layer, paged -> swimmer). Re-braid; keep STRICTLY better only
     (open, drc, vias) lexicographic; iterate until a sweep is dry.

All boards/sidecars under tmp/. usage: improve_k.py FO_BOARD [K]
"""
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
sys.path.insert(0, HERE)
from plan_global import opt_rides  # noqa: E402

import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument('fo')
_ap.add_argument('k', nargs='?', default='28')
_ap.add_argument('tag', nargs='?', default='')
_ap.add_argument('--num-improve', type=int, default=3,
                 help='evolution sweeps over the waste list (each '
                      'sweep re-diagnoses and tries targeted flips)')
_a = _ap.parse_args()
FO = _a.fo
K = _a.k
NETS = subprocess.run([sys.executable, 'coherent_nets.py', K],
                      capture_output=True, text=True).stdout.strip()
TAG = os.path.join('tmp', 'imp' + K + _a.tag)
SIDECAR = os.path.splitext(FO)[0] + '.pages.json'
ENV = dict(os.environ, TWO_PAGE='1')


def braid(out):
    r = subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', out], env=ENV,
        capture_output=True, text=True)
    g = subprocess.run(
        [sys.executable, 'grade_k.py', out + '.kicad_pcb', NETS],
        capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', g)
    return (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m \
        else (99, 99, 999)


def dump_plan(path):
    subprocess.run(
        [sys.executable, 'braid.py', '--board', FO, '--dest', 'DU1',
         '--nets', NETS, '--out', os.devnull, '--plan-json', path],
        env=ENV, capture_output=True, text=True)
    return json.load(open(path))


def actual_vias(board):
    sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
    from kicad_parser import parse_kicad_pcb
    p = parse_kicad_pcb(board)
    nm = {i: n.name.split('/')[-1] for i, n in p.nets.items()}
    out = {}
    for v in p.vias:
        n = nm.get(v.net_id)
        if n:
            out[n] = out.get(n, 0) + 1
    return out


def put_pages(pages):
    json.dump(pages, open(SIDECAR, 'w'), indent=1)


# ---- step 1: free baseline (no sidecar)
if os.path.exists(SIDECAR):
    os.remove(SIDECAR)
best_score = braid(TAG + '_free')
best_board = TAG + '_free.kicad_pcb'
plan = dump_plan(TAG + '_plan.json')
own = {n: d.get('page') for n, d in plan.items()}
best_pages = None
print(f'free draw: open={best_score[0]} drc={best_score[1]} '
      f'vias={best_score[2]}')

# ---- step 2: pin own pages
put_pages(own)
s = braid(TAG + '_pin')
print(f'own pages pinned: open={s[0]} drc={s[1]} vias={s[2]}')
if s < best_score:
    best_score, best_board, best_pages = s, TAG + '_pin.kicad_pcb', dict(own)

# ---- steps 3-4: diagnose, targeted flips, keep strictly better
rides, divers, model = opt_rides(plan)
cur_pages = dict(best_pages) if best_pages else dict(own)
for sweep in range(_a.num_improve):
    act = actual_vias(best_board)
    waste = sorted((n for n in plan
                    if act.get(n, 0) > model.get(n, 99)),
                   key=lambda n: model[n] - act.get(n, 0))
    print(f'sweep {sweep}: {len(waste)} waste nets '
          f'{[(n, act.get(n, 0), model[n]) for n in waste[:8]]}')
    improved = False
    for n in waste:
        variants = []
        if cur_pages.get(n) is None:
            if n not in divers:
                variants.append(rides[n])       # swimmer -> model page
        else:
            other = 'B.Cu' if cur_pages[n] == 'F.Cu' else 'F.Cu'
            if rides.get(n) == other:
                variants.append(other)          # paged -> model's layer
            variants.append(None)               # paged -> swimmer
        for v in variants:
            trial = dict(cur_pages)
            trial[n] = v
            put_pages(trial)
            s = braid(TAG + '_try')
            tagv = v[0] if v else 'swim'
            if s < best_score:
                print(f'  {n} -> {tagv}: open={s[0]} drc={s[1]} '
                      f'vias={s[2]}   ACCEPT')
                best_score = s
                cur_pages = trial
                shutil.copy(TAG + '_try.kicad_pcb', TAG + '_best.kicad_pcb')
                shutil.copy(TAG + '_try.kicad_pro', TAG + '_best.kicad_pro')
                best_board = TAG + '_best.kicad_pcb'
                improved = True
                break
            print(f'  {n} -> {tagv}: open={s[0]} drc={s[1]} '
                  f'vias={s[2]}')
    if not improved:
        break

if os.path.exists(SIDECAR):
    os.remove(SIDECAR)
print(f'\nBEST: {best_board} open={best_score[0]} drc={best_score[1]} '
      f'vias={best_score[2]}')

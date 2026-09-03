#!/usr/bin/env python3
"""#622 PLAN SEARCH: plan candidates realized and judged through the
CHAIN (0903).

Measured this session: every model-side change to the plan's asks
(sub-bus split, via-site capacity, band lanes, far sites, per-net
sides) graded WORSE than the plain plan at K35/K41 even when the
fanout obeyed it better, while the human's destination assignment on
our own source stubs takes the realized crossing pairs 133 -> 100 and
the floor 68 -> 54 -- and the plan's own cost rates that assignment
worse than ours. The plan's cost is not a judge of its asks; the chain
is. So: the plan emits its per-bus ALTERNATIVES (every certified
side, the refused sides forced uncertified, and 'free' = no side
constraint), each is realized through fanout + braid + grade, and the
lexicographic chain grade (open, drc, vias) picks. Greedy rounds: the
best candidate that beats the incumbent is adopted (its bus pinned)
and the remaining buses are searched again from there.

Plain is round 0's incumbent, so the result is never worse than plain.
A candidate a few opens short (--rescue N) is judged AFTER the
single-net close rescue on its frozen copper (complete first, 0902).

Measured 0903 (chain stage + the same rescue on both sides, fresh
tags, grade = (open, drc, vias)):

    K28  plain 52 0/0          search 52 0/0   (no per-bus deviation helps)
    K35  plain 82 1o -> 86 0/0 search 69 3o -> 79 0/0   (8-net address
         bus forced `down`, SDQ10/SDQ8 free; two candidates reach 79)
    K41  plain 91 5o 3d        search 84 3o -> 92 2o 0d (7-net bus free)

~2-4 min per candidate at K35 (3 in parallel), 12 candidates a round.
The plan's own cost ranked NONE of the winners first: at K35 the
winning side cost 323 against the chosen 322, and 'free' has no cost
at all. That is the finding: realized grades, not the plan's model,
choose the assignment.

usage: plan_search.py TAG K [--parallel 3] [--rounds 2] [--max-cands 12]
"""
import argparse
import concurrent.futures as cf
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)
ap = argparse.ArgumentParser()
ap.add_argument('tag')
ap.add_argument('k')
ap.add_argument('--parallel', type=int, default=3)
ap.add_argument('--rounds', type=int, default=2)
ap.add_argument('--max-cands', type=int, default=12)
ap.add_argument('--base', default='fb_t2q_base.kicad_pcb')
ap.add_argument('--rescue', type=int, default=3, help='rescue candidates with up to N opens (0 = off)')
a = ap.parse_args()
K = a.k
T0 = time.time()
ENV = dict(os.environ, TWO_PAGE='1', TP_SCOPE='split', PLAN_OPTS='--two-page',
           BASE=a.base)
for k_ in ('PLAN_BUS_SPLIT', 'PLAN_BAND_LANES', 'PLAN_DST_SITES', 'TP_PLAN_SITE',
           'PLAN_BUS_SIDES', 'PLAN_LEG_CROSS'):
    ENV.pop(k_, None)
NETS = subprocess.run([sys.executable, 'coherent_nets.py', K],
                      capture_output=True, text=True).stdout.strip()


def say(msg):
    print(f'[{time.time() - T0:6.0f}s] {msg}', flush=True)


def plan_dump(forced, path):
    ff = f'{path}_force.json'
    json.dump(forced, open(ff, 'w'))
    env = dict(ENV, PLAN_ONLY='1', PLAN_FORCE_SIDES=ff,
               PLAN_DUMP_BUSES=f'{path}_buses.json')
    subprocess.run([sys.executable, 'fanout_from_plan.py', f'{path}.kicad_pcb', K,
                    f'--board={a.base}', '--no-lines', '--two-page',
                    '--no-plane-drop'], env=env, capture_output=True, text=True)
    return json.load(open(f'{path}_buses.json'))


def realize(label, forced):
    """chain_k with the forced sides; returns (open, drc, vias) or None."""
    ff = f'tmp/{label}_force.json'
    json.dump(forced, open(ff, 'w'))
    env = dict(ENV, PLAN_FORCE_SIDES=ff)
    with open(f'tmp/{label}_chain.log', 'w') as lg:
        subprocess.run(['bash', 'chain_k.sh', f'tmp/{label}', K, '--',
                        '--no-plane-drop'], env=env, stdout=lg, stderr=lg)
    board = f'tmp/{label}_k{K}.kicad_pcb'
    if not os.path.exists(board):
        return None
    r = subprocess.run([sys.executable, 'grade_k.py', board, NETS],
                       capture_output=True, text=True).stdout
    m = re.search(r'open=(\d+) drc=(\d+) vias=(\d+)', r)
    g = (int(m.group(1)), int(m.group(2)), int(m.group(3))) if m else None
    if g and a.rescue and 0 < g[0] <= a.rescue and g[1] == 0:
        # COMPLETE FIRST (0902 rule): a candidate a few opens short is
        # judged AFTER the single-net close rescue on its frozen copper
        # -- the search's first K35 win was exactly that (69v/3o ->
        # 79v clean, where plain's rescue lands higher)
        rs = subprocess.run([sys.executable, '-c', f'''
import surgical as sg
b, g, o = sg.rescue_close({board!r}, {f"tmp/{label}_fo_k{K}.kicad_pcb"!r}, set(), {NETS!r}, {label!r}, 'DU1')
print('RESCUED', b, g)'''], capture_output=True, text=True).stdout
        m2 = re.search(r"RESCUED (\S+) \((\d+), (\d+), (\d+)\)", rs)
        if m2:
            g2 = (int(m2.group(2)), int(m2.group(3)), int(m2.group(4)))
            say(f'  {label}: rescue {g} -> {g2} ({m2.group(1)})')
            g = min(g, g2)
    return g


forced = {}
incumbent = None
best_label = None
for rnd in range(a.rounds + 1):
    buses = plan_dump(forced, f'tmp/{a.tag}_r{rnd}_plan')
    cands = []
    if rnd == 0:
        cands.append(('plain', {}))
    for key, b in buses.items():
        if key in forced:
            continue
        chosen = b.get('chosen')
        base_cost = min((c for d, c, _x in b['alts'] if d == chosen), default=0)
        opts = [(c - base_cost, d) for d, c, _x in b['alts'] if d != chosen]
        opts.append((0.0, 'free'))
        opts += [(1e6, d) for d in b['refused']]
        for delta, d in sorted(opts):
            cands.append((f'{d}:{key[:18]}', dict(forced, **{key: d})))
    # per-bus alternatives interleaved: the cheapest deviation of every
    # bus before the second of any, largest bus first
    order, seen = [], {}
    for c in cands:
        k_ = c[0].split(':', 1)[-1]
        seen[k_] = seen.get(k_, 0) + 1
        order.append((seen[k_], -len(c[1]), c))
    order.sort(key=lambda t: (t[0], t[1]))
    cands = [c for _n, _l, c in order][:a.max_cands + (1 if rnd == 0 else 0)]
    say(f'round {rnd}: {len(cands)} candidate(s): ' + ', '.join(c[0] for c in cands))
    results = {}
    with cf.ThreadPoolExecutor(max_workers=a.parallel) as ex:
        futs = {ex.submit(realize, f'{a.tag}_r{rnd}_c{i}', f): (i, name, f)
                for i, (name, f) in enumerate(cands)}
        for fut in cf.as_completed(futs):
            i, name, f = futs[fut]
            g = fut.result()
            results[i] = (g, name, f)
            say(f'  c{i} {name}: {g}')
    ranked = sorted(((g, i) for i, (g, _n, _f) in results.items() if g), key=lambda t: t[0])
    if not ranked:
        say('no candidate realized')
        break
    g, i = ranked[0]
    say(f'round {rnd} best: c{i} {results[i][1]} {g} (incumbent {incumbent})')
    if incumbent is None or g < incumbent:
        incumbent, forced, best_label = g, results[i][2], f'{a.tag}_r{rnd}_c{i}'
        say(f'  ADOPT {best_label}: forced {forced}')
    else:
        say('  no improvement; stop')
        break
say(f'FINAL {best_label}: {incumbent} forced {forced}')

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

Version 2 (beam + cache): ties go to the candidate with FEWER forced
buses (a no-op 'free' tie was pinning a bus and hiding the path behind
it); `--beam N` expands the incumbent plus the best tied/near bases
(deduped by the realized asks dump); realized grades are CACHED by
forced assignment (tmp/plan_search_cache_k{K}.json -- the chain is
deterministic) so a relaunch replays its rounds in seconds.

    K35  86 -> 79 (round 1) -> 68 clean (round 3: the 4-net data bus
         forced RIGHT, a side certify REFUSED, + the 7- and 11-net
         buses free; 63/1 open rescued to 68); plateau at 68 by round 6.
    Evolution from that 68 board: improve_k 66 -> 64 clean, the new
    K35 record (was 66; tmp/k35rec64_final).

~2-4 min per candidate at K35 (3 in parallel), 12-16 candidates a round.
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
ap.add_argument('--beam', type=int, default=3, help='bases expanded per round (incumbent + tied/near candidates)')
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


CACHE_PATH = f'tmp/plan_search_cache_k{K}.json'
try:
    CACHE = json.load(open(CACHE_PATH))
except Exception:
    CACHE = {}


def ckey(forced):
    return json.dumps(sorted(forced.items())) + f'|rescue{a.rescue}'


def realize(label, forced):
    """chain_k with the forced sides; returns (open, drc, vias) or None.
    The chain is deterministic, so a realized assignment is cached
    across runs (tmp/plan_search_cache_k{K}.json)."""
    hit = CACHE.get(ckey(forced))
    if hit:
        say(f'  {label}: cached {hit["grade"]} ({hit["label"]})')
        return tuple(hit['grade']), hit.get('sig')
    ff = f'tmp/{label}_force.json'
    json.dump(forced, open(ff, 'w'))
    env = dict(ENV, PLAN_FORCE_SIDES=ff)
    with open(f'tmp/{label}_chain.log', 'w') as lg:
        subprocess.run(['bash', 'chain_k.sh', f'tmp/{label}', K, '--',
                        '--no-plane-drop'], env=env, stdout=lg, stderr=lg)
    board = f'tmp/{label}_k{K}.kicad_pcb'
    if not os.path.exists(board):
        return None, None
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
    # the plan's realized asks (when the fanout dumps them) identify
    # a no-op deviation whose plan equals another's
    sig = None
    try:
        sig = open(f'tmp/{label}_fo_k{K}_asks.json').read()
    except OSError:
        pass
    if g:
        CACHE[ckey(forced)] = dict(grade=list(g), label=label, sig=sig)
        json.dump(CACHE, open(CACHE_PATH, 'w'))
    return g, sig


forced = {}
incumbent = None
best_label = None
bases = [{}]                 # the assignments this round expands (beam)
tried = set()                # forced dicts already realized


def expand(forced, rnd, bi):
    """Single-bus deviations of one base assignment, cheapest per bus
    first (plan cost), interleaved so every bus's first deviation
    comes before any bus's second."""
    buses = plan_dump(forced, f'tmp/{a.tag}_r{rnd}_b{bi}_plan')
    cands = []
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
    order, seen = [], {}
    for c in cands:
        k_ = c[0].split(':', 1)[-1]
        seen[k_] = seen.get(k_, 0) + 1
        order.append((seen[k_], -len(c[1]), c))
    order.sort(key=lambda t: (t[0], t[1]))
    return [c for _n, _l, c in order]


def fkey(f):
    return tuple(sorted(f.items()))


for rnd in range(a.rounds + 1):
    cands = []
    if rnd == 0:
        cands.append(('plain', {}))
    # BEAM: expand every base of the round, interleaved, so a base
    # that TIED the incumbent (a no-op on its own) still gets its
    # second deviation tried -- the K35 79 needed exactly that (the
    # 8-net bus `down` ties plain at 86; with SDQ10/SDQ8 freed on top
    # it reaches 69/3 open -> 79 clean)
    per_base = [expand(f, rnd, bi) for bi, f in enumerate(bases)]
    i = 0
    while any(per_base) and len(cands) < a.max_cands + (1 if rnd == 0 else 0):
        lst = per_base[i % len(per_base)]
        i += 1
        while lst:
            c = lst.pop(0)
            if fkey(c[1]) not in tried:
                cands.append(c)
                break
    for c in cands:
        tried.add(fkey(c[1]))
    say(f'round {rnd}: {len(cands)} candidate(s): ' + ', '.join(c[0] for c in cands))
    results = {}
    with cf.ThreadPoolExecutor(max_workers=a.parallel) as ex:
        futs = {ex.submit(realize, f'{a.tag}_r{rnd}_c{i}', f): (i, name, f)
                for i, (name, f) in enumerate(cands)}
        for fut in cf.as_completed(futs):
            i, name, f = futs[fut]
            g, sig = fut.result()
            results[i] = (g, name, f, sig)
            say(f'  c{i} {name}: {g}')
    # ties go to the candidate with FEWER forced buses (plain first):
    # adopting a no-op deviation pins a bus for nothing and hides the
    # deviations behind it (measured: a 'free' tie at 86 adopted over
    # plain, and the 79 path through that bus's `down` was never seen)
    ranked = sorted(((g, len(results[i][2]), i) for i, (g, _n, _f, _s) in results.items() if g))
    if not ranked:
        say('no candidate realized')
        break
    g, _nf, i = ranked[0]
    say(f'round {rnd} best: c{i} {results[i][1]} {g} (incumbent {incumbent})')
    if incumbent is None or g < incumbent:
        incumbent, forced, best_label = g, results[i][2], f'{a.tag}_r{rnd}_c{i}'
        say(f'  ADOPT {best_label}: forced {forced}')
    # next round's bases: the incumbent plus the best few OTHER
    # candidates within the beam (ties and near-ties, judged by grade)
    bases = [forced]
    sigs = {results[i][3]} if results[i][3] else set()
    for g2, _nf2, i2 in ranked:
        f2, s2 = results[i2][2], results[i2][3]
        if len(bases) >= a.beam:
            break
        if fkey(f2) == fkey(forced) or g2[0] > incumbent[0] or g2[1] > incumbent[1]:
            continue
        if s2 and s2 in sigs:
            continue                      # a no-op: same plan as a base
        bases.append(f2)
        if s2:
            sigs.add(s2)
    say(f'  bases for round {rnd + 1}: ' + '; '.join(
        ', '.join(f'{k[:14]}={v}' for k, v in b.items()) or 'plain' for b in bases))
    if incumbent is not None and g >= incumbent and len(bases) == 1:
        say('  no improvement and nothing to expand; stop')
        break
say(f'FINAL {best_label}: {incumbent} forced {forced}')

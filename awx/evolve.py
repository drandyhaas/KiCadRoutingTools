#!/usr/bin/env python3
"""evolve.py -- a POPULATION of routed worlds: local descent in each, far
jumps and crossovers between them, elitist selection on the routed grade
(#622, 2026-09-18; Andy: "converge monotonically to a local optimum, and
sometimes jump, in another parallel world, to a far-away place and let
that evolve to its own minimum -- genetic optimization, roughly").

A WORLD is a fanout board (both ends laid, its plan sidecar beside it)
and its routed board with a grade. Three operators, each one subprocess,
so a generation runs its worlds side by side (--jobs; each is already the
shape of a Modal arm):

  descend   one round of `replan.py` in --mode=incremental with the
            coupled-set probes (--coupled=census): the worst nets'
            ends moved one at a time, only the coupled lanes re-routed,
            the rest of the copper kept; a standing probe IS the new
            board. Monotone by construction.
  jump      a plan-level RE-SOLVE with no holds: a few random nets' current
            classes banned at both ends, another CP-SAT seed, the parent
            plan as hint; fanned out and braided by the chain on the
            parent's source view. Lands in another basin -- usually worse
            at first; descent from there is the point.
  cross     the chain with the hold channel over TWO parents: a random
            half of the nets held at parent A's ends, the rest at parent
            B's (B's teeth named as moves on A's source view), the solve
            filling what conflicts. The genome is per-net ends, so this
            is the natural recombination.

Selection is elitist over exact routed grades (open, drc, vias), worlds
deduplicated by copper; the population's best is monotone.

usage: evolve.py TAG K --seeds=STEM[,STEM...] [--pop=4] [--gens=3]
                 [--jumps=2] [--cross=1] [--jobs=2] [--jump-nets=2]
                 [--descend="--rounds=2 --worst=6 --probes=2 --min-vias=2 --coupled=census --grade=inproc"]
                 [--board=BENCH] [--dest=REF] [--seed=N]
                 [--descend-env="DST_CLIMB=2"] [--jump-env="DST_CLIMB=2 SRC_CLIMB=4"]
  --seeds   recorded worlds: a chain stem (STEM_fo_kK.kicad_pcb + STEM_kK.kicad_pcb)
            or a replan stem (STEM_fo.kicad_pcb + STEM.kicad_pcb); relative to tmp/
            unless it contains a slash.
  writes    tmp/TAG/g<N>/... per generation, tmp/TAG/best_k<K>.kicad_pcb (+ _fo,
            sidecar) whenever the best changes, tmp/TAG/evolve_k<K>.json.
Run it under the chain's own env (PLAN_PAGES=1 PLAN_JUDGE=count ...).
"""
import concurrent.futures
import json
import os
import random
import re
import shutil
import subprocess
import sys
import time
from collections import Counter

ARGV = [a for a in sys.argv[1:] if not a.startswith('--')]
OPTS = dict(a[2:].split('=', 1) for a in sys.argv[1:] if a.startswith('--'))
os.environ.setdefault('SRC_CLIMB', '0')        # the chain's menus for our own reading (see plan_loop)
# a harness: the caches the router leaves off, on (the probe memo's closed worlds, the taut strings)
os.environ.setdefault('PROBE_MEMO', '1')
os.environ.setdefault('TAUT_MEMO', '1')
# THE BENCH REACHES EVERY OPERATOR (2026-09-19, the zynq article): replan
# defaults to the H3 bench and DU1, and the three replan calls below used
# to pass neither, so on a second article every descent re-fanned the
# WRONG board. Set in main() from --board / --dest; on the H3 bench these
# are replan's own defaults, so nothing there changes.
BENCH_ARGS = []

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb  # noqa: E402
import source_realize as sr  # noqa: E402
import braid as te  # noqa: E402
import replan  # noqa: E402
import plan_loop as pl  # noqa: E402  arm_boards, winner_pair, grade, better, fmt_g, run_chain
from coherent_nets import coherent_nets  # noqa: E402
import dedupe_boards  # noqa: E402
import probe_memo as pm  # noqa: E402  closed worlds: a null descent is never re-run
import rules as _rules  # noqa: E402

log = pl.log


def stage_world(dst_stem, F, R):
    """A world in the --from convention: STEM.kicad_pcb (+ log, pack,
    refusals) and STEM_fo.kicad_pcb (+ sidecar)."""
    replan.copy_board(R, dst_stem + '.kicad_pcb', eco=True)
    for ext in ('.log', '.pack.json', '_refusals.json', '.census.json'):
        s = R[:-len('.kicad_pcb')] + ext
        if os.path.exists(s):
            shutil.copy(s, dst_stem + ext)
    replan.copy_board(F, dst_stem + '_fo.kicad_pcb')
    shutil.copy(F[:-len('.kicad_pcb')] + '.plan.json', dst_stem + '_fo.plan.json')


def import_seed(spec, K, nets_csv, dst_stem):
    """A recorded run as a world: a chain stem (with its portfolio arms, the
    shipped one staged with its sidecars) or a replan stem."""
    stem = spec if os.path.isabs(spec) else os.path.join(HERE, spec)
    if not (os.path.exists(f'{stem}_fo_k{K}.kicad_pcb') or os.path.exists(f'{stem}_fo.kicad_pcb')):
        stem = os.path.join(HERE, 'tmp', spec)          # relative to tmp/ when not found as given
    F = R = None
    if os.path.exists(f'{stem}_fo_k{K}.kicad_pcb'):
        pairs = pl.arm_boards(stem, K)
        F, R = pl.winner_pair(stem, K, pairs)
    elif os.path.exists(f'{stem}_fo.kicad_pcb') and os.path.exists(f'{stem}.kicad_pcb'):
        F, R = f'{stem}_fo.kicad_pcb', f'{stem}.kicad_pcb'
    if not F or not R:
        return None
    if not os.path.exists(R[:-len('.kicad_pcb')] + '.pack.json'):
        return None                                   # no braid record: replan cannot read its verdict
    stage_world(dst_stem, F, R)
    g, line = pl.grade(dst_stem + '.kicad_pcb', nets_csv)
    return g


def child_env(extra=None, chain=False):
    env = dict(os.environ)
    env.pop('SRC_CLIMB', None)          # replan sets its own 14; the chain's default is 0
    env.update(extra or {})
    return env


def world_key(world, K, args, env_extra):
    """What a descent of this world is a function of: its routed copper,
    its fanout copper, the plan sidecar and the census beside it, the
    descent's arguments and knobs, the code."""
    stem = world['stem']
    pr = replan.parsed(stem + '.kicad_pcb')
    pf = replan.parsed(stem + '_fo.kicad_pcb')

    def _sha(path):
        if not os.path.exists(path):
            return None
        import hashlib
        with open(path, 'rb') as f:
            return hashlib.sha1(f.read()).hexdigest()[:16]
    return pm.key_of({'code': pm.code_hash(), 'knobs': pm.knob_hash(env_extra), 'K': K, 'args': args,
                      'R': pm.copper_hash(pr.segments, pr.vias), 'F': pm.copper_hash(pf.segments, pf.vias),
                      'sidecar': _sha(stem + '_fo.plan.json'), 'census': _sha(stem + '.census.json')})


def descend(world, K, out_dir, args, nets_csv, env_extra=None):
    """One replan round from this world; the improved world or None.
    A world whose descent under these arguments gained nothing before
    is CLOSED: not descended again (probe_memo, 'closed'); the parent
    stands."""
    out = os.path.join(out_dir, 'd')
    t0 = time.time()
    closed = pm.Store(K, 'closed') if pm.ENABLED else None
    key = world_key(world, K, args, env_extra) if closed else None
    if closed and closed.get(key):
        pm.bump('closed_hit')
        return None, f'closed: its descent gained nothing before ({closed.get(key).get("origin")})', 0
    with open(out + '.out', 'w', encoding='utf-8') as f:
        p = subprocess.run([sys.executable, '-u', os.path.join(HERE, 'replan.py'), world['stem'], str(K),
                            f'--from={world["stem"]}', f'--out={out}', '--mode=incremental',
                            '--apply=strip'] + BENCH_ARGS + args.split(), cwd=HERE, env=child_env(env_extra),
                           stdout=f, stderr=subprocess.STDOUT, text=True)
    stem = f'{out}_rp_k{K}'
    if not os.path.exists(stem + '.kicad_pcb'):
        return None, f'no board (rc {p.returncode})', round(time.time() - t0)
    g, line = pl.grade(stem + '.kicad_pcb', nets_csv)
    if closed and g is not None and not pl.better(g, world['grade']):
        closed.put(key, {'origin': world.get('origin'), 'name': world.get('name'), 'grade': g,
                         'stem': stem, 'seconds': round(time.time() - t0), 'when': time.time()})
    return {'stem': stem, 'grade': g, 'origin': f'descend<{world["name"]}>'}, line, round(time.time() - t0)


def chain_world(tagpath, K, nets_csv, dst_stem, origin):
    stem = os.path.join(HERE, tagpath)
    pairs = pl.arm_boards(stem, K)
    F, R = pl.winner_pair(stem, K, pairs)
    if not F or not R:
        return None
    stage_world(dst_stem, F, R)
    g, line = pl.grade(dst_stem + '.kicad_pcb', nets_csv)
    return {'stem': dst_stem, 'grade': g, 'origin': origin}


def jump_near(world, K, out_dir, n_nets, rng, nets_csv, env_extra=None, tries=3):
    """A NEAR jump: `n_nets` random nets moved to a random other class
    each through the descent's own probes (replan --perturb), the probe
    board taken whatever its grade. One probe per net instead of a
    chain; lands a move or two away."""
    out = os.path.join(out_dir, 'j')
    seed = rng.randint(1, 10 ** 6)
    t0 = time.time()
    with open(out + '.out', 'w', encoding='utf-8') as f:
        p = subprocess.run([sys.executable, '-u', os.path.join(HERE, 'replan.py'), world['stem'], str(K),
                            f'--from={world["stem"]}', f'--out={out}', '--mode=incremental', '--apply=strip',
                            f'--perturb={n_nets}', f'--perturb-tries={tries}', f'--seed={seed}',
                            '--coupled=census', '--grade=inproc'] + BENCH_ARGS,
                           cwd=HERE, env=child_env(env_extra), stdout=f, stderr=subprocess.STDOUT, text=True)
    stem = f'{out}_rp_k{K}'
    if not os.path.exists(stem + '.kicad_pcb'):
        return None, round(time.time() - t0)
    g, line = pl.grade(stem + '.kicad_pcb', nets_csv)
    moved = []
    try:
        txt = open(out + '.out', encoding='utf-8').read()
        moved = re.findall(r'^    (\S+): JUMPED', txt, re.M)
    except OSError:
        pass
    if not moved:
        return None, round(time.time() - t0)     # nothing moved: the same world, not a jump
    return {'stem': stem, 'grade': g,
            'origin': f'jump<{world["name"]}; near {moved}; seed {seed}>'}, round(time.time() - t0)


def cross_probe(A, Bw, K, out_dir, rng, nets_csv, par, env_extra=None):
    """A crossover through the probes (replan --cross): B's ends asked for
    on A's routed board for a random half of the nets whose ends differ,
    one probe each, the landing taken whatever its grade. Population
    members differ in a handful of nets (2-10 of 41 at K41, measured), so
    this is a few probes where the chain crossover was 214 s and landed
    worse than either parent."""
    out = os.path.join(out_dir, 'x')
    seed = rng.randint(1, 10 ** 6)
    t0 = time.time()
    args = [sys.executable, '-u', os.path.join(HERE, 'replan.py'), A['stem'], str(K),
            f'--from={A["stem"]}', f'--out={out}', f'--cross={Bw["stem"]}', f'--seed={seed}',
            '--mode=incremental', '--apply=strip', '--coupled=census', '--grade=inproc'] + BENCH_ARGS
    if par:
        args.append(f'--par={par}')
    with open(out + '.out', 'w', encoding='utf-8') as f:
        subprocess.run(args, cwd=HERE, env=child_env(env_extra), stdout=f, stderr=subprocess.STDOUT, text=True)
    stem = f'{out}_rp_k{K}'
    if not os.path.exists(stem + '.kicad_pcb'):
        return None, round(time.time() - t0)
    crossed, n_differ = [], None
    try:
        txt = open(out + '.out', encoding='utf-8').read()
        crossed = re.findall(r'^    (\S+): CROSSED', txt, re.M)
        m = re.search(r'cross \(seed \d+\): (\d+) of \d+ nets differ', txt)
        n_differ = int(m.group(1)) if m else None
    except OSError:
        pass
    if not crossed:
        return None, round(time.time() - t0)     # nothing taken from B: not a new world
    g, line = pl.grade(stem + '.kicad_pcb', nets_csv)
    return {'stem': stem, 'grade': g,
            'origin': f'cross<{A["name"]} x {Bw["name"]}; {len(crossed)} of {n_differ} differing net(s) from B; seed {seed}>'}, \
        round(time.time() - t0)


def key(w):
    g = w['grade']
    return (len(g[0]), g[1] != 0, g[2]) if g else (99, True, 10 ** 6)


def main():
    if len(ARGV) < 2 or not OPTS.get('seeds'):
        sys.exit(__doc__)
    tag, K = ARGV[0], int(ARGV[1])
    bench = OPTS.get('board', os.path.join(HERE, 'fb_t2q_fresh.kicad_pcb'))
    dest = OPTS.get('dest', 'DU1')
    BENCH_ARGS[:] = [f'--board={os.path.abspath(bench)}', f'--dest={dest}']
    POP = int(OPTS.get('pop', 4))
    GENS = int(OPTS.get('gens', 3))
    JUMPS = int(OPTS.get('jumps', 2))
    CROSS = int(OPTS.get('cross', 1))
    JOBS = int(OPTS.get('jobs', 2))
    # --jump=near (default, 2026-09-18): a jump through the probes, a few
    # nets moved to random other classes -- one probe each; --jump=chain:
    # the plan-level re-solve with class bans (665 s at K51, landed 84..141)
    JNETS = int(OPTS.get('jump-nets', 2))
    _mpar = re.search(r'--par=(\d+)', OPTS.get('descend', ''))
    PAR = int(_mpar.group(1)) if _mpar else 0
    # min-vias 2 (2026-09-18): at the frontier (K41 67, K51 83) no net carries
    # three lane vias any more, and a descent with the threshold at three
    # returns in 7 s having probed nothing. The 2-via one-dive nets are the
    # hypotheses now (91 -> 85 came from them with the climb menus).
    DESC = OPTS.get('descend', '--rounds=2 --worst=6 --probes=2 --min-vias=2 --coupled=census --grade=inproc')
    # THE MENUS (2026-09-18, the human-ends TEST): the same descent that stalls
    # at 91 on our plan takes the human's ends 87 -> 79, and the human's ends
    # differ from our menus only in the CLIMB classes -- destination climbs
    # (six nets) and source climbs -- both enumerated, both off. The descent
    # judges by the route and can afford them; a jump does not care where it
    # lands, so it re-plans on the full menus. Env for the two operators:
    DESC_ENV = dict(kv.split('=', 1) for kv in OPTS.get('descend-env', 'DST_CLIMB=2').split() if '=' in kv)
    JUMP_ENV = dict(kv.split('=', 1) for kv in OPTS.get('jump-env', 'DST_CLIMB=2 SRC_CLIMB=4').split() if '=' in kv)
    rng = random.Random(int(OPTS.get('seed', 1)))
    _rules.install_defaults()
    root = os.path.join(HERE, 'tmp', tag)
    os.makedirs(root, exist_ok=True)
    t_all = time.time()
    nets_all = coherent_nets(K, bench)
    nets_csv = ','.join(nets_all)
    pcb0 = parse_kicad_pcb(bench)
    byname0 = {n.name.split('/')[-1]: (i, n) for i, n in pcb0.nets.items()}
    ends0 = te.endpoints(pcb0, nets_all, byname0)
    dref = Counter(ends0[nm][2] for nm in nets_all if nm in ends0).most_common(1)[0][0]
    names = [nm for nm in nets_all if nm in ends0 and ends0[nm][2] == dref]
    log(f'evolve: K{K} tag {tag}; pop {POP}, {GENS} generation(s), {JUMPS} jump(s) + {CROSS} cross per '
        f'generation, jobs {JOBS}; descend: {DESC} {DESC_ENV}; cross probe; jump near '
        f'({JNETS} net(s)) env {JUMP_ENV}; '
        f'memo {"on" if pm.ENABLED else "OFF"} ({pm.MEMO_DIR}, code {pm.code_hash()})')
    # ---- the initial population
    pop = []
    g0 = os.path.join(root, 'g0')
    os.makedirs(g0, exist_ok=True)
    for i, spec in enumerate([s for s in OPTS['seeds'].split(',') if s]):
        stem = os.path.join(g0, f's{i}')
        g = import_seed(spec, K, nets_csv, stem)
        if g is None:
            log(f'  seed {spec}: not a routed world with a braid record -- skipped')
            continue
        pop.append({'name': f's{i}', 'stem': stem, 'grade': g, 'origin': f'seed {spec}'})
        log(f'  seed {i} {pl.fmt_g(g)} <- {spec}')
    if not pop:
        sys.exit('no seed world')
    pop.sort(key=key)
    best = pop[0]
    stage_world(os.path.join(root, f'best_k{K}'), best['stem'] + '_fo.kicad_pcb', best['stem'] + '.kicad_pcb')
    ledger = {'tag': tag, 'K': K, 'gens': [], 'pop0': [dict(w) for w in pop]}
    n_world = len(pop)
    for gen in range(1, GENS + 1):
        t_g = time.time()
        gdir = os.path.join(root, f'g{gen}')
        os.makedirs(gdir, exist_ok=True)
        log(f'\n=== generation {gen}: population ' + '; '.join(f'{w["name"]} {pl.fmt_g(w["grade"])}' for w in pop[:POP]))
        jobs = []
        for i, w in enumerate(pop[:POP]):
            d = os.path.join(gdir, f'w{i}')
            os.makedirs(d, exist_ok=True)
            jobs.append(('descend', w, d))
        for j in range(JUMPS):
            w = pop[min(j, len(pop) - 1)] if j < len(pop) else rng.choice(pop[:POP])
            d = os.path.join(gdir, f'j{j}')
            os.makedirs(d, exist_ok=True)
            jobs.append(('jump', w, d))
        for c in range(CROSS):
            if len(pop) < 2:
                break
            a, b = rng.sample(pop[:POP], 2)
            d = os.path.join(gdir, f'x{c}')
            os.makedirs(d, exist_ok=True)
            jobs.append(('cross', (a, b), d))

        def _run(job):
            kind, w, d = job
            try:
                if kind == 'descend':
                    nw, line, secs = descend(w, K, d, DESC, nets_csv, DESC_ENV)
                    return kind, w, nw, line, secs
                if kind == 'jump':
                    nw, secs = jump_near(w, K, d, JNETS, rng, nets_csv, JUMP_ENV)
                    return kind, w, nw, '', secs
                nw, secs = cross_probe(w[0], w[1], K, d, rng, nets_csv, PAR, JUMP_ENV)
                return kind, w, nw, '', secs
            except Exception as e:
                return kind, w, None, f'{type(e).__name__}: {str(e)[:120]}', 0
        new = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, JOBS)) as ex:
            for kind, w, nw, line, secs in ex.map(_run, jobs):
                src = w['name'] if isinstance(w, dict) else f'{w[0]["name"]} x {w[1]["name"]}'
                if nw is None:
                    log(f'  {kind:8s} {src:12s} -> nothing ({line}) ({secs} s)')
                    continue
                n_world += 1
                nw['name'] = f'g{gen}{kind[0]}{n_world}'
                # a descent that did not improve returns the same board: keep the parent
                if kind == 'descend' and not pl.better(nw['grade'], w['grade']):
                    log(f'  {kind:8s} {src:12s} -> {pl.fmt_g(nw["grade"])} (no gain) ({secs} s)')
                    continue
                new.append(nw)
                log(f'  {kind:8s} {src:12s} -> {nw["name"]} {pl.fmt_g(nw["grade"])} ({secs} s)')
        # ---- a JUMP or CROSSOVER world is judged only after it has DESCENDED:
        # its landing grade says nothing about its basin (measured: jumps land
        # at 102-140 with an open net and elitist selection dropped every one
        # before it was ever descended, which defeats the jump). One descent
        # each, side by side, in the same generation; the descended world
        # replaces the landing when better.
        fresh = [w for w in new if not w['origin'].startswith('descend')]
        if fresh:
            log(f'  descending the {len(fresh)} new world(s) before selection ...')
            jobs2 = []
            for i, w in enumerate(fresh):
                d = os.path.join(gdir, f'n{i}')
                os.makedirs(d, exist_ok=True)
                jobs2.append(('descend', w, d))
            with concurrent.futures.ThreadPoolExecutor(max_workers=max(1, JOBS)) as ex:
                for kind, w, nw, line, secs in ex.map(_run, jobs2):
                    if nw is None:
                        log(f'  {kind:8s} {w["name"]:12s} -> nothing ({line}) ({secs} s)')
                        continue
                    if not pl.better(nw['grade'], w['grade']):
                        log(f'  {kind:8s} {w["name"]:12s} -> {pl.fmt_g(nw["grade"])} (no gain) ({secs} s)')
                        continue
                    n_world += 1
                    nw['name'] = f'g{gen}d{n_world}'
                    nw['origin'] = f'descend<{w["name"]}: {w["origin"]}>'
                    new.append(nw)
                    log(f'  {kind:8s} {w["name"]:12s} -> {nw["name"]} {pl.fmt_g(nw["grade"])} ({secs} s)')
        # ---- selection: elitist, deduplicated by copper
        cand = pop + new
        seen = set()
        uniq = []
        for w in sorted(cand, key=key):
            try:
                fp_ = dedupe_boards.fingerprint(w['stem'] + '.kicad_pcb')
            except Exception:
                fp_ = w['stem']
            if fp_ in seen:
                continue
            seen.add(fp_)
            uniq.append(w)
        pop = uniq
        if key(pop[0]) < key(best):
            best = pop[0]
            stage_world(os.path.join(root, f'best_k{K}'), best['stem'] + '_fo.kicad_pcb', best['stem'] + '.kicad_pcb')
            log(f'  NEW BEST {best["name"]} {pl.fmt_g(best["grade"])} ({best["origin"]})')
        log(f'  generation {gen}: best {best["name"]} {pl.fmt_g(best["grade"])}; population '
            + '; '.join(f'{w["name"]} {pl.fmt_g(w["grade"])}' for w in pop[:POP]) + f' ({time.time() - t_g:.0f} s'
            + (f'; {pm.stats()["closed_hit"]} closed world(s) skipped' if pm.stats()['closed_hit'] else '') + ')')
        ledger['gens'].append({'gen': gen, 'pop': [dict(w) for w in pop[:POP]], 'best': dict(best),
                               'new': [dict(w) for w in new]})
        with open(os.path.join(root, f'evolve_k{K}.json'), 'w', encoding='utf-8') as f:
            json.dump(ledger, f, indent=1)
    log(f'\nevolve: best {pl.fmt_g(best["grade"])} <- {best["name"]} ({best["origin"]}) -> '
        f'{os.path.relpath(os.path.join(root, f"best_k{K}.kicad_pcb"), HERE)} ({time.time() - t_all:.0f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())

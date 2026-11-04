#!/usr/bin/env python3
"""Replay a recorded placement run with NO AI, place it with a tree, route
it by the corpus board's recorded chain, and grade it (#1486).

    placement_replay.py RUN [RUN ...] --out OUT [--src TREE] [--route all|judged|kept]
                        [--seeds LIST] [--modal] [--corpus DIR]

RUN is the run directory an AI pass left (`run_place_board.sh`): its
`placement_manifest.json` (every placement step, with the files each read,
by content -- `placement.run_record`) and `place_result.json` (the board and
its corpus set). For each RUN:

1. the steps before `place_simple` (the pile, the emitted intent, a fitted
   template, a pose set by hand) are replayed in OUT/<board>/work with the
   files each read restored as it read them, the original run directory
   remapped to that one and every tool taken from `--src`;
2. `place_simple` places the recorded seeds (or `--seeds`) with `--src`:
   on this machine, or with `--modal` on Modal (`placement_ab/place_modal.py`);
3. the seeds `--route` names are routed by the corpus board's recorded chain
   (`placement_ab/route_placed.py`, or `route_modal.py` with `--modal`):
   `all` every seed, `judged` the ones place_simple names for judging,
   `kept` the first of those (the one its objective rates best) -- and when
   no seed came out legal, so none is judged, the seeds cheapest first;
4. OUT/<board>/grade.json and the table printed at the end carry, per seed,
   the routed score (blocking, joins needed, vias, copper, segments) and the
   placement's own report (parts without a legal pose, floorplan errors,
   assembly, broken limits, seconds), and the human original's routed vias,
   copper and segments as the benchmark to approach.

Two arms are two OUT directories over the same RUNs with different `--src`
trees; `--compare A B` prints them paired per board and seed.
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
THIS_REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(THIS_REPO, 'py_placer'))

#: Where each recorded tool lives in a tree.
SCRIPTS = {
    'make_unplaced': '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py',
    'check_floorplan': 'py_tools/check_floorplan.py',
    'fit_template': 'py_tools/fit_template.py',
    'place_pose': 'py_placer/place_pose.py',
    'place_simple': 'py_placer/place_simple.py',
    'judge_seeds': 'py_placer/judge_seeds.py',
}
#: place_simple options the replay sets itself.
OWN = {'--seed': 1, '--seeds': 1, '--json': 1, '--jobs': 1, '--groups': 1, '--intent': 1,
       '--scattered': 1, '--clustered': 1, '--anchored': 1,
       '--clustered-seeds': 1, '--anchored-seeds': 1}


def remapper(old_run: str, new_run: str):
    def remap(p: str) -> str:
        return new_run + p[len(old_run):] if p.startswith(old_run) else p
    return remap


def _abs(p: str, cwd: str) -> str:
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(cwd, p))


def replay_prep(doc, old_run, work, src) -> dict:
    """Replay every step before the LAST place_simple in `work`; return
    that step with its paths remapped (None when there is none). The last
    is the placement the run judged: an agent that re-places (a relaxed
    intent, say) writes the same seeds over the first pass."""
    from placement import run_record
    remap = remapper(old_run, work)
    last = max((i for i, s in enumerate(doc['steps']) if s['tool'] == 'place_simple'),
               default=None)
    if last is None:
        return None
    for i, step in enumerate(doc['steps'][:last + 1]):
        cwd = remap(step['cwd'])
        os.makedirs(cwd, exist_ok=True)
        run_record.restore_inputs(doc, step, lambda p: remap(_abs(p, step['cwd'])))
        argv = [remap(_abs(t, step['cwd'])) if os.sep in t and not t.startswith('-') else t
                for t in step['argv']]
        for t in argv:                     # the directories the run had made
            if t.startswith(work + os.sep):
                os.makedirs(os.path.dirname(t), exist_ok=True)
        if i == last:
            return dict(step, argv=argv, cwd=cwd)
        if step['tool'] in ('place_simple', 'judge_seeds'):
            continue
        r = subprocess.run([sys.executable, '-u', '-X', 'utf8',
                            os.path.join(src, SCRIPTS[step['tool']])] + argv,
                           cwd=cwd, capture_output=True, text=True)
        with open(os.path.join(work, 'prep.log'), 'a', encoding='utf-8') as fh:
            fh.write(f"$ {step['tool']} {shlex.join(argv)}\nexit {r.returncode}\n"
                     f"{r.stdout[-2000:]}{r.stderr[-2000:]}\n")


def split_place(argv):
    """`(board, groups, intent, extra)` of a recorded place_simple argv."""
    pos, extra, opts, it = [], [], {}, iter(argv)
    for tok in it:
        if tok in OWN:
            opts[tok] = next(it, None)
        elif tok.startswith('-'):
            extra.append(tok)
        elif extra and extra[-1].startswith('--') and _takes_value(extra[-1]):
            extra.append(tok)
        else:
            pos.append(tok)
    return pos[0], opts.get('--groups'), opts.get('--intent'), extra


def recorded_starts(argv):
    """`(seeds, flags)` of a recorded place_simple run: the seeds it placed
    and the `--clustered-seeds` / `--anchored-seeds` that start each seed as
    the run started it. A run without --seed(s) numbers its own starts
    (6 scattered, then 6 clustered, by default), which an explicit seed list
    would lose: every seed would start scattered."""
    import place_simple
    r = place_simple.build_parser().parse_args(argv)
    if r.seeds or r.seed is not None:
        seeds, cs, an = r.seeds or str(r.seed), r.clustered_seeds, r.anchored_seeds
    else:
        sc, cl, na = (place_simple.SCATTERED_SEEDS if r.scattered is None else r.scattered,
                      place_simple.CLUSTERED_SEEDS if r.clustered is None else r.clustered,
                      place_simple.ANCHORED_SEEDS if r.anchored is None else r.anchored)
        seeds = f'0-{sc + cl + na - 1}'
        cs, an = place_simple.start_lists(sc, cl, na)
    return seeds, ((['--clustered-seeds', cs] if cs else [])
                   + (['--anchored-seeds', an] if an else []))


def _takes_value(flag):
    import place_simple                                      # noqa: F401
    ap = place_simple.build_parser()
    for act in ap._actions:
        if flag in act.option_strings:
            return act.nargs != 0
    return False


def place(board_name, step, seeds, out_dir, src, modal, corpus) -> str:
    """Place the seeds; return the directory holding s<seed>.kicad_pcb/.json."""
    board, groups, intent, extra = split_place(step['argv'])
    rec_seeds, starts = recorded_starts(step['argv'])
    seeds, extra = seeds or rec_seeds, starts + extra
    placed = os.path.join(out_dir, 'placed')
    if modal:
        board, groups, intent = (_abs(p, step['cwd']) for p in (board, groups, intent))
        cmd = [sys.executable, os.path.join(HERE, 'placement_ab', 'place_modal.py'),
               os.path.join(out_dir, 'arm'), f'{board_name}={board}:{groups}:{intent}', seeds,
               '--src', src, '--corpus', corpus] + (['--'] + extra if extra else [])
        subprocess.run(cmd, check=False)
        return os.path.join(out_dir, 'arm', board_name)
    os.makedirs(placed, exist_ok=True)
    subprocess.run([sys.executable, '-u', '-X', 'utf8', os.path.join(src, SCRIPTS['place_simple']),
                    board, os.path.join(placed, 's{seed}.kicad_pcb'), '--groups', groups,
                    '--intent', intent, '--seeds', seeds, '--jobs', '2',
                    '--json', os.path.join(placed, 's{seed}.json')] + extra,
                   cwd=step['cwd'], check=False,
                   stdout=open(os.path.join(out_dir, 'place.log'), 'w'), stderr=subprocess.STDOUT)
    return placed


def chosen(placed, which, judge=None):
    """The seeds `which` routes, from the placed boards and judge.json (or,
    with none, the `judge` seeds place_simple would name)."""
    seeds = sorted(int(f[1:-len('.kicad_pcb')]) for f in os.listdir(placed)
                   if f.startswith('s') and f.endswith('.kicad_pcb') and f[1:-10].isdigit())
    if which == 'all':
        return seeds
    j = os.path.join(placed, 'judge.json')
    reports = {s: json.load(open(p)) for s in seeds
               for p in [os.path.join(placed, f's{s}.json')] if os.path.exists(p)}
    if os.path.exists(j):
        judged = json.load(open(j)).get('judge')
    else:
        # each seed placed alone (on Modal) writes no judge.json: judge
        # its reports as place_simple judges a run of several seeds
        import place_simple
        judged = place_simple.judge_order(reports)[:judge or place_simple.JUDGED_SEEDS]
    if not judged:
        # no seed came out legal, so place_simple judged none: a stress test
        # still routes and grades them, cheapest first
        judged = sorted(seeds, key=lambda s: reports[s]['final']['cost'] if s in reports
                        else float('inf'))
    return judged[:1] if which == 'kept' else judged


def route(board_name, cset, placed, seeds, out_dir, src, modal, corpus) -> dict:
    """Route each seed by the corpus board's recorded chain; {seed: score}."""
    routed = os.path.join(out_dir, 'routed')
    if modal:
        subprocess.run([sys.executable, os.path.join(HERE, 'placement_ab', 'route_modal.py'),
                        os.path.dirname(placed), f'{board_name}={board_name}:{cset}',
                        ','.join(map(str, seeds)), '--out', routed, '--prefix', 'r',
                        '--full', '--src', src, '--corpus', corpus], check=False)
        name = lambda s: os.path.join(routed, f'r_{board_name}_s{s}')  # noqa: E731
    else:
        for s in seeds:
            subprocess.run([sys.executable, os.path.join(HERE, 'placement_ab', 'route_placed.py'),
                            os.path.join(placed, f's{s}.kicad_pcb'), board_name, cset,
                            os.path.join(routed, f's{s}'), '--repo', src, '--corpus', corpus],
                           check=False, capture_output=True)
        name = lambda s: os.path.join(routed, f's{s}')       # noqa: E731
    out = {}
    for s in seeds:
        p = os.path.join(name(s), 'score.json')
        out[s] = json.load(open(p)) if os.path.exists(p) else None
    return out


def grade(placed, scores) -> dict:
    rows = {}
    for s, sc in sorted(scores.items()):
        pj = os.path.join(placed, f's{s}.json')
        rep = json.load(open(pj)) if os.path.exists(pj) else {}
        q = (sc or {}).get('quality', {})
        rows[str(s)] = {
            'blocking': None if sc is None else sc.get('blocking'),
            'joins_needed': None if sc is None else sc.get('joins_needed'),
            'vias': q.get('vias'), 'copper_mm': q.get('copper_mm'), 'segments': q.get('segments'),
            'illegal': len(rep.get('illegal') or []),
            'floorplan_errors': (rep.get('floorplan') or {}).get('errors'),
            'assembly_blocking': (rep.get('assembly') or {}).get('blocking'),
            'claim_broken': (rep.get('final') or {}).get('claim_broken'),
            'decap_broken': (rep.get('final') or {}).get('decap_broken'),
            'place_seconds': sum((rep.get('seconds') or {}).values()) or None}
    return rows


def human(name, cset, out_dir, src, corpus):
    """The human original's routed quality (vias, copper, segments), by the
    same scorer: the benchmark to approach, never a pose to match."""
    orig = os.path.join(corpus, f'boards_set{cset}', f'{name}.kicad_pcb')
    out = os.path.join(out_dir, 'human.score.json')
    if not os.path.exists(out) and os.path.exists(orig):
        subprocess.run([sys.executable, '-X', 'utf8', os.path.join(src, 'py_tools', 'board_score.py'),
                        orig, '--json', out, '--quiet'], check=False, capture_output=True)
    if not os.path.exists(out):
        return None
    q = json.load(open(out)).get('quality', {})
    return {k: q.get(k) for k in ('vias', 'copper_mm', 'segments')}


def replay(run, a) -> dict:
    from placement import run_record
    doc = run_record.load(os.path.join(run, 'placement_manifest.json'))
    res = json.load(open(os.path.join(run, 'place_result.json')))
    name, cset = res['board'], str(res['set'])
    out_dir = os.path.join(os.path.abspath(a.out), name)
    work = os.path.join(out_dir, 'work')
    os.makedirs(work, exist_ok=True)
    step = replay_prep(doc, os.path.abspath(run), work, a.src)
    if step is None:
        return {'board': name, 'error': 'the record has no place_simple step'}
    placed = place(name, step, a.seeds, out_dir, a.src, a.modal, a.corpus)
    import place_simple
    seeds = chosen(placed, a.route, place_simple.build_parser().parse_args(step['argv']).judge)
    rows = grade(placed, route(name, cset, placed, seeds, out_dir, a.src, a.modal, a.corpus))
    doc = {'board': name, 'set': cset, 'src': a.src, 'route': a.route, 'seeds': rows,
           'human': human(name, cset, out_dir, a.src, a.corpus)}
    json.dump(doc, open(os.path.join(out_dir, 'grade.json'), 'w'), indent=1)
    return doc


KEYS = ('blocking', 'joins_needed', 'vias', 'copper_mm', 'segments', 'illegal',
        'floorplan_errors', 'assembly_blocking')


def table(docs):
    print(f"{'board':20s} {'seed':>4s} " + ' '.join(f'{k[:9]:>9s}' for k in KEYS))
    for d in docs:
        for s, row in d.get('seeds', {}).items():
            print(f"{d['board']:20s} {s:>4s} " + ' '.join(
                f"{'-' if row[k] is None else round(row[k]):>9}" for k in KEYS))
        h = d.get('human')
        if h:
            print(f"{d['board']:20s} {'hum':>4s} {'':>9} {'':>9} " + ' '.join(
                f"{'-' if h[k] is None else round(h[k]):>9}" for k in ('vias', 'copper_mm', 'segments')))


def compare(a_dir, b_dir):
    """Two arms paired per board and seed: blocking, then joins, then vias."""
    better = worse = same = 0
    for board in sorted(os.listdir(a_dir)):
        ga, gb = (os.path.join(x, board, 'grade.json') for x in (a_dir, b_dir))
        if not (os.path.exists(ga) and os.path.exists(gb)):
            continue
        sa, sb = json.load(open(ga))['seeds'], json.load(open(gb))['seeds']
        for s in sorted(set(sa) & set(sb), key=int):
            ka = tuple(sa[s].get(k) if sa[s].get(k) is not None else 1e9 for k in KEYS[:3])
            kb = tuple(sb[s].get(k) if sb[s].get(k) is not None else 1e9 for k in KEYS[:3])
            verdict = 'better' if kb < ka else 'worse' if kb > ka else 'same'
            better, worse, same = better + (verdict == 'better'), worse + (verdict == 'worse'), \
                same + (verdict == 'same')
            print(f"{board:20s} s{s:<3s} A {ka}  B {kb}  {verdict}")
    print(f"B against A, per board and seed: {better} better, {worse} worse, {same} same")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*', help='run directories an AI pass left')
    ap.add_argument('--out', help='the arm directory: OUT/<board>/...')
    ap.add_argument('--src', default=THIS_REPO, help='the tree to place and route with')
    ap.add_argument('--route', choices=('all', 'judged', 'kept'), default='judged',
                    help='which seeds to route (default: the judged ones)')
    ap.add_argument('--seeds', help='the seeds to place (default: the recorded ones)')
    ap.add_argument('--modal', action='store_true', help='place and route on Modal')
    ap.add_argument('--corpus', default=os.environ.get(
        'KICAD_STRESS_CORPUS', os.path.expanduser('~/Documents/kicad_stress_test')))
    ap.add_argument('--compare', nargs=2, metavar=('ARM_A', 'ARM_B'),
                    help='print two arm directories paired per board and seed, and exit')
    a = ap.parse_args(argv)
    if a.compare:
        compare(*a.compare)
        return 0
    if not a.runs or not a.out:
        ap.error('give RUN directories and --out (or --compare A B)')
    a.src = os.path.abspath(a.src)
    docs = [replay(os.path.abspath(r), a) for r in a.runs]
    table(docs)
    return 0


if __name__ == '__main__':
    sys.exit(main())

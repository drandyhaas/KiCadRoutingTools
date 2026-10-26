#!/usr/bin/env python3
"""Run the seed judge's routing chain over the seeds judge.json names, and score each.

    python3 -X utf8 py_placer/judge_seeds.py wk/<run>/judge.json \\
        --chain wk/<run>/judge_chain.txt --intent wk/<run>/intent.json [--jobs 1]

The placement skill's judging step (plan-pcb-placement 3.7): every seed
`place_simple` names in judge.json is routed by the SAME chain, its last board
copied out and scored, so the seeds compare. Before this, each run wrote its
own shell helper for it (#1327).

THE CHAIN is a text file, one command per line, the board's routing chain as
`plan-pcb-routing` writes it (pour, fanout, the decap clearance step,
route_diff, route.py). A line ending in `\\` continues on the next; blank
lines and `#` comments are skipped. Each command names its boards by
placeholder:

  {in}    the board the step reads: the seed's placement for the first step,
          then the board the last step wrote
  {out}   the board the step writes, `s<S>_route/<N>_<script>.kicad_pcb`; a
          step without `{out}` writes no board and leaves `{in}` as it is
  {dir}   the seed's route folder, `s<S>_route/` (for `--json-out {dir}/route.json`)
  {seed}  the seed number
  {run}   the folder judge.json is in

e.g.

    python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND --plane-layers B.Cu
    python3 -X utf8 py_router/route.py {in} {out} --nets '*' --track-width 0.127 \\
        --json-out {dir}/route.json

Every `route.py` and `route_diff.py` step runs with `--quick 2`, added when
the line has no `--quick` of its own, so the chain plan-pcb-routing writes for
the full route can be the file as it stands. A command runs from the current
folder (run from the repo root, as every skill command does), without a
shell: quote a glob (`'*'`) as you would on the command line, and it reaches
the tool unexpanded. `python3`/`python` at the head of a line is this
interpreter.

PER SEED S, from the run folder: `s<S>_route/` is made afresh, the seed's
placement copied in (`in.kicad_pcb`, siblings included), each step run in
order with its output in `s<S>_route/chain.log`, the last board copied out to
`s<S>_fast.kicad_pcb` (copy_board, siblings included) and scored by
`py_tools/quick_score.py` (`--intent` passed through) into
`s<S>_fast.score.json` -- the names the skill and the GUI's candidates table
read. A step's non-zero exit is logged and the chain goes on (route.py exits
non-zero when nets fail, and the fast route still judges); a step whose
`{out}` was not written stops that seed.

The assembly is graded too, against the board the seeds were placed from
(judge.json's `baseline`, which place_simple writes), as `--baseline`.

At the end the scored seeds are ranked by `quick_score.judge_key` (floorplan
errors plus the assembly count, blocking, joins_needed, then vias, copper_mm,
segments) and the best
is named; the kept board is the skill's to copy (its placement, never the
`--quick` board). `JSON_SUMMARY:` carries every seed's score and the ranking.

THE ARM. A chain is ONE arm of the routing recipe. Where `plan-pcb-routing`
prescribes two arms for the board's class (a 2-layer through-hole board
without buses: the defaults, and `--layer-costs 1.0 1.0 --via-cost 600`),
write one chain per arm and judge the seeds under each: the arms can pick
different seeds (#1383). Keep a seed that wins under the arm the final route
will use; where both arms will be routed, one that ranks first or second under
both.

LEVEL 2 IS A SCREEN, NOT A VERDICT. It skips the rescue tail and the plane
finalize, so its vias and copper differ from the full route's by a few percent
(measured -2% to +8% in copper per seed) and its blocking is decided by a
single open net when every seed routes. When the best seeds lie within that
spread -- the same placement defects, blocking within `CLOSE_BLOCKING` net of
the best and copper within `CLOSE_COPPER` of it -- the run NOTEs them, and
`--confirm close` (or `--confirm K` for the K best) routes those seeds again
at full level (`--quick` taken off every route step) into `s<S>_full_route/`,
`s<S>_full.kicad_pcb` and `s<S>_full.score.json`, and names the best of the
confirmed seeds by the same key.

Exit codes: 0 every seed scored; 4 a seed's chain wrote no board or its
score could not be read (named); 2 nothing run: judge.json unreadable or
naming no seed, a seed's placement missing, or a chain that reads no `{in}`,
writes no `{out}` or names an unknown placeholder.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'actor'}

import _path  # noqa: F401  (py_placer -> py_router/py_tools on sys.path)

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLACEHOLDERS = ('in', 'out', 'dir', 'seed', 'run')
#: `{name}` in a chain line; KiCad's overbar (`/~{RESET}`) is a net name, not one
_PLACEHOLDER_RE = re.compile(r'(?<!~)\{(\w+)\}')
#: the steps the judge runs at --quick 2 (plan-pcb-placement 3.7)
QUICK_SCRIPTS = ('route.py', 'route_diff.py')
#: Two seeds' level-2 scores are "close" when they carry the same placement
#: defects and the second's blocking is within this many nets, and its copper
#: within this fraction, of the best's: level 2 reads copper -2% to +8% off
#: the full route per seed, so a gap inside that cannot rank them (#1383).
CLOSE_BLOCKING = 1
CLOSE_COPPER = 0.10
#: `--confirm close` full-routes at most this many seeds (the best ones).
CONFIRM_MAX = 4


def read_chain(path):
    """[line] -- the chain file's commands, continuations joined, comments
    and blank lines dropped."""
    steps, cur = [], ''
    with open(path, encoding='utf-8') as fh:
        for raw in fh:
            line = raw.rstrip('\r\n')
            if not cur and (not line.strip() or line.lstrip().startswith('#')):
                continue
            if line.endswith('\\'):
                cur += line[:-1] + ' '
                continue
            cur += line
            steps.append(cur.strip())
            cur = ''
    if cur.strip():
        steps.append(cur.strip())
    return steps


def chain_problems(steps):
    """Why this chain cannot judge a seed, or [] when it can."""
    out = []
    if not steps:
        return ['the chain file holds no command']
    for i, s in enumerate(steps, 1):
        bad = sorted({n for n in _PLACEHOLDER_RE.findall(s) if n not in PLACEHOLDERS})
        if bad:
            out.append(f"step {i} names {', '.join('{' + n + '}' for n in bad)}; the "
                       f"placeholders are {', '.join('{' + n + '}' for n in PLACEHOLDERS)}")
        try:
            shlex.split(s)
        except ValueError as ex:
            out.append(f'step {i} does not parse as a command line ({ex}): {s}')
    if not any('{in}' in s for s in steps):
        out.append('no step reads {in}: the chain never sees the seed\'s placement')
    if not any('{out}' in s for s in steps):
        out.append('no step writes {out}: the chain leaves no board to score')
    return out


def _script(argv):
    """The basename of the first .py the command runs, or ''."""
    return next((os.path.basename(t) for t in argv if t.endswith('.py')), '')


def _strip_quick(argv):
    """`argv` without any `--quick` option: `--quick`, its level, `--quick=N`."""
    out, skip = [], False
    for i, t in enumerate(argv):
        if skip:
            skip = False
            continue
        if t.startswith('--quick='):
            continue
        if t == '--quick':
            skip = i + 1 < len(argv) and argv[i + 1].isdigit()
            continue
        out.append(t)
    return out


def step_argv(line, subst, full=False):
    """The command to run for one chain line: placeholders filled token by
    token (a path with spaces stays one argument), `python3`/`python` made
    this interpreter, and `--quick 2` added to a route.py / route_diff.py
    step with no `--quick` of its own. With `full`, every `--quick` is taken
    off a route step instead, so it runs at the full level (the confirming
    route, #1383). -> (argv, quick_added)"""
    argv = []
    for tok in shlex.split(line):
        for k, v in subst.items():
            tok = tok.replace('{' + k + '}', str(v))
        argv.append(tok)
    if argv and argv[0] in ('python3', 'python'):
        argv[0] = sys.executable
    elif argv and argv[0].endswith('.py'):
        argv = [sys.executable, '-X', 'utf8'] + argv
    added = False
    if full:
        return (_strip_quick(argv) if _script(argv) in QUICK_SCRIPTS else argv), False
    if _script(argv) in QUICK_SCRIPTS and not any(
            t == '--quick' or t.startswith('--quick=') for t in argv):
        argv += ['--quick', '2']
        added = True
    return argv, added


def judge_seed(run, s, steps, intent, lock, baseline=None, full=False):
    """Route seed `s` by the chain and score it. -> its result dict. With
    `full` the route steps run at the full level and the files are
    `s<S>_full_route/`, `s<S>_full.*` (`--confirm`)."""
    from copy_board import copy_board
    t0 = time.time()
    tag = 'full' if full else 'fast'
    placed = os.path.join(run, f's{s}.kicad_pcb')
    w = os.path.join(run, f's{s}_full_route' if full else f's{s}_route')
    shutil.rmtree(w, ignore_errors=True)
    os.makedirs(w)
    cur = os.path.join(w, 'in.kicad_pcb')
    copy_board(placed, cur)
    res = {'seed': s, 'steps': []}
    log_path = os.path.join(w, 'chain.log')
    with open(log_path, 'w', encoding='utf-8') as log:
        for i, line in enumerate(steps, 1):
            out = os.path.join(w, f"{i}_{os.path.splitext(_script(shlex.split(line)))[0] or 'step'}"
                                  f".kicad_pcb")
            argv, added = step_argv(line, {'in': cur, 'out': out, 'dir': w, 'seed': s, 'run': run},
                                    full=full)
            log.write(f"$ {' '.join(shlex.quote(a) for a in argv)}\n"
                      + ('# --quick 2 added: the judge routes every route.py / route_diff.py '
                         'step at level 2\n' if added else ''))
            log.flush()
            ts = time.time()
            rc = subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT).returncode
            log.write(f'# exit {rc} in {time.time() - ts:.1f} s\n')
            log.flush()
            res['steps'].append({'step': i, 'exit': rc, 'quick_added': added})
            if '{out}' in line:
                if not os.path.isfile(out):
                    res['failed'] = (f'step {i} ({_script(argv) or argv[0]}) exited {rc} and '
                                     f'wrote no board; see {log_path}')
                    break
                cur = out
    if 'failed' not in res:
        fast = os.path.join(run, f's{s}_{tag}.kicad_pcb')
        copy_board(cur, fast)
        score = os.path.join(run, f's{s}_{tag}.score.json')
        if os.path.exists(score):
            os.remove(score)
        qs = [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_tools', 'quick_score.py'),
              fast, '--json', score] + (['--intent', intent] if intent else []) \
            + (['--baseline', baseline] if baseline else [])
        with open(log_path, 'a', encoding='utf-8') as log:
            log.write(f"$ {' '.join(shlex.quote(a) for a in qs)}\n")
            log.flush()
            rc = subprocess.run(qs, stdout=log, stderr=subprocess.STDOUT).returncode
        try:
            with open(score, encoding='utf-8') as fh:
                res['score'] = json.load(fh)
        except (OSError, ValueError) as ex:
            res['failed'] = f'quick_score exited {rc} and wrote no score ({ex}); see {log_path}'
    res['seconds'] = round(time.time() - t0, 1)
    with lock:
        print(f'  seed {s}{" (full route)" if full else ""}: {_line(res)} '
              f'({res["seconds"]:.0f} s)', flush=True)
    return res


def close_seeds(scored):
    """The seeds (best first) whose level-2 scores lie within the quick/full
    spread of the best's: the same placement defects (floorplan errors plus
    assembly count), blocking at most `CLOSE_BLOCKING` above, copper at most
    `CLOSE_COPPER` above. `scored` is the ranked results with a `score`;
    the best is always first. [] when nothing is scored."""
    if not scored:
        return []
    def parts(r):
        sc = r['score']
        by = sc.get('blocking_by') if isinstance(sc.get('blocking_by'), dict) else {}
        q = sc.get('quality') if isinstance(sc.get('quality'), dict) else {}
        blk, cu = sc.get('blocking'), q.get('copper_mm')
        return ((by.get('floorplan') or 0) + (by.get('assembly') or 0),
                blk if isinstance(blk, (int, float)) else None,
                cu if isinstance(cu, (int, float)) else None)
    d0, b0, c0 = parts(scored[0])
    out = [scored[0]['seed']]
    for r in scored[1:]:
        d, b, c = parts(r)
        if d != d0 or b is None or b0 is None or b > b0 + CLOSE_BLOCKING:
            continue
        if c0 is not None and c is not None and c > c0 * (1.0 + CLOSE_COPPER):
            continue
        out.append(r['seed'])
    return out


def _line(res):
    if 'failed' in res:
        return f"FAILED: {res['failed']}"
    sc = res['score']
    q = sc.get('quality') or {}
    fp = (sc.get('blocking_by') or {}).get('floorplan')
    asm = (sc.get('blocking_by') or {}).get('assembly')
    return (f"blocking {sc.get('blocking')} joins_needed {sc.get('joins_needed')} "
            + (f"floorplan {fp} " if fp is not None else '')
            + (f"assembly {asm} " if asm is not None else '')
            + f"vias {q.get('vias')} copper_mm {q.get('copper_mm')} segments {q.get('segments')}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('judge', help="place_simple's judge.json; the seeds' boards are beside it")
    ap.add_argument('--chain', required=True,
                    help='the routing chain, one command per line with {in} {out} {dir} '
                         '{seed} {run} placeholders')
    ap.add_argument('--intent',
                    help="the run's intent, passed to quick_score: the floorplan graded with "
                         "the route (the skill passes it)")
    ap.add_argument('--seeds', metavar='LIST',
                    help='judge these seeds (0-3, or 0,1,5) instead of the ones judge.json '
                         'names')
    ap.add_argument('--confirm', metavar='K|close',
                    help="after the level-2 ranking, route these seeds again at the FULL level "
                         "and name the best of them: the K best ranked seeds, or `close`, the "
                         "best ones whose level-2 scores lie within the quick/full spread of "
                         "the best's (at most 4). Level 2 cannot tell such seeds apart (#1383)")
    ap.add_argument('--jobs', type=int, default=1,
                    help='seeds routed at once (default 1: a judging route takes up to about '
                         '1.3 GB on a small board and 5 GB on a 350-part one)')
    a = ap.parse_args(argv)
    if a.confirm is not None and a.confirm != 'close' and not (
            a.confirm.isdigit() and int(a.confirm) >= 1):
        print(f"judge_seeds: --confirm {a.confirm!r} is not `close` or a seed count of 1 or more",
              file=sys.stderr)
        return 2
    run = os.path.dirname(os.path.abspath(a.judge))
    try:
        with open(a.judge, encoding='utf-8') as fh:
            doc = json.load(fh)
        named = [int(s) for s in doc.get('judge') or ()]
    except (OSError, ValueError, TypeError, AttributeError) as ex:
        print(f'judge_seeds: cannot read {a.judge}: {ex}', file=sys.stderr)
        return 2
    if a.seeds:
        from place_simple import _seed_list
        try:
            named = _seed_list(a.seeds)
        except ValueError:
            print(f'judge_seeds: --seeds {a.seeds!r} is not a list like 0-3 or 0,1,5',
                  file=sys.stderr)
            return 2
    if not named:
        ill = doc.get('illegal') or []
        print(f"judge_seeds: {a.judge} names no seed to judge"
              + (f" (every placed seed left a part illegal: {ill})" if ill else '')
              + ". Nothing was routed: see plan-pcb-placement 3.6, 'When no seed can be "
              "judged'.", file=sys.stderr)
        return 2
    missing = [s for s in named if not os.path.isfile(os.path.join(run, f's{s}.kicad_pcb'))]
    if missing:
        print(f"judge_seeds: no placement for seed(s) {', '.join(map(str, missing))} in {run}. "
              f"Nothing was routed.", file=sys.stderr)
        return 2
    if a.intent and not os.path.isfile(a.intent):
        print(f'judge_seeds: --intent {a.intent} is not a file. Nothing was routed.',
              file=sys.stderr)
        return 2
    try:
        steps = read_chain(a.chain)
    except OSError as ex:
        print(f'judge_seeds: cannot read --chain {a.chain}: {ex}', file=sys.stderr)
        return 2
    bad = chain_problems(steps)
    if bad:
        print('judge_seeds: the chain cannot judge a seed: ' + '; '.join(bad)
              + '. Nothing was routed.', file=sys.stderr)
        return 2
    print(f"judge_seeds: seeds {', '.join(map(str, named))}, {len(steps)} step(s) each, "
          f"{max(1, a.jobs)} at a time")
    if not a.intent:
        print('  NOTE: no --intent: quick_score leaves the floorplan ungraded')
    # the board the seeds were placed from, which place_simple records: the
    # assembly is graded against it, as board_score --baseline grades it
    baseline = doc.get('baseline') if isinstance(doc, dict) else None
    if baseline and not os.path.isfile(baseline):
        print(f'  NOTE: judge.json names baseline {baseline}, which is not a file: '
              f'quick_score leaves the assembly ungraded')
        baseline = None
    elif not baseline:
        print('  NOTE: judge.json names no baseline: quick_score leaves the assembly ungraded')
    lock = threading.Lock()
    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        results = list(pool.map(lambda s: judge_seed(run, s, steps, a.intent, lock, baseline),
                                named))
    from quick_score import judge_key
    scored = sorted((r for r in results if 'score' in r), key=lambda r: judge_key(r['score']))
    failed = [r for r in results if 'failed' in r]
    if scored:
        print('judge_seeds: ranked by quick_score.judge_key (floorplan errors + assembly, '
              'blocking, joins_needed, vias, copper_mm, segments):')
        for r in scored:
            print(f"  seed {r['seed']}: {_line(r)}")
        print(f"judge_seeds: best seed {scored[0]['seed']}: keep its placement "
              f"s{scored[0]['seed']}.kicad_pcb (never the --quick board)")
    close = close_seeds(scored)
    confirmed, best = [], (scored[0]['seed'] if scored else None)
    if scored and a.confirm is not None:
        pick = (close[:CONFIRM_MAX] if a.confirm == 'close'
                else [r['seed'] for r in scored][:int(a.confirm)])
        if len(pick) < 2 and a.confirm == 'close':
            print('judge_seeds: --confirm close: no other seed lies within the quick/full '
                  'spread of the best; nothing to confirm')
        else:
            print(f"judge_seeds: confirming seed(s) {', '.join(map(str, pick))} with a full "
                  f"route (--quick taken off every route step)")
            with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
                full = list(pool.map(lambda s: judge_seed(run, s, steps, a.intent, lock,
                                                          baseline, full=True), pick))
            confirmed = sorted((r for r in full if 'score' in r),
                               key=lambda r: judge_key(r['score']))
            # a seed whose confirm route fails was still judged at level 2
            failed += [dict(r, stage='full-route confirm') for r in full
                       if 'failed' in r]
            if confirmed:
                print('judge_seeds: confirmed ranking (full route, same key):')
                for r in confirmed:
                    print(f"  seed {r['seed']}: {_line(r)}")
                best = confirmed[0]['seed']
                note = '' if best == scored[0]['seed'] else \
                    f" (level 2 had put seed {scored[0]['seed']} first)"
                print(f"judge_seeds: best seed {best} by the full route{note}: keep its "
                      f"placement s{best}.kicad_pcb")
    elif len(close) > 1:
        print(f"  NOTE: seeds {', '.join(map(str, close))} lie within the quick/full spread of "
              f"the best (same defects, blocking within {CLOSE_BLOCKING}, copper within "
              f"{CLOSE_COPPER:.0%}): level 2 cannot rank them. Re-run with --confirm close "
              f"to route them at the full level")
    for r in failed:
        print(f"  WARNING: seed {r['seed']} "
              f"{r['stage'] + ' failed' if r.get('stage') else 'not judged'}: "
              f"{r['failed']}")
    print('JSON_SUMMARY: ' + json.dumps({
        'seeds': {str(r['seed']): ({'failed': r['failed']} if 'failed' in r else {
            'blocking': r['score'].get('blocking'),
            'joins_needed': r['score'].get('joins_needed'),
            'floorplan': (r['score'].get('blocking_by') or {}).get('floorplan'),
            **{k: (r['score'].get('quality') or {}).get(k)
               for k in ('vias', 'copper_mm', 'segments')}})
            | {'seconds': r['seconds']} for r in results},
        'ranking': [r['seed'] for r in scored],
        'close': close,
        'confirmed': [r['seed'] for r in confirmed],
        'best': best}))
    return 4 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

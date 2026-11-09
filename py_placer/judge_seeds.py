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

When `list_nets.py --diff-pairs` finds pairs on the board, the chain carries
a `route_diff.py` step before route.py, the one `identify-diff-pairs` /
`plan-pcb-routing` give the board. A chain without one routes each pair as two
single nets: the run WARNs, naming the pair count, and judges on (#1472).

Every `route.py` and `route_diff.py` step runs with `--quick 2`, added when
the line has no `--quick` of its own, so ONE chain file in this format serves
both the judging route and the full route (`--full`, `--confirm`, which take
every `--quick` off): its route steps need no level of their own. It is the
board's plan-pcb-routing chain with its boards named by placeholder; that
skill's own `<board>_plan.sh` names them by path and begins with `cd`, and is
not this format (no step reads {in}). `route_plan_check.py` proves either
file before it runs (#1592). A command runs from the current
folder (run from the repo root, as every skill command does), without a
shell: quote a glob (`'*'`) as you would on the command line, and it reaches
the tool unexpanded. `python3`/`python` at the head of a line is this
interpreter. A leading `NAME=value` prefix (`KICAD_SMOOTH_ROUTE=1 python3
...`, the form plan-pcb-routing puts on a route step) is that step's
environment, over the judge's own, and no other step's (#1497).

PER SEED S, from the run folder: `s<S>_route/` is made afresh, the seed's
placement copied in (`in.kicad_pcb`, siblings included), each step run in
order with its output in `s<S>_route/chain.log` and its board in
`<N>_<script>.kicad_pcb`, the copy and every step written as run to
`s<S>_route/redo_commands.sh` (redo_record's format: a manifest of that route
alone, which `tests/stress/redo_stress_test.py` replays, whatever
REDO_MANIFEST the caller exported -- the tools still self-record into that
one as before), the last board copied out to
`s<S>_fast.kicad_pcb` (copy_board, siblings included) and scored by
`py_tools/quick_score.py` (`--intent` passed through) into
`s<S>_fast.score.json` -- the names the skill and the GUI's candidates table
read. A step's non-zero exit is logged and the chain goes on (route.py exits
non-zero when nets fail, and the fast route still judges); a step whose
`{out}` was not written stops that seed.

The assembly is graded too, against the board the seeds were placed from
(judge.json's `baseline`, which place_simple writes), as `--baseline`.

THE CLEAN SEEDS. With no `--seeds`, a seed judge.json grades with a
floorplan error or a NOT BUILDABLE assembly (its `floorplan` / `assembly`
count above 0) is skipped and named: no route clears either, so it ranks
last whatever its route (#1645). judge.json still names it (#1296). When
every named seed is one, nothing is routed. `--seeds` routes exactly what it
names, an unclean seed included (one repaired by hand).

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
both. This tool works in the folder judge.json is in and makes its files afresh
there, so each arm needs a folder of its own, or the last leaves the only boards,
logs and scores (plan-pcb-placement 3.7's recipe, #1666).

LEVEL 2 IS A SCREEN, NOT A VERDICT. It skips the rescue tail and the plane
finalize, so its vias and copper differ from the full route's by a few percent
(measured -2% to +8% in copper per seed) and its blocking is decided by a
single open net when every seed routes. When the best seeds lie within that
spread -- the same placement defects, blocking within `CLOSE_BLOCKING` net of
the best and copper within `CLOSE_COPPER` of it -- the run NOTEs them, and
`--confirm close` (or `--confirm K` for the K best) routes those seeds again
at full level (`--quick` taken off every route step) into `s<S>_full_route/`,
`s<S>_full.kicad_pcb` and `s<S>_full.score.json`, and names the best of the
confirmed seeds by the same key. `--confirm close` takes at most
`CONFIRM_MAX` (4) seeds, the best of the close ones; when more are close,
the NOTE and the confirming run name the ones it leaves out and the options
that confirm every one (`--seeds <the close seeds> --confirm K`, #1473). The
re-run routes its seeds at level 2 again before confirming them: no score
on disk records the chain, intent and code it was routed with.

THE FULL ROUTE, WITH NO LEVEL-2 PASS. `--full` routes the seeds at the full
level only, into the `--confirm` names (`s<S>_full_route/`,
`s<S>_full.kicad_pcb`, `s<S>_full.score.json`), and ranks them by that route.
`--seeds S --full` is the full route of one board by the chain: per-step
boards, `chain.log` and the route's own `redo_commands.sh` in
`s<S>_full_route/`, the routed board in `s<S>_full.kicad_pcb` (#1592).

Exit codes: 0 every seed scored; 4 a seed's chain wrote no board or its
score could not be read (named); 2 nothing run: judge.json unreadable or
naming no seed (with no `--seeds`, no seed without a placement defect), a seed's placement missing, `--full` with `--confirm`, or a
chain that reads no `{in}`, writes no `{out}`, names an unknown placeholder
or carries a shell builtin (`cd`, `export`: no shell runs the line).
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

if getattr(sys, 'frozen', False):
    from krt_standalone.process import run as run_process
else:
    run_process = subprocess.run

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PLACEHOLDERS = ('in', 'out', 'dir', 'seed', 'run')
#: `{name}` in a chain line; KiCad's overbar (`/~{RESET}`) is a net name, not one
_PLACEHOLDER_RE = re.compile(r'(?<!~)\{(\w+)\}')
#: the steps the judge runs at --quick 2 (plan-pcb-placement 3.7)
QUICK_SCRIPTS = ('route.py', 'route_diff.py')
#: shell builtins a plan.sh carries (`cd <repo>` first, #1337) and a chain
#: cannot: the judge runs each line as a process, with no shell (#1592)
SHELL_ONLY = ('cd', 'export', 'set', 'source', '.', 'pushd', 'popd', 'unset')
#: Two seeds' level-2 scores are "close" when they carry the same placement
#: defects and the second's blocking is within this many nets, and its copper
#: within this fraction, of the best's: level 2 reads copper -2% to +8% off
#: the full route per seed, so a gap inside that cannot rank them (#1383).
CLOSE_BLOCKING = 1
CLOSE_COPPER = 0.10
#: `--confirm close` full-routes at most this many seeds (the best ones); the
#: ones it leaves out are named, with the options that confirm them (#1473).
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


def placeholders(text):
    """The placeholder names `text` uses (`{in}` -> 'in'), known or not.
    KiCad's overbar (`/~{RESET}`) is a net name, not one."""
    return set(_PLACEHOLDER_RE.findall(text))


def is_chain(steps):
    """Whether these command lines are a chain in this format: one of them
    names a placeholder. A `<board>_plan.sh` names its boards by path, so it
    names none; route_plan_check reads a file as a chain by this (#1592)."""
    return any(placeholders(s) & set(PLACEHOLDERS) for s in steps)


def chain_problems(steps):
    """Why this chain cannot judge a seed, or [] when it can."""
    out = []
    if not steps:
        return ['the chain file holds no command']
    for i, s in enumerate(steps, 1):
        bad = sorted(placeholders(s) - set(PLACEHOLDERS))
        if bad:
            out.append(f"step {i} names {', '.join('{' + n + '}' for n in bad)}; the "
                       f"placeholders are {', '.join('{' + n + '}' for n in PLACEHOLDERS)}")
        try:
            head = step_env(s, split=True)[1][:1]
        except ValueError as ex:
            out.append(f'step {i} does not parse as a command line ({ex}): {s}')
            continue
        if head and head[0] in SHELL_ONLY:
            # run without a shell, `cd` is no program: FileNotFoundError on
            # Windows and Linux, a no-op /usr/bin/cd on macOS -- never a cwd
            out.append(f"step {i} is the shell's `{head[0]}`, and the judge runs each line "
                       f"without a shell, from the current folder (the repo root): drop it")
    if not any('{in}' in s for s in steps):
        out.append('no step reads {in}: the chain never sees the seed\'s placement')
    if not any('{out}' in s for s in steps):
        out.append('no step writes {out}: the chain leaves no board to score')
    return out


def _script(argv):
    """The basename of the first .py the command runs, or ''."""
    return next((os.path.basename(t) for t in argv if t.endswith('.py')), '')


def unpaired_diff_pairs(board, steps):
    """The differential pairs `list_nets.find_differential_pairs` finds by
    name on `board` when no chain step runs route_diff.py, else [] (#1472):
    without that step route.py routes each pair as two single nets."""
    if any(_script(shlex.split(s)) == 'route_diff.py' for s in steps):
        return []
    from kicad_parser import parse_kicad_pcb
    from list_nets import find_differential_pairs
    return find_differential_pairs(parse_kicad_pcb(board))


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


def step_env(line, split=False):
    """The env knobs a chain line sets for its step alone -- its leading
    `NAME=value` tokens, the prefix plan-pcb-routing puts on a route step and
    the redo recorder writes (#1497) -- as a dict; with `split`, `(env, the
    command's tokens after them)`. Handed to the process as argv, such a
    prefix was run as a program and the judge died on PermissionError."""
    from redo_record import split_env_prefix
    env, rest = split_env_prefix(shlex.split(line))
    return (env, rest) if split else env


def step_argv(line, subst, full=False):
    """The command to run for one chain line: placeholders filled token by
    token (a path with spaces stays one argument), `python3`/`python` made
    this interpreter, and `--quick 2` added to a route.py / route_diff.py
    step with no `--quick` of its own. With `full`, every `--quick` is taken
    off a route step instead, so it runs at the full level (the confirming
    route, #1383). A leading `NAME=value` prefix is not part of the command:
    it is the step's environment (`step_env`, #1497). -> (argv, quick_added)"""
    argv = []
    for tok in step_env(line, split=True)[1]:
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


def record_step(manifest, argv, knobs=None):
    """One command into a route folder's own manifest (#1592), in the format
    `redo_record` writes a self-recorded step in: this interpreter spelled
    `python3`, the run's cwd, and the knobs the step runs with (the judge's
    own and the line's prefix) as its `NAME=value` prefix."""
    from env_knobs import is_replay_knob
    from redo_record import write_record
    cmd = ['python3'] + list(argv[1:]) if argv and argv[0] == sys.executable else list(argv)
    env = dict(os.environ, **(knobs or {}))
    write_record(manifest, cmd, os.getcwd(),
                 {k: v for k, v in sorted(env.items()) if is_replay_knob(k)})


def judge_seed(run, s, steps, intent, lock, baseline=None, full=False):
    """Route seed `s` by the chain and score it. -> its result dict. With
    `full` the route steps run at the full level and the files are
    `s<S>_full_route/`, `s<S>_full.*` (`--confirm`, `--full`). The route
    folder gets the per-step boards, `chain.log`, and `redo_commands.sh`: the
    copy of the placement and every step, as run -- a manifest of this route
    alone, whatever REDO_MANIFEST the caller exported (#1592)."""
    from copy_board import copy_board
    t0 = time.time()
    tag = 'full' if full else 'fast'
    placed = os.path.join(run, f's{s}.kicad_pcb')
    w = os.path.join(run, f's{s}_full_route' if full else f's{s}_route')
    shutil.rmtree(w, ignore_errors=True)
    os.makedirs(w)
    cur = os.path.join(w, 'in.kicad_pcb')
    copy_board(placed, cur)
    manifest = os.path.join(w, 'redo_commands.sh')
    record_step(manifest, [sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_router', 'copy_board.py'), placed, cur])
    res = {'seed': s, 'steps': []}
    log_path = os.path.join(w, 'chain.log')
    with open(log_path, 'w', encoding='utf-8') as log:
        for i, line in enumerate(steps, 1):
            out = os.path.join(w, f"{i}_{os.path.splitext(_script(shlex.split(line)))[0] or 'step'}"
                                  f".kicad_pcb")
            argv, added = step_argv(line, {'in': cur, 'out': out, 'dir': w, 'seed': s, 'run': run},
                                    full=full)
            knobs = step_env(line)
            record_step(manifest, argv, knobs)
            log.write(f"$ {' '.join(shlex.quote(a) for a in [f'{k}={v}' for k, v in knobs.items()] + argv)}\n"
                      + ('# --quick 2 added: the judge routes every route.py / route_diff.py '
                         'step at level 2\n' if added else ''))
            log.flush()
            ts = time.time()
            rc = run_process(argv, stdout=log, stderr=subprocess.STDOUT,
                             env=dict(os.environ, **knobs) if knobs else None).returncode
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
            rc = run_process(qs, stdout=log, stderr=subprocess.STDOUT).returncode
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


def confirm_all(close, ranking):
    """The options that confirm every close seed, where `--confirm close`
    takes only the `CONFIRM_MAX` best of them: `--confirm K` when the close
    seeds are the K best ranked, else `--seeds <them> --confirm K` (#1473)."""
    k = len(close)
    if ranking[:k] == close:
        return f'--confirm {k}'
    return f"--seeds {','.join(map(str, close))} --confirm {k}"


def placement_defects(doc, named):
    """`{seed: why}` for the seeds of `named` whose placement judge.json
    grades with a defect no route clears: a floorplan error or a NOT
    BUILDABLE assembly (its `floorplan` / `assembly` count above 0, #1645).
    place_simple ranks such seeds last rather than dropping them (#1296), so
    the judge routes them only when named by `--seeds`. A seed judge.json
    did not grade (None, or no entry) is not one."""
    fp, asm = doc.get('floorplan') or {}, doc.get('assembly') or {}
    out = {}
    for s in named:
        e, b = fp.get(str(s)), asm.get(str(s))
        parts = ([f'{e} floorplan error(s)'] if e else []) + (
            [f'assembly {b} (NOT BUILDABLE)'] if b else [])
        if parts:
            out[s] = ', '.join(parts)
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
                         f"the best's (at most {CONFIRM_MAX}; the rest are named, with the "
                         f"--seeds ... --confirm K that confirms them). Level 2 cannot tell such "
                         f"seeds apart (#1383)")
    ap.add_argument('--full', action='store_true',
                    help="route the seeds at the FULL level only (--quick taken off every route "
                         "step) and rank them by that route: no level-2 pass. With --seeds S, the "
                         "full route of one seed by the chain, into s<S>_full_route/ (per-step "
                         "boards, chain.log, redo_commands.sh) and s<S>_full.kicad_pcb (#1592)")
    ap.add_argument('--jobs', type=int, default=1,
                    help='seeds routed at once (default 1: a judging route takes up to about '
                         '1.3 GB on a small board and 5 GB on a 350-part one)')
    a = ap.parse_args(argv)
    from placement import run_record              # the placement record (#1486)
    run_record.record('judge_seeds', argv)
    if a.confirm is not None and a.confirm != 'close' and not (
            a.confirm.isdigit() and int(a.confirm) >= 1):
        print(f"judge_seeds: --confirm {a.confirm!r} is not `close` or a seed count of 1 or more",
              file=sys.stderr)
        return 2
    if a.full and a.confirm is not None:
        print('judge_seeds: --full routes every seed at the full level already; --confirm '
              're-routes level-2 seeds at it. Pass one of them. Nothing was routed.',
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
    if not a.seeds:
        unclean = placement_defects(doc, named)
        if unclean:
            why = '; '.join(f"seed {s}: {t}" for s, t in unclean.items())
            named = [s for s in named if s not in unclean]
            if not named:
                print(f"judge_seeds: every seed {a.judge} names carries a placement defect "
                      f"no route clears ({why}). Nothing was routed: see plan-pcb-placement "
                      f"3.6, 'When no seed can be judged' (--seeds routes a seed you name, "
                      f"e.g. one repaired by hand).", file=sys.stderr)
                return 2
            print(f"judge_seeds: skipping seed(s) {', '.join(map(str, unclean))}, which "
                  f"cannot be DONE as placed ({why}); --seeds routes a seed you name")
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
          f"{max(1, a.jobs)} at a time"
          + (', at the full level (--full: no level-2 pass)' if a.full else ''))
    if not a.intent:
        print('  NOTE: no --intent: quick_score leaves the floorplan ungraded')
    pairs = unpaired_diff_pairs(os.path.join(run, f's{named[0]}.kicad_pcb'), steps)
    if pairs:
        print(f"  WARNING: the board has {len(pairs)} differential pair(s) by name (e.g. "
              f"{pairs[0][0]} / {pairs[0][1]}) and no chain step runs route_diff.py: route.py "
              f"routes each pair as two single nets, so the seeds are ranked without their "
              f"pairs. Add the route_diff.py step identify-diff-pairs / plan-pcb-routing give "
              f"the board, before route.py")
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
        results = list(pool.map(lambda s: judge_seed(run, s, steps, a.intent, lock, baseline,
                                                     full=a.full), named))
    from quick_score import judge_key
    scored = sorted((r for r in results if 'score' in r), key=lambda r: judge_key(r['score']))
    failed = [r for r in results if 'failed' in r]
    if scored:
        print('judge_seeds: ranked by quick_score.judge_key (floorplan errors + assembly, '
              'blocking, joins_needed, vias, copper_mm, segments)'
              + (', full route' if a.full else '') + ':')
        for r in scored:
            print(f"  seed {r['seed']}: {_line(r)}")
        b = scored[0]['seed']
        print(f"judge_seeds: best seed {b}"
              + (f" by the full route: keep its placement s{b}.kicad_pcb; its routed board is "
                 f"s{b}_full.kicad_pcb" if a.full else
                 f": keep its placement s{b}.kicad_pcb (never the --quick board)"))
    # the quick/full spread is level 2's: a full route has none to settle
    close = [] if a.full else close_seeds(scored)
    ranking = [r['seed'] for r in scored]
    # the close seeds `--confirm close` leaves out, past its cap
    left = close[CONFIRM_MAX:] if a.confirm in (None, 'close') else []
    capped = (f"--confirm close takes at most {CONFIRM_MAX}, the {CONFIRM_MAX} best of the "
              f"{len(close)} close seeds, and leaves out {', '.join(map(str, left))} "
              f"({confirm_all(close, ranking)} confirms every one)") if left else ''
    confirmed, best = [], (scored[0]['seed'] if scored else None)
    if scored and a.confirm is not None:
        pick = close[:CONFIRM_MAX] if a.confirm == 'close' else ranking[:int(a.confirm)]
        if len(pick) < 2 and a.confirm == 'close':
            print('judge_seeds: --confirm close: no other seed lies within the quick/full '
                  'spread of the best; nothing to confirm')
        else:
            print(f"judge_seeds: confirming seed(s) {', '.join(map(str, pick))} with a full "
                  f"route (--quick taken off every route step)" + (f": {capped}" if capped else ''))
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
                if capped:
                    print(f"  NOTE: best of the confirmed seeds only: {capped}")
    elif len(close) > 1:
        print(f"  NOTE: seeds {', '.join(map(str, close))} lie within the quick/full spread of "
              f"the best (same defects, blocking within {CLOSE_BLOCKING}, copper within "
              f"{CLOSE_COPPER:.0%}): level 2 cannot rank them. Re-run with --confirm close "
              f"to route them at the full level"
              + (f": {capped}" if capped else ''))
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
        'level': 'full' if a.full else 2,
        'ranking': [r['seed'] for r in scored],
        'close': close,
        'confirmed': [r['seed'] for r in confirmed],
        'best': best}))
    return 4 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

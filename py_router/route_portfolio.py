#!/usr/bin/env python3
"""Route one step several ways side by side and keep the best board (#1941).

    python3 py_router/route_portfolio.py in.kicad_pcb out.kicad_pcb -- <route.py arguments>
    python3 py_router/route_portfolio.py in.kicad_pcb out.kicad_pcb \\
        --variants default inside_out --variant deep='--max-ripup 6' \\
        --jobs 3 -- --nets '*' --clearance-ceiling 0.15

A route.py run is deterministic and uses one core. This runs one route.py
per VARIANT -- the same arguments plus a few flags that change the search --
each as its own process, `--jobs` at a time, and ships the attempt the
project's own score ranks best. It makes no attempt faster and changes no
step: it is a driver around route.py, run only when asked for.

Variants. `--variants` names built-in ones (`--list-variants` prints them;
all of them when no variant is given) and `--variant NAME=FLAGS` adds one of
your own, FLAGS being route.py flags in one quoted string. They run in the
order given, and that order is the last tie-break. A variant's flags follow
the shared arguments, so where both set a flag the variant's wins.

Every attempt has a folder of its own, `<output>.portfolio/<NN>_<name>/`:
`in.kicad_pcb` (copy_board: the board and its siblings), `out.kicad_pcb`
with the project route.py writes beside it, `route.log` and `score.json`. No
two attempts read or write one path: a shared argument that names a file
route.py writes (`--json-out`, `--preview-png`) must carry `{dir}`, which
becomes the attempt's folder, and `--output` is refused.

The best. Each board an attempt wrote is scored by `board_score.py`
(`--baseline` the input, check_connected's KiCad cross-check off so a board
always scores the same) and the attempts are ranked by
`quick_score.judge_key`: blocking, then joins_needed, then vias, copper_mm,
segments -- then the variant's place in the list. Never by which finished
first. An attempt that wrote no board is not a candidate; one whose board
could not be scored ranks after every scored one. The winner and its
siblings are copied to the output path, so the shipped board is that
attempt's own file.

What is kept. `<output>.portfolio/portfolio.json` and the `PORTFOLIO_JSON:`
line carry every attempt's command, exit code, seconds and score, the
ranking and the winner. `<output>.portfolio/redo_commands.sh` is a manifest
of every attempt as run. With REDO_MANIFEST set, the WINNING route.py
command, input to output, is what the run's manifest records, so a replay is
one route.py call; the attempts themselves record nothing there. The losers'
boards are removed unless `--keep-losers`; their logs and scores stay.

Exit codes: 0 a board shipped; 1 no attempt wrote a board (nothing is
written to the output path); 2 bad arguments.
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing'], 'kind': 'actor'}

import argparse
import contextlib
import io
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

ROUTE = os.path.join(HERE, 'route.py')
COPY = os.path.join(HERE, 'copy_board.py')
SCORE = os.path.join(REPO, 'py_tools', 'board_score.py')

#: The built-in variants, in tie-break order: a name and the route.py flags
#: it adds. `default` adds none, so the portfolio never ships a board the
#: score ranks below the plain step's.
BUILTIN_VARIANTS = (
    ('default', ()),
    ('inside_out', ('--ordering', 'inside_out')),
    ('original', ('--ordering', 'original')),
    ('mps_reverse', ('--mps-reverse-rounds',)),
    ('backward', ('--direction', 'backward')),
)

#: route.py flags whose value is a file the run WRITES: shared between
#: attempts only with `{dir}` in the value.
WRITES_FILE = ('--json-out', '--preview-png')
#: route.py's other spelling of the output board: the driver names each
#: attempt's output itself.
REFUSED = ('--output',)

#: GB one route is allowed for when the default `--jobs` is chosen: the most
#: judge_seeds records for a route of a large board.
ROUTE_GB = 5.0

_NAME = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')


class BadArguments(Exception):
    pass


def default_jobs(n_variants, cores=None, mem_gb=None):
    """Attempts run at a time when `--jobs` is not given: half the cores,
    no more than fit in memory at ROUTE_GB each, no more than there are
    variants, and at least one."""
    if cores is None:
        cores = os.cpu_count() or 2
    if mem_gb is None:
        try:
            mem_gb = os.sysconf('SC_PAGE_SIZE') * os.sysconf('SC_PHYS_PAGES') / 2 ** 30
        except (ValueError, OSError, AttributeError):
            mem_gb = None
    jobs = max(1, cores // 2)
    if mem_gb:
        jobs = min(jobs, max(1, int(mem_gb // ROUTE_GB)))
    return max(1, min(n_variants, jobs))


def read_variants(names, custom):
    """[(name, [flags])] in run order: the built-ins NAMES picks (all of
    them when neither argument names any), then each `NAME=FLAGS` of
    CUSTOM."""
    builtin = dict(BUILTIN_VARIANTS)
    if not names and not custom:
        names = [n for n, _ in BUILTIN_VARIANTS]
    out = []
    for n in names or ():
        if n not in builtin:
            raise BadArguments(f"unknown variant '{n}'; the built-in ones are "
                               + ', '.join(builtin))
        out.append((n, list(builtin[n])))
    for spec in custom or ():
        name, eq, flags = spec.partition('=')
        if not eq or not _NAME.match(name):
            raise BadArguments(f"--variant takes NAME=FLAGS, the name of letters, digits, "
                               f"'_', '.' and '-': {spec!r}")
        try:
            out.append((name, shlex.split(flags)))
        except ValueError as ex:
            raise BadArguments(f"--variant {name}: {ex}")
    seen = set()
    for n, _ in out:
        if n in seen:
            raise BadArguments(f"variant '{n}' is named twice")
        seen.add(n)
    return out


def shared_file_problems(args):
    """Why ARGS (route.py arguments every attempt gets) would have two
    attempts write one file, as messages; [] when they would not."""
    bad = []
    for i, a in enumerate(args):
        flag, eq, val = a.partition('=')
        if flag in REFUSED:
            bad.append(f"{flag} is refused: each attempt's output board is named by the driver")
        elif flag in WRITES_FILE:
            if not eq:
                val = args[i + 1] if i + 1 < len(args) else ''
            if '{dir}' not in val:
                bad.append(f"{flag} {val or '<nothing>'} would be written by every attempt; "
                           f"name it under {{dir}}, the attempt's own folder "
                           f"({flag} {{dir}}/{os.path.basename(val) or 'file'})")
    return bad


def plan_attempts(work, variants, route_args):
    """One row per variant, nothing run: its folder, the board it reads, the
    board it writes, its log, its score file and its route.py arguments
    (`{dir}` filled)."""
    rows = []
    for i, (name, flags) in enumerate(variants):
        d = os.path.join(work, f'{i:02d}_{name}')
        fill = [a.replace('{dir}', d) for a in list(route_args) + list(flags)]
        rows.append({'index': i, 'name': name, 'flags': list(flags), 'dir': d,
                     'input': os.path.join(d, 'in.kicad_pcb'),
                     'board': os.path.join(d, 'out.kicad_pcb'),
                     'log': os.path.join(d, 'route.log'),
                     'score_file': os.path.join(d, 'score.json'),
                     'args': fill})
    return rows


def writes(rows):
    """Every path the planned attempts write, one entry per write."""
    out = []
    for r in rows:
        out += [r['input'], r['board'], r['log'], r['score_file']]
        for i, a in enumerate(r['args']):
            flag, eq, val = a.partition('=')
            if flag in WRITES_FILE:
                out.append(os.path.abspath(val if eq else r['args'][i + 1]))
    return out


def _child_env():
    env = dict(os.environ)
    env.pop('REDO_MANIFEST', None)   # the driver records the winning command
    return env


def _recorded(argv):
    """ARGV as a manifest spells it: this interpreter as `python3`."""
    return ['python3'] + list(argv[1:]) if argv and argv[0] == sys.executable else list(argv)


def _knobs():
    from env_knobs import is_replay_knob
    return {k: v for k, v in sorted(os.environ.items()) if is_replay_knob(k)}


def route_argv(row, src=None, dst=None):
    return [sys.executable, '-u', '-X', 'utf8', ROUTE, src or row['input'],
            dst or row['board']] + row['args']


def grade(row, baseline):
    """board_score's score of the attempt's board, or None when it wrote
    none. check_connected's KiCad cross-check is off, as in autoroute's
    grade, so one board always scores the same."""
    env = _child_env()
    env['KICAD_NO_GRADE_RECONCILE'] = '1'
    if os.path.exists(row['score_file']):
        os.remove(row['score_file'])
    argv = [sys.executable, '-X', 'utf8', SCORE, row['board'], '--baseline', baseline,
            '--json', row['score_file'], '--label', row['name']]
    with open(row['log'], 'a', encoding='utf-8') as log:
        log.write(f"$ {' '.join(shlex.quote(a) for a in argv)}\n")
        log.flush()
        subprocess.run(argv, stdout=log, stderr=subprocess.STDOUT, env=env)
    try:
        with open(row['score_file'], encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def run_attempt(row, baseline, lock):
    """Route one variant in its own folder and score what it wrote. Fills
    ROW's `rc`, `seconds`, `wrote` and `score`."""
    t0 = time.time()
    argv = route_argv(row)
    with open(row['log'], 'w', encoding='utf-8') as log:
        log.write(f"$ {' '.join(shlex.quote(a) for a in argv)}\n")
        log.flush()
        row['rc'] = subprocess.run(argv, cwd=row['dir'], stdout=log,
                                   stderr=subprocess.STDOUT, env=_child_env()).returncode
    row['wrote'] = os.path.isfile(row['board'])
    row['score'] = grade(row, baseline) if row['wrote'] else None
    row['seconds'] = round(time.time() - t0, 1)
    with lock:
        print(f"  [{row['index']:02d}] {row['name']}: {describe(row)} "
              f"({row['seconds']:.0f} s)", flush=True)
    return row


def rank_key(row):
    """Best first: quick_score.judge_key over the attempt's score, then its
    place in the variant list."""
    from quick_score import judge_key
    return judge_key(row.get('score') or {}) + (row['index'],)


def select(rows):
    """(the winning row or None, every candidate best first). A candidate
    is an attempt that wrote a board."""
    ranked = sorted((r for r in rows if r.get('wrote')), key=rank_key)
    return (ranked[0] if ranked else None), ranked


def describe(row):
    if not row.get('wrote'):
        return f"exit {row.get('rc')}, no board"
    sc = row.get('score')
    if not sc:
        return f"exit {row.get('rc')}, board not scored"
    q = sc.get('quality') or {}
    return (f"blocking {sc.get('blocking')} joins_needed {sc.get('joins_needed')} "
            f"vias {q.get('vias')} copper_mm {q.get('copper_mm')} segments {q.get('segments')}")


def _public(row):
    sc = row.get('score') or {}
    return {'index': row['index'], 'name': row['name'], 'flags': row['flags'],
            'dir': row['dir'], 'log': row['log'],
            'cmd': _recorded(route_argv(row)), 'rc': row.get('rc'),
            'seconds': row.get('seconds'),
            'board': row['board'] if row.get('wrote') else None,
            'score': ({k: sc.get(k) for k in ('blocking', 'joins_needed', 'blocking_by',
                                              'quality', 'unknown')}
                      if row.get('score') else None),
            'score_file': row['score_file'] if row.get('score') else None}


def _ship(src, dst):
    """Copy board SRC and its siblings to DST as autoroute ships its board:
    the shipped set is exactly SRC's."""
    from autoroute import _ship as ship
    with contextlib.redirect_stdout(io.StringIO()):
        ship(src, dst)


def _drop_boards(row):
    """Remove an attempt's boards and their siblings; its log and score stay."""
    for f in os.listdir(row['dir']):
        if f.startswith(('in.', 'out.')):
            os.remove(os.path.join(row['dir'], f))


def record_winner(row, src, dst):
    """The winning command, SRC to DST, into the run's REDO_MANIFEST (no-op
    without one): a replay is that one route.py call."""
    manifest = os.environ.get('REDO_MANIFEST')
    if not manifest or manifest in (os.devnull, '/dev/null'):
        return None
    from redo_record import _resolve_manifest, write_record
    cmd = _recorded(route_argv(row, src, dst))
    try:
        manifest = _resolve_manifest(manifest)
        if manifest:
            write_record(manifest, cmd, os.getcwd(), _knobs())
    except OSError:
        return None
    return cmd


def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        usage='%(prog)s input output [options] -- <route.py arguments>',
        epilog='Exit 0 = a board shipped; 1 = no attempt wrote a board; 2 = bad arguments.')
    ap.add_argument('input_file')
    ap.add_argument('output_file')
    ap.add_argument('--variants', nargs='+', default=[], metavar='NAME',
                    help='built-in variants to run, in this order (default: every '
                         'built-in one when no --variant is given either)')
    ap.add_argument('--variant', action='append', default=[], metavar='NAME=FLAGS',
                    help="a variant of your own: route.py flags in one quoted string "
                         "(--variant deep='--max-ripup 6'); repeatable, run after --variants")
    ap.add_argument('--jobs', type=int, default=None,
                    help='attempts run at a time (default: half the cores, no more '
                         'than fit in memory at %g GB each, no more than there are '
                         'variants)' % ROUTE_GB)
    ap.add_argument('--keep-losers', action='store_true',
                    help="keep every attempt's boards (default: only the winner's; "
                         "logs and scores are always kept)")
    ap.add_argument('--list-variants', action='store_true',
                    help='print the built-in variants and exit')
    return ap


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--list-variants' in argv:
        for name, flags in BUILTIN_VARIANTS:
            print(f"{name}: {' '.join(flags) or '(no extra flags)'}")
        return 0
    route_args = []
    if '--' in argv:
        cut = argv.index('--')
        argv, route_args = argv[:cut], argv[cut + 1:]
    ap = build_parser()
    a = ap.parse_args(argv)
    src, out = os.path.abspath(a.input_file), os.path.abspath(a.output_file)
    try:
        variants = read_variants(a.variants, a.variant)
        problems = [p for _, flags in [('', route_args)] + variants
                    for p in shared_file_problems(flags)]
        if not src.endswith('.kicad_pcb') or not out.endswith('.kicad_pcb'):
            problems.append('the input and the output are .kicad_pcb paths')
        elif not os.path.isfile(src):
            problems.append(f'no such board: {a.input_file}')
        elif src == out:
            problems.append('the output is the input: route to a fresh path')
        elif not os.path.isdir(os.path.dirname(out)):
            problems.append(f'no such folder for the output: {os.path.dirname(out)}')
        if a.jobs is not None and a.jobs < 1:
            problems.append('--jobs is 1 or more')
        if problems:
            raise BadArguments('; '.join(problems))
    except BadArguments as ex:
        print(f'route_portfolio: error: {ex}', file=sys.stderr)
        return 2
    jobs = a.jobs or default_jobs(len(variants))

    from copy_board import copy_board
    from redo_record import write_record
    t_start = time.time()
    work = out + '.portfolio'
    # Stage the input before clearing the work folder: it may live there (a
    # previous run's attempt).
    stage = tempfile.mkdtemp(prefix='.portfolio_input_', dir=os.path.dirname(out))
    try:
        staged = os.path.join(stage, 'input.kicad_pcb')
        copy_board(src, staged)
        if os.path.isdir(work):
            shutil.rmtree(work)
        os.makedirs(work)
        rows = plan_attempts(work, variants, route_args)
        paths = [os.path.abspath(p) for p in writes(rows)]
        assert len(paths) == len(set(paths)), 'two attempts would write one path'
        manifest = os.path.join(work, 'redo_commands.sh')
        for r in rows:
            os.makedirs(r['dir'])
            # quietly: a board with no project has said so once, above
            with contextlib.redirect_stdout(io.StringIO()):
                copy_board(staged, r['input'])
            write_record(manifest, _recorded([sys.executable, '-X', 'utf8', COPY,
                                              src, r['input']]), r['dir'], {})
            write_record(manifest, _recorded(route_argv(r)), r['dir'], _knobs())
    finally:
        shutil.rmtree(stage, ignore_errors=True)

    print(f"route_portfolio: {len(rows)} attempt(s), {jobs} at a time, in {work}", flush=True)
    lock = threading.Lock()
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        list(pool.map(lambda r: run_attempt(r, src, lock), rows))

    winner, ranked = select(rows)
    report = {'input': src, 'output': out if winner else None, 'work': work,
              'route_args': route_args, 'jobs': jobs,
              'ranked_by': 'quick_score.judge_key (blocking, joins_needed, vias, '
                           'copper_mm, segments), then the variant order',
              'attempts': [_public(r) for r in rows],
              'ranking': [r['name'] for r in ranked],
              'winner': winner['name'] if winner else None,
              'winner_index': winner['index'] if winner else None,
              'kept_losers': bool(a.keep_losers),
              'manifest': manifest}
    if winner:
        _ship(winner['board'], out)
        report['winning_cmd'] = _recorded(route_argv(winner, src, out))
        report['recorded'] = record_winner(winner, src, out) is not None
        if not a.keep_losers:
            for r in rows:
                if r is not winner:
                    _drop_boards(r)
        print(f"route_portfolio: kept {winner['name']} ({describe(winner)}); "
              f"ranking: {', '.join(report['ranking'])}", flush=True)
    else:
        print('route_portfolio: no attempt wrote a board; nothing shipped', flush=True)
    report['seconds'] = round(time.time() - t_start, 1)
    with open(os.path.join(work, 'portfolio.json'), 'w', encoding='utf-8') as fh:
        json.dump(report, fh, indent=1, sort_keys=True)
    print('PORTFOLIO_JSON: ' + json.dumps(report, sort_keys=True), flush=True)
    return 0 if winner else 1


if __name__ == '__main__':
    sys.exit(main())

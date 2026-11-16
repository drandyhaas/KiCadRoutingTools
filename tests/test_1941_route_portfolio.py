#!/usr/bin/env python3
"""#1941: route_portfolio runs several route.py attempts side by side and
ships the one the project's score ranks best.

  1. the variants: the built-in set by default, `--variants` picks and
     orders, `--variant NAME=FLAGS` adds; an unknown name, a name given
     twice and a spec without `=` are refused for that reason; the default
     `--jobs` is bounded by cores, memory and the variant count;
  2. no two attempts write one path: every planned write is distinct, a
     shared `--json-out` must carry `{dir}`, and `--output` is refused;
  3. selection, with stand-in attempts of known scores: blocking decides
     before vias, joins_needed breaks a blocking tie, an exact tie goes to
     the earlier variant, the attempt that finishes first is not favoured,
     an attempt with no board never wins and an unscored board ranks last;
     the shipped board and project are the winner's own bytes, the losers'
     boards go unless `--keep-losers`, and the run's REDO_MANIFEST gets the
     winning command alone;
  4. no attempt writes a board: exit 1, nothing at the output path;
  5. a real two-variant run on an in-repo board: the winner is the attempt
     its own score files rank first, and the recorded winning command,
     replayed alone, gives the same copper.
"""
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

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))

import run_utils  # noqa: E402
import route_portfolio as rp  # noqa: E402
from quick_score import judge_key  # noqa: E402
from redo_record import split_env_prefix  # noqa: E402

FAILS = []
TOOL = run_utils.tool('route_portfolio.py')
#: a committed board WITH its .kicad_pro, so the shipped siblings are checkable
BOARD = os.path.join(REPO, 'kicad_files', 'routed_output.kicad_pcb')
#: a small unrouted board for the real run
REAL = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')

#: A stand-in for route.py: copies the input board (and project) to the
#: output after `--sleep S`, writes nothing with `--no-board`, and notes
#: whether a REDO_MANIFEST reached it.
STUB = r'''
import os, shutil, sys, time
src, dst, rest = sys.argv[1], sys.argv[2], sys.argv[3:]
if '--sleep' in rest:
    time.sleep(float(rest[rest.index('--sleep') + 1]))
with open(os.path.join(os.path.dirname(dst), 'manifest_seen'), 'w') as fh:
    fh.write(os.environ.get('REDO_MANIFEST', 'none'))
if '--no-board' in rest:
    sys.exit(1)
shutil.copy2(src, dst)
shutil.copy2(src[:-len('.kicad_pcb')] + '.kicad_pro', dst[:-len('.kicad_pcb')] + '.kicad_pro')
with open(dst, 'a') as fh:
    fh.write('\n; ' + ' '.join(rest) + '\n')
'''


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def score(blocking, joins, vias, copper=100.0, segments=10):
    return {'blocking': blocking, 'joins_needed': joins, 'blocking_by': {},
            'quality': {'vias': vias, 'copper_mm': copper, 'segments': segments}}


def stub_run(td, scores, extra=(), manifest=None):
    """main() with the stand-in route and SCORES ({variant: score or None})
    in place of board_score. -> (exit code, report or None, output path)."""
    out = os.path.join(td, 'out.kicad_pcb')
    argv = [BOARD, out, '--jobs', str(len(scores))]
    for name, flags in scores_flags(scores).items():
        argv += ['--variant', f'{name}={flags}']
    argv += list(extra)
    real_route, real_grade = rp.ROUTE, rp.grade
    rp.ROUTE = os.path.join(td, 'stub_route.py')
    with open(rp.ROUTE, 'w') as fh:
        fh.write(STUB)
    rp.grade = lambda row, baseline: scores[row['name']]
    env = os.environ.get('REDO_MANIFEST')
    if manifest:
        os.environ['REDO_MANIFEST'] = manifest
    else:
        os.environ.pop('REDO_MANIFEST', None)
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = rp.main(argv)
    finally:
        rp.ROUTE, rp.grade = real_route, real_grade
        os.environ.pop('REDO_MANIFEST', None)
        if env is not None:
            os.environ['REDO_MANIFEST'] = env
    m = re.search(r'^PORTFOLIO_JSON: (\{.*\})$', buf.getvalue(), re.M)
    return rc, (json.loads(m.group(1)) if m else None), out


#: per stub variant, the stand-in's flags (set by each case before stub_run)
FLAGS = {}


def scores_flags(scores):
    return {n: FLAGS.get(n, '--tag ' + n) for n in scores}


def winner_of(scores, **kw):
    td = tempfile.mkdtemp(prefix='rpf_')
    try:
        rc, rep, _ = stub_run(td, scores, **kw)
        return rc, rep
    finally:
        shutil.rmtree(td, ignore_errors=True)


def test_variants():
    print('1. the variants')
    names = [n for n, _ in rp.BUILTIN_VARIANTS]
    ok([n for n, _ in rp.read_variants([], [])] == names and names[0] == 'default'
       and rp.BUILTIN_VARIANTS[0][1] == (),
       f"no variant named: the built-in set, the plain step first: {names}")
    got = rp.read_variants(['mps_reverse', 'default'], ["deep=--max-ripup 6 --ordering 'original'"])
    ok(got == [('mps_reverse', ['--mps-reverse-rounds']), ('default', []),
               ('deep', ['--max-ripup', '6', '--ordering', 'original'])],
       f"--variants picks and orders, --variant follows: {got}")
    ok([n for n, _ in rp.read_variants([], ['a=--max-ripup 4'])] == ['a'],
       'a --variant alone runs alone')
    out = os.path.join(tempfile.gettempdir(), 'rpf_never_written.kicad_pcb')
    run_utils.check([sys.executable, TOOL, BOARD, out, '--variants', 'nope'], refuse="unknown variant 'nope'", code=2)
    run_utils.check([sys.executable, TOOL, BOARD, out, '--variants', 'default', '--variant', 'default=--max-ripup 4'],
                    refuse="variant 'default' is named twice", code=2)
    run_utils.check([sys.executable, TOOL, BOARD, out, '--variant', 'justaname'],
                    refuse='--variant takes NAME=FLAGS', code=2)
    run_utils.check([sys.executable, TOOL, BOARD, BOARD], refuse='the output is the input', code=2)
    run_utils.check([sys.executable, TOOL, BOARD, os.path.join(out + '.nodir', 'o.kicad_pcb')],
                    refuse='no such folder for the output', code=2)
    ok(not os.path.exists(out) and not os.path.exists(out + '.portfolio'),
       'a refused run writes nothing')
    ok(rp.default_jobs(4, cores=10, mem_gb=24) == 4 and rp.default_jobs(8, cores=10, mem_gb=24) == 4
       and rp.default_jobs(8, cores=10, mem_gb=64) == 5 and rp.default_jobs(8, cores=10, mem_gb=8) == 1
       and rp.default_jobs(2, cores=1, mem_gb=None) == 1,
       'default --jobs: half the cores, what memory holds, the variant count, at least 1')


def test_distinct_paths():
    print('2. no two attempts write one path')
    variants = rp.read_variants([], [])
    rows = rp.plan_attempts('/w', variants, ['--nets', '*', '--json-out', '{dir}/r.json'])
    paths = rp.writes(rows)
    ok(len(paths) == 5 * len(variants) and len(set(paths)) == len(paths),
       f"{len(paths)} planned writes over {len(variants)} attempts, all distinct")
    ok(len({r['dir'] for r in rows}) == len(rows)
       and all(p.startswith(r['dir'] + os.sep) for r in rows
               for p in (r['input'], r['board'], r['log'], r['score_file'])),
       "each attempt's board, log and score are in its own folder")
    shared = rp.writes(rp.plan_attempts('/w', variants, ['--json-out', '/w/r.json']))
    ok(len(set(shared)) < len(shared),
       'non-vacuity: a shared --json-out without {dir} IS a repeated path in the plan')
    ok(rp.shared_file_problems(['--json-out', '{dir}/r.json', '--preview-png={dir}/p.png']) == [],
       '{dir} in the value is accepted, in either spelling')
    out = os.path.join(tempfile.gettempdir(), 'rpf_never_written.kicad_pcb')
    run_utils.check([sys.executable, TOOL, BOARD, out, '--', '--json-out', 'r.json'],
                    refuse='--json-out r.json would be written by every attempt', code=2)
    run_utils.check([sys.executable, TOOL, BOARD, out, '--variant', 'a=--preview-png=p.png'],
                    refuse='--preview-png p.png would be written by every attempt', code=2)
    run_utils.check([sys.executable, TOOL, BOARD, out, '--', '--output', 'x.kicad_pcb'],
                    refuse='--output is refused', code=2)


def test_selection():
    print('3. selection over stand-in attempts')
    FLAGS.clear()
    # blocking before vias, whatever the order and whichever finishes first
    FLAGS.update({'fast': '--tag fast', 'best': '--sleep 1.5 --tag best'})
    rc, rep = winner_of({'fast': score(2, 2, 5), 'best': score(0, 0, 90)})
    ok(rc == 0 and rep['winner'] == 'best' and rep['ranking'] == ['best', 'fast'],
       f"blocking 0 with 90 vias beats blocking 2 with 5, though it is listed last and "
       f"finishes last: {rep['ranking']}")
    rc, rep = winner_of({'fast': score(0, 0, 5), 'best': score(2, 2, 90)})
    ok(rep['winner'] == 'fast', 'non-vacuity: with the scores swapped the winner swaps')
    FLAGS.clear()
    rc, rep = winner_of({'rail': score(1, 16, 3), 'pad': score(1, 1, 40)})
    ok(rep['winner'] == 'pad', 'equal blocking: fewer joins_needed wins before vias')
    rc, rep = winner_of({'a': score(0, 0, 9, 50.0, 7), 'b': score(0, 0, 9, 49.5, 9),
                         'c': score(0, 0, 8, 80.0, 9)})
    ok(rep['ranking'] == ['c', 'b', 'a'], f"then vias, copper_mm, segments: {rep['ranking']}")
    rc, rep = winner_of({'first': score(0, 0, 9), 'second': score(0, 0, 9)})
    ok(rep['winner'] == 'first' and rep['winner_index'] == 0,
       'an exact tie goes to the earlier variant')
    rc, rep = winner_of({'second': score(0, 0, 9), 'first': score(0, 0, 9)})
    ok(rep['winner'] == 'second', '... by its place in the list, not its name')
    FLAGS.update({'none': '--no-board'})
    rc, rep = winner_of({'none': score(0, 0, 0), 'unscored': None, 'poor': score(7, 9, 99)})
    ok(rc == 0 and rep['ranking'] == ['poor', 'unscored'] and rep['winner'] == 'poor',
       f"no board: not a candidate; an unscored board ranks last: {rep['ranking']}")
    by = {a['name']: a for a in rep['attempts']}
    ok(by['none']['board'] is None and by['none']['rc'] == 1 and by['unscored']['score'] is None,
       'the report says which attempt wrote no board and which was not scored')
    FLAGS.clear()

    for keep in (False, True):
        td = tempfile.mkdtemp(prefix='rpf_')
        try:
            manifest = os.path.join(td, 'redo_commands.sh')
            rc, rep, out = stub_run(td, {'a': score(3, 3, 1), 'b': score(0, 0, 4)},
                                    extra=(['--keep-losers'] if keep else [])
                                    + ['--', '--nets', '*'], manifest=manifest)
            work = out + '.portfolio'
            a, b = (os.path.join(work, d) for d in ('00_a', '01_b'))
            run_utils.evidence(os.path.join(b, 'out.kicad_pcb'))
            same = all(open(out[:-10] + ext, 'rb').read()
                       == open(os.path.join(b, 'out' + ext), 'rb').read()
                       for ext in ('.kicad_pcb', '.kicad_pro'))
            ok(rc == 0 and rep['winner'] == 'b' and same,
               "the shipped board and project are the winner's own bytes")
            ok(b'--tag b' in open(out, 'rb').read() and b'--tag a' not in open(out, 'rb').read(),
               "non-vacuity: the two attempts' boards differ, and the shipped one is b's")
            has = os.path.exists(os.path.join(a, 'out.kicad_pcb'))
            ok(has == keep and os.path.exists(os.path.join(a, 'in.kicad_pcb')) == keep,
               f"the loser's boards are {'kept with --keep-losers' if keep else 'removed'}")
            ok(os.path.isfile(os.path.join(a, 'route.log'))
               and rep['kept_losers'] == keep, "its log stays either way")
            if keep:
                continue
            ok({open(os.path.join(d, 'manifest_seen')).read() for d in (a, b)} == {'none'},
               'no attempt is handed the REDO_MANIFEST')
            lines = [l for l in open(run_utils.evidence(manifest)).read().splitlines()
                     if l and not l.startswith('#') and l != 'set -e']
            cmd = split_env_prefix(shlex.split(lines[0]))[1] if lines else []
            ok(len(lines) == 1 and cmd[-6:] == [BOARD, out, '--nets', '*', '--tag', 'b']
               and cmd == rep['winning_cmd'] and rep['recorded'] is True,
               f"the run's manifest holds the winning command alone, input to output: {cmd[-6:]}")
            own = [split_env_prefix(shlex.split(l))[1] for l in open(os.path.join(work, 'redo_commands.sh')).read().splitlines()
                   if l and not l.startswith('#') and l != 'set -e']
            ok(len(own) == 4 and [c[-2:] for c in own[1::2]] == [['--tag', 'a'], ['--tag', 'b']]
               and all('copy_board.py' in c[3] for c in own[0::2]),
               "the work folder's manifest holds every attempt: its copy, then its route")
            disk = json.load(open(os.path.join(work, 'portfolio.json')))
            ok(disk['winner'] == 'b' and [x['score']['blocking'] for x in disk['attempts']] == [3, 0]
               and all(x['cmd'] and x['rc'] == 0 for x in disk['attempts']),
               "portfolio.json carries every attempt's command, exit code and score")
        finally:
            shutil.rmtree(td, ignore_errors=True)


def test_nothing_shipped():
    print('4. no attempt writes a board')
    FLAGS.clear()
    FLAGS.update({'x': '--no-board', 'y': '--no-board'})
    td = tempfile.mkdtemp(prefix='rpf_')
    try:
        rc, rep, out = stub_run(td, {'x': score(0, 0, 0), 'y': score(0, 0, 0)},
                                manifest=os.path.join(td, 'redo_commands.sh'))
        ok(rc == 1 and rep['winner'] is None and rep['output'] is None and rep['ranking'] == [],
           f"exit 1, no winner: rc={rc}")
        ok(not os.path.exists(out) and not os.path.exists(os.path.join(td, 'redo_commands.sh')),
           'nothing at the output path, nothing recorded')
    finally:
        shutil.rmtree(td, ignore_errors=True)
        FLAGS.clear()


def copper(board):
    from board_score import quality
    return quality(board)


def test_real_run():
    print('5. a real two-variant run')
    td = tempfile.mkdtemp(prefix='rpf_')
    try:
        out = os.path.join(td, 'out.kicad_pcb')
        manifest = os.path.join(td, 'redo_commands.sh')
        env = dict(os.environ, REDO_MANIFEST=manifest)
        env.pop('REDO_RUNDIR', None)
        r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, run_utils.evidence(REAL), out,
                            '--variants', 'default', 'mps_reverse', '--jobs', '2',
                            '--keep-losers'], capture_output=True, text=True, env=env)
        m = re.search(r'^PORTFOLIO_JSON: (\{.*\})$', r.stdout, re.M)
        ok(r.returncode == 0 and m is not None, f"the run ships a board: rc={r.returncode} "
                                                f"{r.stderr.strip()[-300:]}")
        if not m:
            return
        rep = json.loads(m.group(1))
        scores = [json.load(open(run_utils.evidence(a['score_file']))) for a in rep['attempts']]
        keys = [judge_key(s) + (i,) for i, s in enumerate(scores)]
        ok(all(isinstance(s.get('blocking'), int) for s in scores),
           f"both boards were scored by board_score: blocking {[s.get('blocking') for s in scores]}")
        ok(keys[0][:-1] != keys[1][:-1],
           f"non-vacuity: the two variants score differently: {keys}")
        best = keys.index(min(keys))
        ok(rep['winner_index'] == best and rep['ranking'][0] == rep['attempts'][best]['name'],
           f"the winner is the attempt its score files rank first: {rep['winner']}")
        won = rep['attempts'][best]['board']
        ok(open(out, 'rb').read() == open(run_utils.evidence(won), 'rb').read()
           and open(out[:-10] + '.kicad_pro', 'rb').read() == open(won[:-10] + '.kicad_pro', 'rb').read(),
           "the shipped board and project are that attempt's own bytes")
        lines = [l for l in open(run_utils.evidence(manifest)).read().splitlines()
                 if l and not l.startswith('#') and l != 'set -e']
        cmd = split_env_prefix(shlex.split(lines[0]))[1] if len(lines) == 1 else []
        ok(bool(cmd) and cmd[5:7] == [REAL, out] and cmd[4].endswith('route.py'),
           'the manifest holds one route.py command, input to output')
        if not cmd:
            return
        again = os.path.join(td, 'replay', 'out.kicad_pcb')
        os.makedirs(os.path.dirname(again))
        subprocess.run([sys.executable] + cmd[1:6] + [again] + cmd[7:], capture_output=True,
                       text=True, env=dict(os.environ, REDO_MANIFEST=os.devnull))
        strip = lambda p: re.sub(rb'\(uuid "?[0-9a-f-]+"?\)', b'', open(p, 'rb').read())
        ok(copper(run_utils.evidence(again)) == copper(out) and strip(again) == strip(out),
           f"replayed alone, the winning command gives the shipped board apart from its "
           f"uuids: {copper(again)}")
    finally:
        shutil.rmtree(td, ignore_errors=True)


def main():
    for t in (test_variants, test_distinct_paths, test_selection, test_nothing_shipped,
              test_real_run):
        t()
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

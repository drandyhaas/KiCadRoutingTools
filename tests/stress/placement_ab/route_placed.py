#!/usr/bin/env python3
"""Route a PLACED board the way the stress corpus routed the original: replay
the board's recorded chain (`runs_set<SET>/<BOARD>/redo_commands.sh`) on the
placed board instead of the recorded input, then score the result.

    route_placed.py PLACED.kicad_pcb BOARD SET OUT_DIR [--quick [LEVEL]]
                    [--corpus DIR] [--repo DIR]

This is how a placement is judged (CLAUDE.md, "What a placement run is FOR"):
by routing it. A placement A/B places each arm over seeds, routes every
placed board with this, and compares `score.json` (blocking first, then
joins_needed, then vias, copper_mm, segments).

  --quick [LEVEL]  route to judge, not to ship. 1 (`--quick` alone) adds
                 route.py's --quick to the chain's whole-board route step(s)
                 (`--nets '*'`, or a bare `'*'`): the main pass without the
                 recovery tail, scored by `board_score`. 2, the seed judge,
                 adds `--quick 2` to those steps and to every route_diff.py
                 step, skipping the copper polish too, and scores with
                 `py_tools/quick_score.py` (the connectivity of the nets
                 without a zone, and board_score's quality counts), as the
                 placement skill does:
                 about half the time of level 1, and it picks seeds as well.
  --corpus DIR   the stress corpus (default $KICAD_STRESS_CORPUS, else
                 ~/Documents/kicad_stress_test).
  --repo DIR     the tree whose scripts the chain runs (default this one). The
                 manifest names the scripts by the absolute path of the tree
                 that recorded it; that prefix is remapped to DIR, and so are
                 its recorded input and run directories, to OUT_DIR/in and
                 OUT_DIR/out.

OUT_DIR is replaced: in/ (the placed board as the chain's input), out/ (every
step's output), manifest/ (the replayed manifest), chain.log, score.json. The
last line printed is the summary:
  NAME: <s> s blocking N {...} vias V copper C segments S
"""
import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
THIS_REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))   # tests/stress/placement_ab/


def baked_prefix(lines, pattern):
    """The one path prefix the manifest's commands spell for `pattern`."""
    found = {m.group(1) for line in lines if not line.startswith('#')
             for m in re.finditer(pattern, line)}
    if len(found) > 1:
        sys.exit(f'manifest names {len(found)} different prefixes for {pattern!r}: {sorted(found)}')
    return found.pop() if found else None


def whole_board_route(line):
    """A recorded route.py step over every net: `--nets '*'`, or the older
    positional spelling with a bare `'*'` (glasgow, librevna, kirdy, ...)."""
    return (not line.startswith('#') and '/route.py ' in line
            and re.search(r"(--nets '\*'|\s'\*'(\s|$))", line) is not None)


def final_board(lines):
    """The chain's last board: the last command's --output, else its second .kicad_pcb."""
    cmds = [x for x in lines if x.strip() and not x.startswith('#') and 'python' in x and '--help' not in x]
    toks = shlex.split(cmds[-1])
    if '--output' in toks:
        return toks[toks.index('--output') + 1]
    return [t for t in toks if t.endswith('.kicad_pcb')][1]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('placed')
    ap.add_argument('board', help='the corpus board name, e.g. tigard')
    ap.add_argument('set', help='its corpus set number, e.g. 3')
    ap.add_argument('out')
    ap.add_argument('--quick', nargs='?', const=1, default=0, type=int, choices=(0, 1, 2), metavar='LEVEL')
    ap.add_argument('--corpus', default=os.environ.get('KICAD_STRESS_CORPUS',
                                                       os.path.expanduser('~/Documents/kicad_stress_test')))
    ap.add_argument('--repo', default=THIS_REPO)
    a = ap.parse_args(argv)
    placed, repo, d = os.path.abspath(a.placed), os.path.abspath(a.repo), os.path.abspath(a.out)
    run_dir = os.path.join(a.corpus, f'runs_set{a.set}', a.board)
    lines = open(os.path.join(run_dir, 'redo_commands.sh')).read().split('\n')
    if a.quick:
        flag = '--quick' if a.quick == 1 else f'--quick {a.quick}'
        lines = [x + ' ' + flag if whole_board_route(x) else x for x in lines]
        if not any(whole_board_route(x) for x in lines):
            sys.exit(f'{run_dir}: no whole-board route step (route.py --nets \'*\' or \'*\') to route --quick')
        if a.quick >= 2:
            lines = [x + ' ' + flag if ('/route_diff.py ' in x and not x.startswith('#')) else x for x in lines]
    # what the recording machine called the tree, the input board's directory and the run directory
    baked_repo = baked_prefix(lines, r"(/[^\s'\"]+?)/(?:py_router|py_placer|py_tools)/\w+\.py")
    baked_in = baked_prefix(lines, rf"(/[^\s'\"]+/boards_unrouted_set{a.set})/")
    baked_run = baked_prefix(lines, rf"(/[^\s'\"]+/runs_set{a.set}/{re.escape(a.board)})(?=[/\s'\"]|$)")

    shutil.rmtree(d, ignore_errors=True)
    ind, out, man = (os.path.join(d, x) for x in ('in', 'out', 'manifest'))
    for x in (ind, out, man):
        os.makedirs(x)
    for dst in (ind, out):
        subprocess.run([sys.executable, os.path.join(repo, 'py_router', 'copy_board.py'), placed,
                        os.path.join(dst, f'{a.board}.kicad_pcb')], check=True, capture_output=True)
    open(os.path.join(man, 'redo_commands.sh'), 'w').write('\n'.join(lines))
    remaps = []
    for old, new in ((baked_in, ind), (baked_run, out), (baked_repo, repo)):
        if old and old != new:
            remaps += ['--remap', f'{old}:{new}']
    t0 = time.time()
    with open(os.path.join(d, 'chain.log'), 'w') as log:
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join(repo, 'tests', 'stress', 'redo_stress_test.py'),
                            os.path.join(man, 'redo_commands.sh'), *remaps, '--workdir', out, '--skip-checks',
                            '--skip-validate', '--continue-on-error'],
                           stdout=log, stderr=subprocess.STDOUT, cwd=repo)
    secs = time.time() - t0
    name = os.path.basename(d)
    final = os.path.join(out, os.path.basename(final_board(lines)))
    if not os.path.exists(final):
        print(f'{name}: chain rc={r.returncode} in {secs:.0f} s; NO final board {os.path.basename(final)}')
        return 1
    if a.quick >= 2:
        # the seed judge's score, as the skill takes it (py_tools/quick_score.py)
        subprocess.run([sys.executable, '-X', 'utf8', os.path.join(repo, 'py_tools', 'quick_score.py'), final,
                        '--json', os.path.join(d, 'score.json')],
                       cwd=repo, capture_output=True)
    else:
        subprocess.run([sys.executable, '-X', 'utf8', os.path.join(repo, 'py_tools', 'board_score.py'), final,
                        '--baseline', placed, '--json', os.path.join(d, 'score.json'), '--quiet'],
                       cwd=repo, capture_output=True)
    sc = json.load(open(os.path.join(d, 'score.json')))
    q = sc.get('quality') or {}
    by = {k: v for k, v in (sc.get('blocking_by') or {}).items() if v}
    print(f"{name}: {secs:.0f} s blocking {sc.get('blocking')} {by if by else ''} vias {q.get('vias')} "
          f"copper {q.get('copper_mm')} segments {q.get('segments')}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

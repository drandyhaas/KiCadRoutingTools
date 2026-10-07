#!/usr/bin/env python3
"""fa10 P1: does a refactor of check_assembly's channels leave its verdict
document byte-identical? Runs `check_assembly --json` in two worktrees on the
same boards and diffs the documents.

A REFACTOR claim ("lifted verbatim", "the census is the channel") is a claim
about every board, not the ones a unit test happens to build -- and a JSON
document that differs anywhere is a behaviour change, whatever the code looks
like. This is the instrument that says so.

    python3 tests/measure_p1_assembly_identity.py --base <base worktree> \
        [--head <head worktree>] [--extra board.kicad_pcb ...] [--json out]

Boards: the git-tracked corpus (`run_utils.corpus_boards()`), plus every
`--extra` (read in place, never copied -- StickHub is CC BY-NC-SA). Each extra
may carry `::<intent.json>` and `::<baseline.kicad_pcb>` suffixes, so the
intent waivers and the moved-vs-baseline gate are exercised too.

Exit 0 = identical on every board; 1 = some document differs (each is named,
with the first differing key); 2 = a board could not be graded in one tree
but could in the other (that is a difference too, and is never skipped).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))


def _grade(tree: str, board: str, intent: str, baseline: str,
           out: str) -> tuple:
    argv = [sys.executable, '-B', '-X', 'utf8',
            os.path.join(tree, 'py_tools', 'check_assembly.py'), board,
            '--json', out]
    if intent:
        argv += ['--intent', intent]
    if baseline:
        argv += ['--baseline', baseline]
    env = dict(os.environ, KRT_NO_BANNER='1')
    r = subprocess.run(argv, capture_output=True, text=True, cwd=tree,
                       env=env, encoding='utf-8', errors='replace')
    doc = None
    if os.path.exists(out):
        with open(out, encoding='utf-8') as fh:
            doc = json.load(fh)
    return r.returncode, doc, r.stderr[-400:]


def _first_diff(a, b, path='') -> str:
    if type(a) is not type(b):
        return f"{path or '<root>'}: {type(a).__name__} vs {type(b).__name__}"
    if isinstance(a, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a or k not in b:
                return f"{path}.{k}: present in only one"
            d = _first_diff(a[k], b[k], f"{path}.{k}")
            if d:
                return d
        return ''
    if isinstance(a, list):
        if len(a) != len(b):
            return f"{path}: length {len(a)} vs {len(b)}"
        for i, (x, y) in enumerate(zip(a, b)):
            d = _first_diff(x, y, f"{path}[{i}]")
            if d:
                return d
        return ''
    return '' if a == b else f"{path}: {a!r} vs {b!r}"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--base', required=True, help='base worktree')
    ap.add_argument('--head', default=ROOT, help='head worktree (this one)')
    ap.add_argument('--extra', nargs='*', default=[],
                    help='extra boards: path[::intent[::baseline]]')
    ap.add_argument('--no-corpus', action='store_true')
    ap.add_argument('--json', help='write the per-board table here')
    args = ap.parse_args(argv)

    jobs = []
    if not args.no_corpus:
        import run_utils
        for b in run_utils.corpus_boards():
            jobs.append((os.path.join(ROOT, b) if not os.path.isabs(b)
                         else b, '', ''))
    for spec in args.extra:
        parts = spec.split('::')
        jobs.append((parts[0], parts[1] if len(parts) > 1 else '',
                     parts[2] if len(parts) > 2 else ''))
    if not jobs:
        print('no boards to grade', file=sys.stderr)
        return 2

    rows, worst = [], 0
    with tempfile.TemporaryDirectory() as td:
        for i, (board, intent, baseline) in enumerate(jobs):
            ob = os.path.join(td, f'b{i}.json')
            oh = os.path.join(td, f'h{i}.json')
            rb, db, eb = _grade(args.base, board, intent, baseline, ob)
            rh, dh, eh = _grade(args.head, board, intent, baseline, oh)
            name = os.path.basename(board)
            if db is None or dh is None:
                same = db is None and dh is None and rb == rh
                status = 'BOTH-FAILED' if same else 'ONE-FAILED'
                worst = max(worst, 0 if same else 2)
                diff = f"base rc {rb} {eb.strip()[-120:]!r} / head rc {rh} {eh.strip()[-120:]!r}"
            else:
                diff = _first_diff(db, dh)
                if rb != rh:
                    diff = diff or f"exit {rb} vs {rh}"
                status = 'IDENTICAL' if not diff else 'DIFFERS'
                if diff:
                    worst = max(worst, 1)
            rows.append({'board': board, 'intent': intent,
                         'baseline': baseline, 'status': status,
                         'first_diff': diff})
            print(f"{status:11s} {name}{'  ' + diff if diff else ''}")
    n_id = sum(r['status'] == 'IDENTICAL' for r in rows)
    print(f"\n{n_id} of {len(rows)} board(s) identical")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(rows, fh, indent=1)
    return worst


if __name__ == '__main__':
    sys.exit(main())

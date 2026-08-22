#!/usr/bin/env python3
"""The standing gate for every placement-engine change: HEALTHY BOARDS DO NOT MOVE.

The repair and reconstruct tools exist to fix damaged placements. Their most
important property is the one that is easiest to lose: on a board that is fine,
they must propose nothing. A vector source that fires on a healthy hole pattern,
a seat search that re-seats a legal part, an intent that records the board's own
geometry as a requirement -- each is a silent regression that no per-board test
catches, because each board is individually plausible.

So the gate is the whole in-repo corpus, swept in one command, read as a table.

Two process lessons from run 7 are baked in:

  * the sweep is READ, then committed -- never chained. Run 7 committed a
    regression because the sweep ran inside an `&&` chain whose exit code
    nobody looked at, so this prints a per-board table and a verdict line, and
    exits nonzero on any movement.
  * a board that CANNOT be swept is not a pass. Parse failures, timeouts and
    crashes are their own bucket and fail the gate; a silent skip is how a
    corpus shrinks from 33 to "the ones that still work".

Usage:
    python3 -X utf8 tests/stress/corpus_noop_sweep.py                # both tools
    python3 -X utf8 tests/stress/corpus_noop_sweep.py --tool reconstruct
    python3 -X utf8 tests/stress/corpus_noop_sweep.py --boards a.kicad_pcb b...
    python3 -X utf8 tests/stress/corpus_noop_sweep.py --json report.json

Exit: 0 all quiet, 1 a healthy board moved (or could not be swept), 2 usage.
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CORPUS = os.path.join(ROOT, 'kicad_files')
TIMEOUT_S = 900
DEFAULT_BASELINE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                'corpus_noop_baseline.json')


def _signature(res):
    """What this board+tool DID, as a stable string the baseline can compare.

    Names, not counts: a regression that swaps which part moves is exactly the
    kind the counts would hide.
    """
    if res.get('refused'):
        return f'refused:{res["refused"]}'
    parts = []
    for key in ('moved', 'proposed', 'legalize_moves', 'unevidenced'):
        vals = sorted(res.get(key) or [])
        if vals:
            parts.append(f'{key}={",".join(vals)}')
    if res.get('vectors'):
        parts.append(f'vectors={res["vectors"]}')
    if res.get('unresolved'):
        parts.append(f'unresolved={res["unresolved"]}')
    return ' '.join(parts) if parts else 'quiet'


def corpus_boards():
    return sorted(glob.glob(os.path.join(CORPUS, '*.kicad_pcb')))


def _summary(out):
    for line in reversed(out.splitlines()):
        if line.startswith('JSON_SUMMARY: '):
            try:
                return json.loads(line[len('JSON_SUMMARY: '):])
            except Exception:
                return {}
    return {}


def sweep_reconstruct(board, workdir):
    """place_reconstruct --dry-run: proposals and would-move must both be empty."""
    out = os.path.join(workdir, 'out.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'),
            board, out, '--dry-run']
    proc = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=TIMEOUT_S)
    rep = _summary(proc.stdout or '')
    if proc.returncode == 3:
        # The board-state refusal (carries copper: moving footprints would
        # strand it). A refusal IS the no-op this gate is asking about, so it
        # counts as quiet -- with its reason recorded, not swallowed.
        return {'refused': 'carries copper', 'moved': [], 'proposed': [],
                'vectors': 0, 'legalize_moves': []}, None
    if proc.returncode not in (0, 4) or not rep:
        return None, f'rc={proc.returncode} ' + (proc.stderr or '')[-160:]
    moved = list(rep.get('would_move') or [])
    # A proposal that the gate rejected is still a proposal: the sweep asks
    # whether the tool SAW damage on a healthy board, not only whether it
    # applied any.
    # The key is `fit_proposals` (place_reconstruct.py:191). This read said
    # `proposals`, so it was ALWAYS [] -- the one arm of the signature that
    # exists to catch "a vector source fires on a healthy hole pattern" was
    # blind to precisely that, on every board, since the arm was written.
    # Accept the old name too: a sweep is only a change detector if it can
    # read a baseline recorded before the rename.
    props = rep.get('fit_proposals')
    if not isinstance(props, dict):
        props = rep.get('proposals')
    proposed = sorted(props.keys()) if isinstance(props, dict) else []
    vectors = (list(rep.get('vectors') or [])
               + list(rep.get('vectors_derived') or [])
               + list(rep.get('vectors_airwire') or []))
    legal = (rep.get('legalize') or {}).get('would_move') or []
    # An UNEVIDENCED move -- one the gate tuple cannot see -- is a silent
    # regression of exactly the kind this sweep exists for, and it was not in
    # the signature either.
    unevidenced = sorted({m.get('ref') for m in (rep.get('unevidenced_moves')
                                                 or []) if m.get('ref')})
    return {'moved': moved, 'proposed': proposed,
            'vectors': len(vectors), 'legalize_moves': list(legal),
            'unevidenced': unevidenced}, None


def sweep_repair(board, workdir):
    """place_seed --repair --dry-run: a healthy board has nothing to repair.

    Repair grades against an INTENT, so this arm needs one. Emitting it from
    the board under test is what a real repair run does -- which also means
    this arm moves whenever the emit changes, and that is deliberate: the
    emit deciding a healthy board's own geometry is a requirement is itself a
    regression the corpus should catch.
    """
    intent = os.path.join(workdir, 'intent.json')
    emit = subprocess.run(
        [sys.executable, '-X', 'utf8',
         os.path.join(ROOT, 'py_tools', 'check_floorplan.py'),
         board, '--emit-intent', intent],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT, timeout=TIMEOUT_S)
    if not os.path.isfile(intent):
        return ({'refused': 'no intent emittable', 'moved': [], 'repaired': 0,
                 'unresolved': 0}, None) if emit.returncode == 3 else (
            None, f'emit-intent rc={emit.returncode} '
                  + (emit.stderr or '')[-140:])
    out = os.path.join(workdir, 'rep.kicad_pcb')
    argv = [sys.executable, '-X', 'utf8',
            os.path.join(ROOT, 'py_placer', 'place_seed.py'),
            board, out, '--intent', intent, '--repair', '--dry-run']
    proc = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT, timeout=TIMEOUT_S)
    rep = _summary(proc.stdout or '')
    if proc.returncode == 3:
        return {'refused': 'board state', 'moved': [], 'repaired': 0,
                'unresolved': 0}, None
    if proc.returncode not in (0, 4) or not rep:
        return None, f'rc={proc.returncode} ' + (proc.stderr or '')[-160:]
    return {'moved': list(rep.get('moved_refs') or []),
            'repaired': rep.get('repaired', 0),
            'unresolved': rep.get('unresolved', 0)}, None


TOOLS = {'reconstruct': sweep_reconstruct, 'repair': sweep_repair}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tool', choices=sorted(TOOLS) + ['both'], default='both')
    ap.add_argument('--boards', nargs='*', default=None,
                    help='board paths (default: the whole in-repo corpus)')
    ap.add_argument('--json', default=None, help='write the full report here')
    ap.add_argument('--baseline', default=DEFAULT_BASELINE,
                    help='committed expectations to compare against '
                         '(default: %(default)s); "" to skip the comparison')
    ap.add_argument('--write-baseline', action='store_true',
                    help='record CURRENT behaviour as the expectation. Only '
                         'after reading the table and agreeing with every row.')
    args = ap.parse_args(argv)

    boards = args.boards or corpus_boards()
    if not boards:
        print('corpus_noop_sweep: no boards found', file=sys.stderr)
        return 2
    tools = sorted(TOOLS) if args.tool == 'both' else [args.tool]

    rows, movers, broken = [], [], []
    t0 = time.time()
    print(f'corpus no-op sweep: {len(boards)} board(s) x {len(tools)} tool(s)')
    for board in boards:
        name = os.path.basename(board)[:-len('.kicad_pcb')]
        with tempfile.TemporaryDirectory() as wd:
            for tool in tools:
                try:
                    res, err = TOOLS[tool](board, wd)
                except subprocess.TimeoutExpired:
                    res, err = None, f'timeout after {TIMEOUT_S}s'
                except Exception as exc:                # noqa: BLE001
                    res, err = None, f'{type(exc).__name__}: {exc}'
                row = {'board': name, 'tool': tool, 'result': res, 'error': err}
                rows.append(row)
                if err:
                    broken.append(row)
                    print(f'  ERROR  {name:32s} {tool:12s} {err}')
                    continue
                sig = _signature(res)
                row['signature'] = sig
                if res.get('refused'):
                    print(f'  quiet  {name:32s} {tool:12s} '
                          f'(refused: {res["refused"]})')
                elif sig == 'quiet':
                    print(f'  quiet  {name:32s} {tool:12s}')
                else:
                    movers.append(row)
                    print(f'  MOVED  {name:32s} {tool:12s} {sig}')

    ok = len(rows) - len(movers) - len(broken)
    print(f'\n{ok}/{len(rows)} sweeps quiet, {len(movers)} moved, '
          f'{len(broken)} could not be swept  ({time.time() - t0:.0f}s)')
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump({'rows': rows, 'quiet': ok, 'moved': len(movers),
                       'broken': len(broken)}, fh, indent=1)

    current = {f'{r["board"]}:{r["tool"]}':
               ('ERROR' if r.get('error') else r.get('signature', 'quiet'))
               for r in rows}
    if args.write_baseline:
        with open(args.baseline, 'w', encoding='utf-8') as fh:
            json.dump(current, fh, indent=1, sort_keys=True)
        print(f'wrote baseline: {args.baseline} ({len(current)} row(s))')
        return 0

    # The gate is a CHANGE detector, not a purity test. Most corpus boards are
    # healthy and must stay silent; a couple carry genuine pre-existing defects
    # (one has a real sub-clearance pad pair), and demanding silence there would
    # either hide a true finding or push someone to delete the board. So the
    # expectation is recorded per row and the failure is a DIFFERENCE.
    expected = {}
    if args.baseline and os.path.isfile(args.baseline):
        try:
            expected = json.load(open(args.baseline, encoding='utf-8'))
        except Exception as exc:                        # noqa: BLE001
            print(f'  (baseline unreadable: {exc})')
    if not expected:
        if broken:
            print('VERDICT: FAIL -- no baseline, and some boards could not be '
                  'swept. Fix those first, then --write-baseline.')
            return 1
        print(f'VERDICT: no baseline recorded. Read the table above; if every '
              f'row is what you intend, run again with --write-baseline.')
        return 0

    deltas = [(k, expected.get(k, '(new row)'), v)
              for k, v in sorted(current.items()) if expected.get(k) != v]
    # A row the baseline has and this run did not produce is only MISSING when
    # this run was supposed to produce it. Sweeping one tool, or a board
    # subset, is a legitimate narrower question -- reporting the rest as
    # missing would train people to ignore the verdict line.
    swept_tools, swept_boards = set(tools), {r['board'] for r in rows}
    gone = [k for k in sorted(expected) if k not in current
            and k.rsplit(':', 1)[-1] in swept_tools
            and k.rsplit(':', 1)[0] in swept_boards]
    skipped = len(expected) - len(current) - len(gone)
    if skipped > 0:
        print(f'  ({skipped} baseline row(s) outside this run\'s scope, not '
              f'compared)')
    for key, was, now in deltas:
        print(f'  CHANGED {key}: {was}  ->  {now}')
    for key in gone:
        print(f'  MISSING {key}: in the baseline, not swept now')
    if deltas or gone:
        print(f'VERDICT: FAIL -- {len(deltas)} row(s) changed, {len(gone)} '
              f'missing. Read each one: an intended improvement re-records the '
              f'baseline, an unintended one is the regression this gate exists '
              f'to catch.')
        return 1
    print(f'VERDICT: PASS ({len(current)} row(s) match the recorded baseline; '
          f'{ok} quiet, {len(movers)} carrying a recorded pre-existing defect)')
    return 0


if __name__ == '__main__':
    sys.path.insert(0, ROOT)
    sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
    sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
    import cli_banner
    cli_banner.install()
    sys.exit(main())

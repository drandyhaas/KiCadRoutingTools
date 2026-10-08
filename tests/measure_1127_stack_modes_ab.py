#!/usr/bin/env python3
"""#1127: the decisive stack-mode A/B, as tests/1127_stack_ab_prereg.json
pre-registers it.

Its inputs are the licence census's own output
(tests/measure_1127_licence_census.py `--json-out`), read through
`--census PATH`.

NOT RUN for #1127. This script was written against the census's FIRST run
(f90801c1), which counted the A/B harness's grader and so named orangecrab
a seed trial board. The fixed census (6e90ddc7) gives L0-undemonstrated
(L dropped) and 2 family-B trial boards per engine (rp2350, ulx3s), which is
STOP A. The script refuses a STOP A census (`decision.stop_A`), so it
records the family-B procedure as written and was never taken. STOP A comes
from the plan and CLAUDE.md's >= 3-board rule, not from the prereg text:
the prereg's `family_B.combine` would have allowed GO with no engine at 3
boards and no improving cell, which is a defect in that text, disclosed
rather than amended because the census numbers already existed.

Had it run, it would be family B, on the census's trial cells. Two consequences of
the census decision:
- With L dropped, `legality.STACK_MODE` was never built (the plan's Phase 4
  is skipped). The arms are therefore #1144's toggle:
  - off  = `legality.STACK_EXACT_CONFIRM = False` (the prereg's 'box');
  - full = `legality.STACK_EXACT_CONFIRM = True` (the prereg's 'exact').
- The prereg's family B is licence -> full. A cell with no L-hole call makes
  every decision of the L run equal to the OFF run's, so on these cells
  "licence" IS "off". The L arm is therefore not run, and its seconds are
  OFF's.

For each trial cell (board x engine, a pile cell when the census found its
C-flips on the pile):
- OFF and FULL are run through `test_placement_ab`'s own `_run_seed` / `_run`
  / `_pile_inputs`, with the flag set around the engine call only;
- the mark is `test_placement_ab._verdict` with the prereg's signal and
  guards.

GO, per engine:
- an engine with >= 3 trial boards must pass `test_placement_ab.gate()` over
  its cells (merged by board: regress dominates, then improve);
- an engine with fewer must have no regressing cell;
- every engine must pass (STACK_MODE is one setting).

Also required, from the prereg:
- FULL body_blocking <= OFF in every cell;
- summed FULL engine seconds <= 1.5 x summed OFF (= L) seconds.

StickHub is run as a separate DIAGNOSTIC block, never counted.

Not collected by run_all (no `test_` prefix).

    python3 -X utf8 tests/measure_1127_stack_modes_ab.py --census PATH
        [--workdir DIR] [--json-out PATH] [--no-stickhub]

Exit 0 = GO, 1 = NO-GO, 2 = not taken, 3 = partial.
"""
import argparse
import contextlib
import io
import json
import os
import sys
import tempfile
import time

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _d in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _d))
sys.path.insert(0, TESTS_DIR)
import test_placement_ab as AB                           # noqa: E402
from test_1094_rotated_courtyards import stickhub        # noqa: E402

PREREG = os.path.join(TESTS_DIR, '1127_stack_ab_prereg.json')
OFF = {'legality.STACK_EXACT_CONFIRM': False}
FULL = {'legality.STACK_EXACT_CONFIRM': True}
COLS = ('seconds', 'intent_errors', 'crossings', 'hpwl', 'unseated',
        'body_blocking', 'body_advisory')


def _quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def _spec(doc, engine):
    fam = doc['family_B']
    return {'signal': fam['signals'][engine], 'guard': fam['guards'][engine]}


def trial_cells(census, doc):
    """The census's family-B trial cells on committed boards, and the
    StickHub cells (diagnostic), as (key, board, engine, pile)."""
    committed = {os.path.splitext(b)[0] for b in doc['boards']}
    out, diag = [], []
    for key, r in sorted(census['cells'].items()):
        parts = key.split('|')
        board, engine, pile = parts[0], parts[1], len(parts) > 2
        hit = (r['c_flip'] > 0 or r['c_flip_any'] > 0)
        if board in committed:
            if hit:
                out.append((key, board, engine, pile))
        elif hit:
            diag.append((key, board, engine, pile))
    return out, diag


def run_cell(board_path, engine, pile, work, key):
    d = os.path.join(work, key.replace('|', '_'))
    os.makedirs(d, exist_ok=True)
    if pile:
        src, intent, _doc, refs = _quiet(AB._pile_inputs, board_path, d,
                                         require_decaps=False)
        kw = {'seed_refs': refs}
    else:
        src, intent, kw = board_path, _quiet(AB._intent_for, board_path, [],
                                             d), {}
    res = {}
    for arm, flags in (('off', OFF), ('full', FULL)):
        out = os.path.join(d, f'{arm}.kicad_pcb')
        t0 = time.time()
        if engine == 'seed':
            g = _quiet(AB._run_seed, src, out, intent, kw,
                       ignore_nets=['GND'], engine_flags=flags)
        else:
            g = _quiet(AB._run, src, out, intent,
                       dict(AB.QUENCH_BASE, ignore_nets=['GND']),
                       engine_flags=flags)
        g['seconds'] = round(time.time() - t0, 1)
        res[arm] = g
    return res


def _board_path(name):
    if name == 'StickHub':
        return stickhub()
    return os.path.join(AB.BOARDS, name + '.kicad_pcb')


def decide(results, doc):
    by_engine = {}
    for key, r in results.items():
        by_engine.setdefault(r['engine'], []).append((key, r))
    verdicts = {}
    ok = True
    for eng, cells in sorted(by_engine.items()):
        boards = sorted({r['board'] for _k, r in cells})
        marks = {k: r['mark'] for k, r in cells}
        if len(boards) >= 3:
            rows = [{'name': k, 'board': r['board']} for k, r in cells]
            passed, lines = AB.gate(rows, marks)
            rule = 'gate(): N >= 3, improve >= N-1, regress 0'
        else:
            passed = not any(m == 'regress' for m in marks.values())
            lines = [f"{len(boards)} trial board(s): held to 'no cell "
                     f"regresses'"]
            rule = 'no cell regresses (N < 3)'
        verdicts[eng] = {'boards': boards, 'marks': marks, 'pass': passed,
                         'rule': rule, 'lines': lines}
        ok = ok and passed
    bb = {k: (r['off'].get('body_blocking'), r['full'].get('body_blocking'))
          for k, r in results.items()}
    bb_ok = all((f or 0) <= (o or 0) for o, f in bb.values())
    t_off = sum(r['off']['seconds'] for r in results.values())
    t_full = sum(r['full']['seconds'] for r in results.values())
    cost_ok = t_full <= 1.5 * t_off + 1e-9
    return {'engines': verdicts, 'body_blocking_ok': bb_ok,
            'body_blocking': bb, 'seconds_off': round(t_off, 1),
            'seconds_full': round(t_full, 1), 'cost_ok': cost_ok,
            'go': bool(ok and bb_ok and cost_ok)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--census', required=True,
                    help="measure_1127_licence_census.py's --json-out")
    ap.add_argument('--workdir', default=None)
    ap.add_argument('--json-out', default=None)
    ap.add_argument('--no-stickhub', action='store_true')
    a = ap.parse_args(argv)
    with open(PREREG, encoding='utf-8') as fh:
        doc = json.load(fh)
    with open(a.census, encoding='utf-8') as fh:
        census = json.load(fh)
    dec = census.get('decision') or {}
    if dec.get('licence') != 'L0-undemonstrated':
        print(f"NOT TAKEN: the census says {dec.get('licence')}; this script "
              f"implements the L-dropped branch only")
        return 2
    if dec.get('stop_A'):
        print("NOT TAKEN: the census says STOP A")
        return 2
    cells, diag = trial_cells(census, doc)
    work = a.workdir or tempfile.mkdtemp(prefix='m1127ab_')
    results = {}
    try:
        for key, board, engine, pile in cells:
            spec = _spec(doc, engine)
            r = run_cell(_board_path(board), engine, pile, work, key)
            mark, notes = AB._verdict(r['off'], r['full'], spec)
            results[key] = {'board': board, 'engine': engine, 'pile': pile,
                            'mark': mark, 'notes': notes,
                            'off': {c: r['off'].get(c) for c in COLS},
                            'full': {c: r['full'].get(c) for c in COLS}}
            print(f"{key:<46} {mark.upper():<8} OFF {results[key]['off']} | "
                  f"FULL {results[key]['full']}", flush=True)
            for n in notes:
                print(f"    {n}", flush=True)
    except Exception as exc:                          # noqa: BLE001 - exit 2
        print(f"NOT TAKEN: {type(exc).__name__}: {exc}")
        return 2
    verdict = decide(results, doc)
    diag_rows = {}
    if not a.no_stickhub:
        for key, board, engine, pile in diag:
            path = _board_path(board)
            if not path:
                print("StickHub demo not found: diagnostic omitted")
                break
            r = run_cell(path, engine, pile, work, key)
            mark, notes = AB._verdict(r['off'], r['full'], _spec(doc, engine))
            diag_rows[key] = {'mark': mark, 'notes': notes,
                              'off': {c: r['off'].get(c) for c in COLS},
                              'full': {c: r['full'].get(c) for c in COLS}}
            print(f"DIAGNOSTIC (never counted) {key:<24} {mark.upper():<8} "
                  f"OFF {diag_rows[key]['off']} | "
                  f"FULL {diag_rows[key]['full']}", flush=True)
            for n in notes:
                print(f"    {n}", flush=True)
    print('')
    for eng, v in verdict['engines'].items():
        print(f"{eng}: {'PASS' if v['pass'] else 'FAIL'} -- {v['rule']}; "
              f"boards {v['boards']}")
        for ln in v['lines']:
            print(f"    {ln}")
    print(f"body_blocking FULL <= OFF in every cell: "
          f"{verdict['body_blocking_ok']}")
    print(f"cost: FULL {verdict['seconds_full']} s vs 1.5 x OFF "
          f"{verdict['seconds_off']} s: {verdict['cost_ok']}")
    print(f"\nFAMILY B: {'GO' if verdict['go'] else 'NO-GO'}")
    if a.json_out:
        with open(a.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'cells': results, 'verdict': verdict,
                       'diagnostic': diag_rows}, fh, indent=1, default=str)
    return 0 if verdict['go'] else 1


if __name__ == '__main__':
    sys.exit(main())

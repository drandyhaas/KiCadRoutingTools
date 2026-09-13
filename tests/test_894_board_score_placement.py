#!/usr/bin/env python3
"""`--placement-terms` reports and changes nothing that gates (#894).

The placement terms exist because a copper-free board scores the same
`blocking` and the same `quality` on every lap, so nothing can rank two
placements. They are REPORT-ONLY, and that is a claim with a sharp test: run
the real scorer twice on one board, with the flag and without, and every key
except `placement` -- and the exit code -- must be identical.

Why it is a top-level key beside `quality` rather than a `parts` member:
`blocking` is a SUM, and these terms are millimetres and counts in four
currencies. Adding a pair length to a violation count produces a number with
no unit. `tests/test_904_lens_components_cover_blocking.py` is the other half
of that argument -- it AST-parses `parts` and would demand a verifier lens for
a component no lens grades -- and it is asserted here too, from this side.

Also pinned: stdout stays ONE `SCORE_JSON=` line. `placement_score` builds a
quench state that prints, and every consumer of this script parses its whole
stdout, so the component is run as a subprocess with `--json` rather than
imported.

Run: python3 -X utf8 tests/test_894_board_score_placement.py
"""
import ast
import io
import json
import os
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 1200

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
import run_utils                                              # noqa: E402

SCORE = os.path.join(ROOT, '.claude', 'skills',
                     'plan-pcb-placement-and-routing', 'scripts',
                     'board_score.py')
PLACED = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                      'esp_prog_placed.kicad_pcb')
LAP5 = os.path.join(ROOT, 'tests', 'fixtures', 'run25',
                    'esp_prog_lap5.kicad_pcb')

passed = 0
failed = 0


def check(name, ok, detail=''):
    global passed, failed
    if ok:
        passed += 1
        print(f'  OK   {name}' + (f' -- {detail}' if detail else ''))
    else:
        failed += 1
        print(f'  FAIL {name}' + (f' -- {detail}' if detail else ''))


def run(board, *extra, out=None):
    argv = [sys.executable, '-X', 'utf8', SCORE, board, '-q']
    if out:
        argv += ['--json', out]
    argv += list(extra)
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=1200, cwd=ROOT,
                       env=run_utils.tool_env())
    doc = None
    if out and os.path.isfile(out):
        with open(out, encoding='utf-8') as fh:
            doc = json.load(fh)
    return r, doc


def test_the_flag_changes_nothing_that_gates():
    with tempfile.TemporaryDirectory(prefix='t894bs_') as tmp:
        r_off, off = run(PLACED, out=os.path.join(tmp, 'off.json'))
        r_on, on = run(PLACED, '--placement-terms',
                       out=os.path.join(tmp, 'on.json'))
    check('the exit code is unchanged',
          r_off.returncode == r_on.returncode,
          f'{r_off.returncode} vs {r_on.returncode}')
    for k in ('blocking', 'blocking_by', 'ungraded', 'unknown', 'advisory',
              'quality', 'components', 'floors', 'connectivity_nets'):
        check(f'{k} is unchanged', off.get(k) == on.get(k),
              f'off={json.dumps(off.get(k))[:70]} on={json.dumps(on.get(k))[:70]}')
    check('...and nothing else in the document moved either',
          {k: v for k, v in off.items() if k != 'placement'}
          == {k: v for k, v in on.items() if k != 'placement'},
          'every key but `placement`')
    check('the key is ABSENT without the flag, so a payload that carries it '
          'asked for it', 'placement' not in off and 'placement' in on,
          f"off={'placement' in off} on={'placement' in on}")
    for where in ('blocking_by', 'ungraded', 'unknown', 'components'):
        val = on.get(where)
        inside = ('placement' in val) if isinstance(val, (dict, list)) else False
        check(f'`placement` is not in {where}', not inside, str(where))


def test_stdout_is_still_one_score_json_line():
    """`placement_score` runs a quench that prints. If it were imported rather
    than subprocessed, its warnings would land in this stream -- which every
    consumer json.loads() whole."""
    r, _doc = run(PLACED, '--placement-terms')
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith('SCORE_JSON=')]
    check('exactly one SCORE_JSON= line', len(lines) == 1, str(len(lines)))
    try:
        json.loads(lines[0][len('SCORE_JSON='):])
        ok, why = True, 'parses'
    except Exception as exc:                                 # noqa: BLE001
        ok, why = False, str(exc)
    check('...and it parses', ok, why)
    check('the placement summary is printed for a reader too',
          'PLACEMENT (report-only, not blocking):' in r.stdout,
          [ln for ln in r.stdout.splitlines() if ln.startswith('PLACEMENT')][:1])


def test_a_copper_free_board_says_its_quality_key_is_degenerate():
    r, _doc = run(PLACED)
    check('scored without the flag, a 0-segment board says so',
          'DEGENERATE QUALITY KEY' in r.stdout,
          [ln for ln in r.stdout.splitlines()
           if 'DEGENERATE' in ln][:1])
    r_on, _d = run(PLACED, '--placement-terms')
    check('...and does not, once it has a key that can rank',
          'DEGENERATE QUALITY KEY' not in r_on.stdout, '')


def test_the_parent_delta_is_reported_against_the_named_lap():
    with tempfile.TemporaryDirectory(prefix='t894p_') as tmp:
        p = os.path.join(tmp, 'parent.json')
        _r, parent = run(PLACED, '--placement-terms', out=p)
        r, child = run(LAP5, '--placement-terms', '--parent-score', p,
                       out=os.path.join(tmp, 'child.json'))
    vs = (child.get('placement') or {}).get('vs_parent') or {}
    check('the child carries a comparison against the parent', bool(vs.get('terms')),
          str(vs.get('verdict')))
    check('...bound to the parent BOARD, not just its numbers',
          vs.get('board_sha') == parent.get('board_sha'),
          f"{str(vs.get('board_sha'))[:12]} vs {str(parent.get('board_sha'))[:12]}")
    named = {r_['term'] for r_ in vs['terms']}
    check('every term is accounted for, judged or not',
          named == set((child['placement'].get('term_order') or [])),
          str(sorted(named)))
    check('a not-comparable term is PRINTED, not dropped',
          all(('NOT JUDGED' in r.stdout) or (r_['judgement'] != 'not-comparable')
              for r_ in vs['terms']),
          [ln for ln in r.stdout.splitlines() if 'vs parent' in ln][:1])


def test_board_score_still_sums_exactly_nine_components():
    """From this side: the new key must not have joined `parts`."""
    tree = ast.parse(io.open(SCORE, encoding='utf-8').read())
    found = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'parts'
                        for t in node.targets)
                and isinstance(node.value, ast.Dict)):
            found.append([k.value for k in node.value.keys
                          if isinstance(k, ast.Constant)])
    check('exactly one `parts` literal', len(found) == 1, str(len(found)))
    check('it has nine members and `placement` is not one of them',
          len(found[0]) == 9 and 'placement' not in found[0],
          str(sorted(found[0])))


def main():
    run_utils.evidence(PLACED, 'the run-25 placed fixture')
    for name in sorted(k for k in globals() if k.startswith('test_')):
        print(f'--- {name}')
        globals()[name]()
    print(f'\n{passed} passed, {failed} failed')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())

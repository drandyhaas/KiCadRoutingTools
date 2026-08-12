#!/usr/bin/env python3
"""The loop gets a stop MECHANISM, not another stop rule.

Every stop rule in this toolchain was already written down -- a 5-lap placement
cap, four routing stop conditions, a budget of 100, "5 consecutive flat" -- and
not one of them had a mechanism. No counter, no budget check, no read of the
ledger that gated continuation; `final` and `stop_condition` were written by
converge.py and read back by nothing. So a run that stopped because the board
was finished and a run that stopped because it was stuck produced the same
artifact, and the difference lived only in whatever the report chose to say.

`converge.py verdict` is that mechanism. What this pins:

  * the four verdicts are DISTINGUISHABLE BY EXIT CODE, so a driver can branch
    on them without parsing prose;
  * reaching `blocking == 0` is the FLOOR, not the finish -- a solved board
    whose quality is still improving gets CONTINUE, because the score is
    lexicographic and quality orders the boards that already got there;
  * only ACCEPTED laps count toward a plateau. A rejected lap is data (it is
    what makes a plateau detectable) but treating it as evidence the half can
    still move would reset the counter on every rejection and the loop would
    never stop;
  * `blocking == null` is not zero. It means a component that was asked for
    could not answer, and it must never read as a perfect board;
  * UNGRADED components do not block DONE -- a corpus board has no spec files,
    so four of nine components cannot be graded and making that fatal would put
    every board permanently in STUCK -- but DONE must NAME them as unexamined.

Run: python3 -X utf8 tests/test_run9_verdict.py
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout

CONTINUE, DONE, STUCK, BUDGET, USAGE = 4, 0, 5, 6, 2

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def score(blocking, vias=10, copper=100.0, segments=50, ungraded=(), unknown=()):
    return {'blocking': blocking,
            'quality': {'vias': vias, 'copper_mm': copper,
                        'segments': segments},
            'ungraded': list(ungraded), 'unknown': list(unknown)}


def run(tmp, rows, board_score, budget=100, flat=5):
    """Write a ledger + score and ask for the verdict. Returns (code, doc)."""
    lp = os.path.join(tmp, 'ledger.jsonl')
    with open(lp, 'w', encoding='utf-8') as fh:
        for i, r in enumerate(rows):
            fh.write(json.dumps(dict(r, iteration=i)) + '\n')
    sp = os.path.join(tmp, 'score.json')
    with open(sp, 'w', encoding='utf-8') as fh:
        json.dump(board_score, fh)
    p = subprocess.run(
        [sys.executable, '-X', 'utf8', 'py_placer/converge.py', 'verdict',
         '--ledger', lp, '--score', sp,
         '--budget', str(budget), '--flat', str(flat)],
        capture_output=True, text=True, encoding='utf-8', errors='replace',
        cwd=ROOT)
    try:
        doc = json.loads(p.stdout)
    except Exception:
        doc = {'stdout': p.stdout, 'stderr': p.stderr}
    return p.returncode, doc


def lap(kind, blocking, accepted=True, **kw):
    return {'kind': kind, 'accepted': accepted,
            'score': score(blocking, **kw)}


def main():
    with tempfile.TemporaryDirectory() as tmp:
        print('a half that is still improving means CONTINUE')
        rows = ([lap('placement', 9 - i) for i in range(6)]
                + [lap('completion', 9 - i) for i in range(6)])
        code, doc = run(tmp, rows, score(3))
        check('improving on both halves -> CONTINUE', code == CONTINUE,
              f'exit {code}: {doc.get("verdict")}')

        print('blocking == 0 is the FLOOR, not the finish')
        # Routing keeps shaving vias after blocking hits 0. The loop must not
        # stop: quality orders the boards that already got there.
        rows = ([lap('placement', 0) for _ in range(6)]
                + [lap('completion', 0, vias=40 - i) for i in range(6)])
        code, doc = run(tmp, rows, score(0, vias=34))
        check('a solved board whose QUALITY still improves -> CONTINUE',
              code == CONTINUE, f'exit {code}: {doc.get("verdict")}')
        check('...and it names which half is still moving',
              doc.get('improving') == ['routing'], str(doc.get('improving')))

        print('both halves plateaued, and the board is solved')
        rows = ([lap('placement', 0) for _ in range(6)]
                + [lap('completion', 0) for _ in range(6)])
        code, doc = run(tmp, rows, score(0))
        check('blocking 0 + both halves flat -> DONE-EXHAUSTED', code == DONE,
              f'exit {code}: {doc.get("verdict")}')

        print('both halves plateaued, and the board is NOT solved')
        rows = ([lap('placement', 4) for _ in range(6)]
                + [lap('completion', 4) for _ in range(6)])
        code, doc = run(tmp, rows, score(4))
        check('blocking > 0 + both halves flat -> STUCK', code == STUCK,
              f'exit {code}: {doc.get("verdict")}')
        check('...and it refuses to call that finished',
              'calling the board finished is not' in doc.get('reason', ''),
              doc.get('reason', '')[:120])

        print('the four verdicts are distinguishable by exit code')
        check('CONTINUE, DONE, STUCK and BUDGET are four distinct codes',
              len({CONTINUE, DONE, STUCK, BUDGET}) == 4)

        print('the budget is enforced, not merely documented')
        rows = [lap('completion', 4) for _ in range(12)]
        code, doc = run(tmp, rows, score(4), budget=12)
        check('ledger rows >= budget -> BUDGET', code == BUDGET,
              f'exit {code}: {doc.get("verdict")}')
        check('...and it outranks STUCK, so the report says which ended it',
              doc.get('verdict') == 'BUDGET', str(doc.get('verdict')))

        print('only ACCEPTED laps count toward a plateau')
        # Rejected laps carry improving scores. If they counted, this half
        # would look like it is still moving and the loop would never stop.
        rows = ([lap('placement', 0) for _ in range(6)]
                + [lap('completion', 0) for _ in range(6)]
                + [lap('completion', 9 - i, accepted=False) for i in range(6)])
        code, doc = run(tmp, rows, score(0))
        check('a run of REJECTED improvements does not reset the plateau',
              code == DONE, f'exit {code}: {doc.get("verdict")}')

        print('blocking == null is not zero')
        rows = ([lap('placement', 0) for _ in range(6)]
                + [lap('completion', 0) for _ in range(6)])
        code, doc = run(tmp, rows, score(None))
        check('an ungradeable board is never DONE', code != DONE,
              f'exit {code}: {doc.get("verdict")}')
        check('...and the verdict reports blocking as null, not 0',
              doc.get('blocking') is None, str(doc.get('blocking')))

        print('UNGRADED does not block DONE, but is never silent')
        rows = ([lap('placement', 0) for _ in range(6)]
                + [lap('completion', 0) for _ in range(6)])
        code, doc = run(tmp, rows, score(
            0, ungraded=('floorplan', 'impedance', 'length', 'net_widths')))
        check('a board with no spec files can still reach DONE', code == DONE,
              f'exit {code}: {doc.get("verdict")}')
        check('...and DONE names every unexamined component',
              all(c in doc.get('reason', '') for c in
                  ('floorplan', 'impedance', 'length', 'net_widths')),
              doc.get('reason', ''))
        check('...calling them UNEXAMINED, not passed',
              'UNEXAMINED' in doc.get('reason', ''), doc.get('reason', ''))

        print('a component that RAN and could not answer is called out')
        code, doc = run(tmp, rows, score(0, unknown=('impedance',)))
        check('unknown names the instrument to fix',
              'could not answer' in doc.get('reason', ''),
              doc.get('reason', ''))

        print('a verdict about no board is refused')
        lp = os.path.join(tmp, 'ledger.jsonl')
        p = subprocess.run(
            [sys.executable, '-X', 'utf8', 'py_placer/converge.py', 'verdict',
             '--ledger', lp, '--score', os.path.join(tmp, 'nope.json')],
            capture_output=True, text=True, encoding='utf-8', cwd=ROOT)
        check('an unreadable score is exit 2, not a guess',
              p.returncode == USAGE, f'exit {p.returncode}')

        print('a fresh ledger is CONTINUE, not DONE')
        code, doc = run(tmp, [], score(0))
        check('no laps yet means the halves have not plateaued',
              code == CONTINUE, f'exit {code}: {doc.get("verdict")}')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

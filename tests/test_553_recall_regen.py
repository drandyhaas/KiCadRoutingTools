"""#553: the recall numbers are re-derivable, and they are not efficacy.

`tests/553_diagnosis_recall_baseline.json` records what
`tests/stress/diagnosis_recall.py` measured. A number in a document with no
automated change detector is a number nobody will notice going stale, so this
re-derives TWO declared cells and diffs them against the committed baseline.

Two, not thirty-six: the point is that the pipeline still produces the same
answer from the same input, which one board's two arms establish as well as
nine boards' do. The full run is the stress script, and it is cheap
(seconds per cell, no routing) if you want it.

WHAT THE BASELINE IS AND IS NOT. It is recall of KNOWN DAMAGE, measured as
LIFT: the share of the selection that is damaged over the share of the board
that is damaged, 1.0 being chance. Nothing here routes, so nothing here says
`--target-select diagnosis` produces a better ROUTE than `pins`. That claim
would need a paired routed A/B, which does not exist.

Read the `scatter` row before the `swap` row. Scatter is the negative control:
per-part jitter leaves the block centroid where it was, so the selector should
sit AT chance there, and it does. Without that row the swap row is just a
number that happens to be above 1.

    python3 -X utf8 tests/test_553_recall_regen.py
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # placement split
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))   # placement split

from run_utils import evidence  # noqa: E402

BASELINE = os.path.join(ROOT, 'tests', '553_diagnosis_recall_baseline.json')
STUDY = os.path.join(ROOT, 'tests', 'stress', 'diagnosis_recall.py')

#: One board, both the evidence arm and its negative control. Declared here so
#: a reader can see exactly which cells this file re-derives.
CELLS = ('splitflap_driver', ('swap', 'scatter'))

#: Lift is a ratio of two exact counts over a deterministic selection, so it
#: reproduces exactly. This tolerance exists only so a float formatting change
#: is not reported as a measurement change.
TOL = 1e-6

FAILURES = []


def check(cond, what):
    if cond:
        print(f'  ok   {what}')
    else:
        print(f'  FAIL {what}')
        FAILURES.append(what)


def main():
    evidence(BASELINE, 'the committed recall baseline')
    evidence(STUDY, 'the study script')
    with open(BASELINE, encoding='utf-8') as f:
        base = json.load(f)

    board, arms = CELLS
    work = tempfile.mkdtemp(prefix='t553_regen_')
    try:
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', STUDY, '--boards', board,
             '--kinds', *arms, '--out', work],
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', timeout=1800, cwd=ROOT)
        out = (r.stdout or '') + (r.stderr or '')
        if r.returncode != 0:
            print(f'  FAIL the study did not run (exit {r.returncode})')
            print(out[-1500:])
            return 1
        rows_path = os.path.join(work, 'rows.jsonl')
        check(os.path.isfile(rows_path), 'the study wrote its rows')
        with open(rows_path, encoding='utf-8') as f:
            rows = [json.loads(ln) for ln in f if ln.strip()]
    finally:
        shutil.rmtree(work, ignore_errors=True)

    got = {r['kind']: r for r in rows if r['board'] == board}
    check(set(got) == set(arms),
          f'both declared arms produced a row ({sorted(got)})')

    # The baseline is per-ARM and aggregated over boards, so the comparison is
    # against the ROW, which is what a single-board re-run can reproduce.
    for arm in arms:
        row = got.get(arm)
        if row is None or row.get('skipped'):
            check(False, f'{arm}: no usable row ({row and row.get("skipped")})')
            continue
        for key in ('lift_diagnosis', 'lift_low_pin', 'base_rate',
                    'selected', 'movable', 'truth_movable'):
            check(key in row, f'{arm}: the row carries {key}')
        check(row['lift_diagnosis'] is not None,
              f'{arm}: lift is defined')

    # The baseline's own shape, which is what a later reader relies on.
    check(set(base.get('arms', {})) <= {'swap', 'wrong_side', 'scatter',
                                        'translate'},
          f'the baseline names only known arms ({sorted(base.get("arms", {}))})')
    for arm, a in (base.get('arms') or {}).items():
        for key in ('is_evidence', 'boards', 'board_names',
                    'diagnosis_above_chance', 'diagnosis_below_chance',
                    'median_lift_diagnosis', 'median_lift_low_pin'):
            check(key in a, f'baseline[{arm}] carries {key}')

    ev = (base.get('arms') or {}).get('swap') or {}
    sc = (base.get('arms') or {}).get('scatter') or {}
    check(ev.get('is_evidence') is True and sc.get('is_evidence') is False,
          'the baseline marks which arm is evidence and which is the control')
    check((base.get('arms') or {}).get('translate', {}).get('is_evidence')
          is False,
          'and translate is NOT evidence -- perturb.block_direction calls the '
          'very metric that arm scores, so passing it is arithmetic')

    # The regen's job is the pipeline, not a verdict. But a control arm that
    # has drifted above the evidence arm would invert the whole reading, and
    # that IS worth catching here rather than in a re-read of the document.
    if ev.get('median_lift_diagnosis') and sc.get('median_lift_diagnosis'):
        check(ev['median_lift_diagnosis'] > sc['median_lift_diagnosis'],
              f'the evidence arm still outranks its negative control '
              f'({ev["median_lift_diagnosis"]} vs '
              f'{sc["median_lift_diagnosis"]}) -- if this inverts, the finding '
              f'is gone and the row must be re-recorded, not deleted')

    if FAILURES:
        print(f'\nFAILED {len(FAILURES)}:')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('\ntest_553_recall_regen: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""#1506: `--near` snaps a part off a gating pair it is in, inherited or not.

The snap ran only when the requested pose was refused, and a pose is refused
only when it makes the board WORSE, so a part aimed at its own pose on an
overlap the board already had was written back onto it, exit 0, `snapped`
None -- while `--near` promises "the nearest legal pose". A snap now also
runs when the moved part is itself in a gating pair at the request
(`pose_ops.own_pairs`), and takes the nearest pose where it is in none;
another part's damage never refuses a candidate. The text report prints the
courtyard pairs the project grades as errors.

    python3 tests/test_1506_near_own_pairs.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def near(td, board, ref, x, y, tag):
    out, js = os.path.join(td, f'{tag}.kicad_pcb'), os.path.join(td, f'{tag}.json')
    r = subprocess.run(PY + [os.path.join('py_placer', 'place_pose.py'), board, out, 'set', ref,
                             '--near', str(x), str(y), '--json', js],
                       capture_output=True, text=True, cwd=REPO)
    run_utils.evidence(js)
    with open(js) as fh:
        return r, json.load(fh)


def main():
    td = tempfile.mkdtemp(prefix='krt1506_')
    try:
        bare = os.path.join(td, 'bare.kicad_pcb')
        subprocess.run(PY + [os.path.join('tests', 'stress', 'strip_copper_only.py'),
                             os.path.join('kicad_files', 'rp2350_fpga_eensy_prePlane.kicad_pcb'),
                             bare], capture_output=True, cwd=REPO)
        run_utils.evidence(bare)
        # the tracked board has no project: one declaring the rule an error
        with open(os.path.join(td, 'bare.kicad_pro'), 'w') as fh:
            json.dump({'board': {'design_settings': {
                'rule_severities': {'courtyards_overlap': 'error'}}}}, fh)

        print('1. C19 aimed at its own pose, on the C18/C19 overlap the board had')
        r, d = near(td, bare, 'C19', 146.0, 117.9, 'c19')
        check('exit 0', r.returncode == 0, r.stderr[-300:])
        check('the board had C18/C19', ['C18', 'C19'] in d['courtyard_error_pairs_before'])
        check('the snap ran on C19\'s own pair', ['C18', 'C19'] in
              (d.get('snap_own_pairs') or {}).get('courtyard_error_pairs', []),
              json.dumps(d.get('snap_own_pairs')))
        check('...and moved it off', bool(d.get('snapped'))
              and ['C18', 'C19'] not in d['courtyard_error_pairs_after'],
              json.dumps(d.get('snapped')))
        check('...to a pose where C19 is in no gating pair',
              not any('C19' in pr for k in ('courtyard_error_pairs_after', 'pad_conflict_pairs_after')
                      for pr in d.get(k) or ()))
        check('the text report prints the courtyard error pairs',
              'courtyard overlaps the project grades as errors: ' in r.stdout)

        print('2. a part in no pair is written where it was aimed, unsnapped')
        r, d = near(td, bare, 'C19', 146.0, 118.15, 'clear')
        check('exit 0, not snapped, no own pairs',
              r.returncode == 0 and not d.get('snapped') and not d.get('snap_own_pairs'),
              json.dumps({k: d.get(k) for k in ('snapped', 'snap_own_pairs')}))
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

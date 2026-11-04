"""#1536: an own-pair `--near` snap walks every pose within `--radius`.

`set REF --near <its own pose>` snaps a part off a gating pair it is already
in (#1506). When the ranker's gate dropped every pose, only the first
`--snap-tries` (6) lattice poses were re-graded, and the part was written
back unmoved -- with a note that hid the ~190 poses never tried, while one
0.79 mm away cleared the pair. An own-pair snap now walks the whole lattice
within `--radius`, nearest first, and the fallback note says how many poses
it re-graded and what refused them.

    python3 tests/test_1536_own_pair_walk.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
POSE = os.path.join(REPO, 'py_placer', 'place_pose.py')
# a small part whose courtyard already overlaps a switch's on the input board,
# aimed at its own pose
REF, AT = 'C16', ('150.75', '116.588869')
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          f"{(': ' + str(detail)[-600:]) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def near(td, board, tag, *extra):
    out, js = os.path.join(td, f'{tag}.kicad_pcb'), os.path.join(td, f'{tag}.json')
    r = run_utils.check(PY + [POSE, board, out, 'set', REF, '--near', *AT, '--json', js,
                              *extra], accept=True)
    with open(run_utils.evidence(js)) as fh:
        return r, json.load(fh)


def main():
    with tempfile.TemporaryDirectory(prefix='krt1536_') as td:
        bare = os.path.join(td, 'bare.kicad_pcb')
        subprocess.run(PY + [os.path.join(REPO, 'tests', 'stress', 'strip_copper_only.py'),
                             os.path.join(REPO, 'kicad_files',
                                          'rp2350_fpga_eensy_prePlane.kicad_pcb'), bare],
                       capture_output=True, cwd=REPO)
        run_utils.evidence(bare, 'the fixture board')
        # the tracked board has no project: one declaring the rule an error
        with open(os.path.join(td, 'bare.kicad_pro'), 'w') as fh:
            json.dump({'board': {'design_settings': {
                'rule_severities': {'courtyards_overlap': 'error'}}}}, fh)

        print('1. at the default --snap-tries, the walk reaches the pose that clears the pair')
        _r, d = near(td, bare, 'walk')
        own = (d.get('snap_own_pairs') or {}).get('courtyard_error_pairs', [])
        pair = next((p for p in own if REF in p), None)
        ok('the part starts in a courtyard error pair of its own', pair is not None, own)
        sn = d.get('snapped') or {}
        ok('snapped, within the default 2 mm radius', bool(sn)
           and (sn.get('dist_mm') or 0) <= 2.0 + 1e-9, sn)
        ok('more candidates re-graded than --snap-tries', (sn.get('candidates_tried') or 0) > 6,
           sn.get('candidates_tried'))
        ok('the pair is gone', pair is not None and pair not in
           (d.get('courtyard_error_pairs_after') or []), d.get('courtyard_error_pairs_after'))
        ok('the census names the unbudgeted walk',
           'own-pair' in str((d.get('snap_census') or {}).get('budget')),
           (d.get('snap_census') or {}).get('budget'))

        print('2. nothing within a small radius: written as asked, and the note says why')
        r, d = near(td, bare, 'small', '--radius', '0.5')
        sc = d.get('snap_census') or {}
        ok('not snapped', not d.get('snapped'), d.get('snapped'))
        ok('every pose in reach re-graded, none untried',
           sc.get('untried') == 0 and sc.get('candidates_tried') == sc.get('lattice', 0)
           + sc.get('ranked', 0) and sc.get('candidates_tried', 0) > 6, sc)
        note = d.get('snap_note') or ''
        ok('the note counts what it re-graded and refused',
           f"re-graded {sc.get('candidates_tried')} candidate(s)" in note
           and 'refused every one' in note, note)
        ok('...and names the lever', 'a larger --radius looks further' in note, note)
        ok('...printed by the CLI', 'note: --snap/--near found no pose clearing' in r.stderr
           and 're-graded' in r.stderr, r.stderr[-400:])
    print('ALL PASS' if not failures else f'FAILED: {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

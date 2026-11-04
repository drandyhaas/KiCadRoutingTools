"""#1535: `place_pose --intent` grades the intent's `keepouts[]`.

`place_pose --intent` checked zones, covers and `mating:` keep-outs, but not
the intent's ordinary `keepouts[]`, and its snap ranker was built without
them: an exact `set` into a keep-out was written with exit 0, and a `--near`
aimed at the keep-out's rim snapped into it, while check_floorplan then
failed the board (`keepout: R20 (F) is inside keep-out ...`).

Now a moved part is graded against them as check_floorplan's `keepout` rule
grades it (`intent_keepout_pairs`, relative to the input), the ranker prices
them, and a demoted (`warn`) rule is reported, never refused.

    python3 tests/test_1535_place_pose_keepouts.py
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
FLOOR = os.path.join(REPO, 'py_tools', 'check_floorplan.py')
#: one 3 mm keep-out circle on empty board
KEEPOUT = {'name': 'mh', 'circle': [98.0, 98.0, 3.0], 'why': 'a standoff head'}
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          f"{(': ' + str(detail)[-600:]) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def floorplan_keepout_errors(board, intent):
    r = subprocess.run(PY + [FLOOR, board, '--intent', intent], capture_output=True,
                       text=True, cwd=REPO)
    return [ln for ln in r.stdout.splitlines() if '[ERROR] keepout:' in ln]


def main():
    with tempfile.TemporaryDirectory(prefix='krt1535_') as td:
        board = os.path.join(td, 'b.kicad_pcb')
        subprocess.run(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                             os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb'),
                             board], capture_output=True, cwd=REPO)
        run_utils.evidence(board, 'the fixture board')
        intent = os.path.join(td, 'intent.json')
        with open(intent, 'w') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'keepouts': [KEEPOUT]}, fh)
        warn = os.path.join(td, 'warn.json')
        with open(warn, 'w') as fh:
            json.dump({'schema': 1, 'kind': 'floorplan-intent', 'keepouts': [KEEPOUT],
                       'severity': {'keepout': 'warn'}}, fh)
        ok('the input board passes the keep-out', not floorplan_keepout_errors(board, intent))

        print('1. an exact pose into the keep-out is refused, by the keep-out')
        out = os.path.join(td, 'exact.kicad_pcb')
        run_utils.check(PY + [POSE, board, out, '--intent', intent, 'set', 'R20', '98', '98'],
                        refuse='intent_keepout_pairs new: R20/mh', code=4)
        ok('nothing written', not os.path.exists(out))

        print('2. --near on the keep-out rim snaps OUT of it, not into it')
        out = os.path.join(td, 'near.kicad_pcb')
        js = os.path.join(td, 'near.json')
        run_utils.check(PY + [POSE, board, out, '--intent', intent, 'set', 'R20',
                              '--near', '98', '101.5', '--json', js], accept=True)
        d = json.load(open(run_utils.evidence(js)))
        ok('snapped', bool(d.get('snapped')), d.get('snapped'))
        ok('no keep-out pair after', not d.get('intent_keepout_pairs_after'),
           d.get('intent_keepout_pairs_after'))
        ok('check_floorplan agrees: no keep-out error',
           not floorplan_keepout_errors(run_utils.evidence(out), intent),
           floorplan_keepout_errors(out, intent))
        kc = d.get('keepout_check') or {}
        ok('the summary names the keep-outs graded, gated',
           kc.get('keepouts') == ['mh'] and kc.get('gated') is True, kc)
        ok('legal_scope discloses the intent keep-outs',
           any('keepouts[]' in s for s in d.get('legal_scope') or ()), d.get('legal_scope'))

        print('3. a pose clear of the keep-out is written as asked')
        out = os.path.join(td, 'clear.kicad_pcb')
        js = os.path.join(td, 'clear.json')
        run_utils.check(PY + [POSE, board, out, '--intent', intent, 'set', 'R20',
                              '98', '102.75', '--json', js], accept=True)
        d = json.load(open(run_utils.evidence(js)))
        ok('not snapped, moved', not d.get('snapped') and d.get('moved'), d.get('snapped'))

        print('4. a demoted keepout rule is reported, never refused')
        out = os.path.join(td, 'warn.kicad_pcb')
        js = os.path.join(td, 'warn_out.json')
        run_utils.check(PY + [POSE, board, out, '--intent', warn, 'set', 'R20', '98', '98',
                              '--json', js], accept=True)
        d = json.load(open(run_utils.evidence(js)))
        kc = d.get('keepout_check') or {}
        ok('not gated, and R20 named inside mh',
           kc.get('gated') is False and [f['ref'] for f in kc.get('inside_after') or ()] == ['R20']
           and [f['keepout'] for f in kc.get('inside_after') or ()] == ['mh'], kc)
    print('ALL PASS' if not failures else f'FAILED: {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

"""#1540: place_simple says WHY a part is left illegal, per seed and once.

A part place_simple left illegal was reported only as "no legal pose found
for: REF" -- false when its input pose is legal, and silent on the cause --
and a run where every seed left the same part illegal ended "judge seeds
none" with judge.json's `illegal` holding seed numbers alone. Each illegal
part now carries the check it fails, with the distance past a zone or the
keep-out it is in (`illegal_why` in the seed's report and in judge.json),
and a part illegal in every seed is WARNED once with its reasons
(`illegal_every_seed`).

The case: a one-member zone drawn inside a locked part's body, so the member
can never be inside it and every seed leaves it outside.

    python3 tests/test_1540_illegal_reason.py
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
BOARD = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
# the zone lies inside the body of the part locked below
WALL, MEMBER, ZONE = 'C2', 'R20', [95.0, 89.0, 101.0, 92.0]
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}"
          f"{(': ' + str(detail)[-700:]) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def run(*args):
    return subprocess.run(PY + list(args), capture_output=True, text=True, cwd=REPO)


def main():
    with tempfile.TemporaryDirectory(prefix='krt1540_') as td:
        board = os.path.join(td, 'b.kicad_pcb')
        run(os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board)
        run_utils.evidence(board, 'the fixture board')
        locked = os.path.join(td, 'locked.kicad_pcb')
        run_utils.check(PY + [os.path.join(REPO, 'py_placer', 'place_pose.py'), board, locked,
                              'lock', WALL], accept=True)
        intent = os.path.join(td, 'intent.json')
        run(os.path.join(REPO, 'py_tools', 'check_floorplan.py'), locked,
            '--emit-intent', intent)
        doc = json.load(open(run_utils.evidence(intent, 'the emitted intent')))
        doc['blocks'] = [{'name': 'walled', 'refs': [MEMBER], 'zone': ZONE}]
        json.dump(doc, open(intent, 'w'))
        groups = os.path.join(td, 'groups.json')
        json.dump({'schema': 1, 'groups': [{'name': 'u1', 'refs': ['U1']}]}, open(groups, 'w'))

        r = run_utils.check(PY + [os.path.join(REPO, 'py_placer', 'place_simple.py'), locked,
                                  os.path.join(td, 's{seed}.kicad_pcb'), '--groups', groups,
                                  '--intent', intent, '--seeds', '0,1', '--jobs', '1',
                                  '--json', os.path.join(td, 's{seed}.json')],
                            refuse='judge seeds none', code=4)
        out = r.stdout
        why = "outside the zone of block 'walled' by"
        print('1. each seed names the part and why')
        for s in (0, 1):
            rep = json.load(open(run_utils.evidence(os.path.join(td, f's{s}.json'))))
            ok(f'seed {s}: {MEMBER} illegal, with its zone and distance',
               rep.get('illegal') == [MEMBER]
               and why in (rep.get('illegal_why') or {}).get(MEMBER, ''), rep.get('illegal_why'))
            log = open(run_utils.evidence(os.path.join(td, f's{s}.log'))).read()
            ok(f'seed {s}: its log warns with the reason, not "no legal pose found"',
               f'WARNING: {MEMBER} is left illegal: {why}' in log
               and 'no legal pose found' not in log, log[-600:])
        ok('the parent seed lines carry the reason',
           out.count(f'illegal: {MEMBER} ({why}') == 2, out[-900:])

        print('2. a part illegal in every seed is said once, with its reasons')
        ok('one WARNING names it', out.count(f'WARNING: {MEMBER} is illegal in every seed') == 1,
           out[-900:])
        judge = json.load(open(run_utils.evidence(os.path.join(td, 'judge.json'))))
        every = (judge.get('illegal_every_seed') or {}).get(MEMBER) or {}
        ok('judge.json: illegal_every_seed carries each seed\'s reason',
           sorted(every) == ['0', '1'] and all(why in (w or '') for w in every.values()),
           judge.get('illegal_every_seed'))
        ok('judge.json: illegal_why per seed, illegal still the seed list',
           judge.get('illegal') == [0, 1]
           and all(MEMBER in (judge.get('illegal_why') or {}).get(str(s), {}) for s in (0, 1)),
           {k: judge.get(k) for k in ('illegal', 'illegal_why')})
    print('ALL PASS' if not failures else f'FAILED: {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

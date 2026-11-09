#!/usr/bin/env python3
"""#1708: a plan whose step reads a board no earlier step writes is refused.

route_plan_check used to prove such a plan (exit 0, "OK"): none of its rules
asked where a step's input board comes from. Run as written, the step's tool
refuses the missing input ("input board ... does not exist", exit 2), and
without `set -e` the file can still exit 0 with a final board that step never
touched. The replayer's chain-hole detectors missed the same shape when the
step's input is its own output path, because they counted the step's own
output -- and any later step's -- as having produced it.

Checks:
  1. the premise: route.py refuses a missing input board, exit 2, for that
     reason;
  2. R18 refuses a step whose input is its own output path with nothing
     earlier writing it, and a step reading a board only a LATER step writes,
     each naming the board, and no other rule reddens;
  3. what R18 must let pass: the same plan reading the board the step before
     wrote, an in-place step on a board an earlier step wrote, a
     `route_diff a b --output b` step (the output named twice), and a second
     seed copied in before any step writes;
  4. a board is its PATH, not its basename: another folder's `s1.kicad_pcb`
     is not the one an earlier step wrote; and a `mv` takes its source away;
  5. the replayer's detectors (`compute_prune_keep`'s chain_holes,
     `compute_contamination`) name the own-output hole too;
  6. the skill's Step 9 asks for `set -e` and states the rule.

    python3 tests/test_1708_plan_board_provenance.py
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [os.path.join(ROOT, 'tests'), os.path.join(ROOT, 'tests', 'stress'),
                os.path.join(ROOT, 'py_router')]
from run_utils import check, evidence, tool  # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
BOARD = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')

VERIFY = ('# python3 -X utf8 py_router/check_drc.py {d}/final.kicad_pcb\n'
          '# python3 -X utf8 py_router/check_connected.py {d}/final.kicad_pcb\n'
          '# python3 -X utf8 py_tools/check_orphan_stubs.py {d}/final.kicad_pcb\n')

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def plan(d, *steps):
    """A plan in folder `d`: cd, copy the board in, then `steps` (each one
    line, `{d}` for the folder), then the three commented checks."""
    body = ''.join(s.format(d=d) + '\n' for s in steps)
    return (f'set -e\ncd {ROOT}\n'
            f'python3 -X utf8 py_router/copy_board.py {BOARD} {d}/s0.kicad_pcb\n'
            + body + VERIFY.format(d=d))


def write(path, text):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    return evidence(path, os.path.basename(path))


def verdict(td, name, text):
    """(rc, failed rule ids, R18 reason lines) for one plan."""
    import subprocess
    evidence(os.path.join(td, name), name)
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECKER, os.path.join(td, name)],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=120)
    assert 'Traceback' not in r.stdout + r.stderr, r.stdout + r.stderr
    lines = r.stdout.splitlines()
    failed = {ln.split()[1] for ln in lines if ln.strip().startswith('FAIL')}
    reasons, inside = [], False
    for ln in lines:
        if ln.strip().startswith(('FAIL', 'ok', 'SKIP')):
            inside = ln.strip().startswith('FAIL') and ' R18 ' in ln
            continue
        if inside and ln.startswith('          '):
            reasons.append(ln.strip())
    return r.returncode, failed, reasons


ROUTE = "python3 -X utf8 py_router/route.py {a} {b} --nets '*' --power-nets GND"


def route(a, b):
    return ROUTE.format(a=a, b=b)


def premise(td):
    print('1. the premise: a step reading a missing board is refused by its tool')
    missing = os.path.join(td, 'nowhere.kicad_pcb')
    check([sys.executable, '-X', 'utf8', tool('route.py'), missing, missing,
           '--nets', '*'], refuse='does not exist', code=2, timeout=300)
    ok('route.py refuses a missing input board, exit 2, "does not exist"', True)


def refusals(td):
    print('2. R18 refuses a board no earlier step writes')
    d = os.path.join(td, 'a')
    os.makedirs(d)
    # the reported shape: the second route step reads AND writes s1b, which
    # nothing before it wrote (the step before wrote s1)
    text = plan(d, route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
                route('{d}/s1b.kicad_pcb', '{d}/s1b.kicad_pcb'),
                route('{d}/s1b.kicad_pcb', '{d}/final.kicad_pcb'))
    write(os.path.join(d, 'plan.sh'), text)
    rc, failed, why = verdict(d, 'plan.sh', text)
    ok('its own output path, written by nothing earlier: refused by R18 alone',
       rc == 4 and failed == {'R18'}, f'rc={rc} failed={sorted(failed)}')
    ok('...naming the board and the own-output reading',
       len(why) == 1 and 's1b.kicad_pcb' in why[0]
       and 'its own output path counts only once an earlier step' in why[0], str(why))
    text = plan(d, route('{d}/s1.kicad_pcb', '{d}/s2.kicad_pcb'),
                route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
                route('{d}/s2.kicad_pcb', '{d}/final.kicad_pcb'))
    write(os.path.join(d, 'later.sh'), text)
    rc, failed, why = verdict(d, 'later.sh', text)
    ok('a board only a LATER step writes: refused by R18 alone',
       rc == 4 and failed == {'R18'} and len(why) == 1
       and 's1.kicad_pcb' in why[0] and 'own output' not in why[0],
       f'rc={rc} failed={sorted(failed)} {why}')


def accepted(td):
    print('3. what R18 must let pass')
    d = os.path.join(td, 'b')
    os.makedirs(d)
    rows = [
        ('each step reads the board the step before wrote',
         [route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
          route('{d}/s1.kicad_pcb', '{d}/final.kicad_pcb')]),
        ('an in-place step on a board an earlier step wrote',
         [route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
          route('{d}/s1.kicad_pcb', '{d}/s1.kicad_pcb'),
          route('{d}/s1.kicad_pcb', '{d}/final.kicad_pcb')]),
        ('the output named twice: route_diff a b --output b',
         ['python3 -X utf8 py_router/route_diff.py {d}/s0.kicad_pcb {d}/s1.kicad_pcb '
          '--nets /D+ /D- --output {d}/s1.kicad_pcb',
          route('{d}/s1.kicad_pcb', '{d}/final.kicad_pcb')]),
        ("the plan's input board read again by a later step",
         [route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
          route(BOARD, '{d}/final.kicad_pcb')]),
    ]
    for n, (what, steps) in enumerate(rows):
        text = plan(d, *steps)
        write(os.path.join(d, f'ok{n}.sh'), text)
        rc, failed, why = verdict(d, f'ok{n}.sh', text)
        ok(f'{what}: not refused', 'R18' not in failed and rc == 0,
           f'rc={rc} failed={sorted(failed)} {why}')


def paths(td):
    print('4. a board is its path; a mv takes its source away')
    d = os.path.join(td, 'c')
    os.makedirs(os.path.join(d, 'other'))
    text = plan(d, route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
                route('{d}/other/s1.kicad_pcb', '{d}/final.kicad_pcb'))
    write(os.path.join(d, 'other.sh'), text)
    rc, failed, why = verdict(d, 'other.sh', text)
    ok("another folder's s1.kicad_pcb is not the one written: refused",
       failed == {'R18'} and len(why) == 1 and 'other/s1.kicad_pcb' in why[0],
       f'failed={sorted(failed)} {why}')
    text = plan(d, route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
                'mv {d}/s1.kicad_pcb {d}/s2.kicad_pcb',
                'mv {d}/s1.kicad_pro {d}/s2.kicad_pro',
                route('{d}/s1.kicad_pcb', '{d}/final.kicad_pcb'))
    write(os.path.join(d, 'mv.sh'), text)
    rc, failed, why = verdict(d, 'mv.sh', text)
    ok('a board read after a mv took it away: refused',
       'R18' in failed and len(why) == 1 and 's1.kicad_pcb' in why[0],
       f'failed={sorted(failed)} {why}')


def replayer(td):
    print("5. the replayer's detectors name the own-output hole")
    import redo_stress_test as R
    d = os.path.join(td, 'd')
    os.makedirs(d)
    m = write(os.path.join(d, 'redo_commands.sh'), plan(
        d, route('{d}/s0.kicad_pcb', '{d}/s1.kicad_pcb'),
        route('{d}/s1b.kicad_pcb', '{d}/s1b.kicad_pcb'),
        route('{d}/s1b.kicad_pcb', '{d}/final.kicad_pcb')))
    cmds = R.parse_manifest(m)
    _keep, info = R.compute_prune_keep(cmds)
    ok('compute_prune_keep: chain_holes names s1b',
       info.get('chain_holes') == ['s1b.kicad_pcb'], str(info.get('chain_holes')))
    holes, tainted, _last = R.compute_contamination(cmds)
    ok('compute_contamination: s1b is a hole and final.kicad_pcb descends from it',
       holes == ['s1b.kicad_pcb'] and 'final.kicad_pcb' in tainted, f'{holes} {tainted}')
    m2 = write(os.path.join(d, 'twice.sh'), plan(
        d, 'python3 -X utf8 py_router/route_diff.py {d}/s0.kicad_pcb {d}/s1.kicad_pcb '
           '--nets /D+ /D- --output {d}/s1.kicad_pcb',
        route('{d}/s1.kicad_pcb', '{d}/final.kicad_pcb')))
    cmds = R.parse_manifest(m2)
    ok('...and not the output named twice (route_diff a b --output b)',
       R.compute_prune_keep(cmds)[1].get('chain_holes') == []
       and R.compute_contamination(cmds)[0] == [], 'reported a hole')


def skill_text():
    print('6. the skill asks for set -e and states the rule')
    text = open(SKILL, encoding='utf-8').read()
    step9 = text[text.index('## Step 9:'):]
    step9 = step9[:step9.index('### Then PROVE it')]
    ok('Step 9 asks for a `set -e` line', 'set -e' in step9, '')
    ok("Step 9 says a step reads the plan's input or an EARLIER step's output",
       "EARLIER step's" in step9 and 'R18' in step9, '')


def main():
    with tempfile.TemporaryDirectory() as td:
        premise(td)
        refusals(td)
        accepted(td)
        paths(td)
        replayer(td)
    skill_text()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

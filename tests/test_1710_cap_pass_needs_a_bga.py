#!/usr/bin/env python3
"""#1710: R12 asks for the cap pass where it has a BGA to work near.

`place_fanout_clearance.py` moves only 2-pad parts near a BGA's ball field:
its movers come from `find_components_by_type(pcb, 'BGA')`. R12 counted
`qfn_fanout.py` as a fanout that needs it, so a plan or judging chain whose
only fanouts were qfn_fanout on a board with no BGA was refused until it
carried a step that, on that board, moves nothing.

Checks:
  1. the premise: on a board with no BGA the pass reports `BGAs: (none)` and
     moves nothing;
  2. a qfn_fanout-only chain and plan, on a board with no BGA (`--board`):
     R12 holds -- and the chain is refused by nothing but R05 (by design,
     #1592: judge_seeds drops the commented checks);
  3. what R12 still refuses: a bga_fanout with no cap pass (with or without a
     board), a qfn_fanout-only plan on a board that HAS a BGA, and a cap pass
     before a later fanout;
  4. without `--board`, a qfn_fanout-only plan is NOTED (the board may have a
     BGA), not refused;
  5. the skill's Step 1c says the pass is for a board with a BGA/PGA.

    python3 tests/test_1710_cap_pass_needs_a_bga.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from run_utils import check, evidence, tool  # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing',
                       'scripts', 'route_plan_check.py')
SKILL = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'SKILL.md')
NO_BGA = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
WITH_BGA = os.path.join(ROOT, 'tests', 'fixtures', '1067', 'u30_crop.kicad_pcb')

POUR = 'python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND --plane-layers B.Cu'
QFN = "python3 -X utf8 py_router/qfn_fanout.py {in} --output {out} --component U1 --nets '*' '!GND'"
BGA = "python3 -X utf8 py_router/bga_fanout.py {in} --output {out} --component U1 --nets '*' '!GND'"
CAP = 'python3 -X utf8 py_placer/place_fanout_clearance.py {in} {out}'
ROUTE = "python3 -X utf8 py_router/route.py {in} {out} --nets '*' --power-nets GND"
VERIFY = ('# python3 -X utf8 py_router/check_drc.py final.kicad_pcb\n'
          '# python3 -X utf8 py_router/check_connected.py final.kicad_pcb\n'
          '# python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb\n')

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def as_plan(steps):
    """The steps as a <board>_plan.sh: boards by path, s0 -> s1 -> ... -> final."""
    lines = ['set -e', f'cd {ROOT}']
    for i, s in enumerate(steps):
        out = 'final.kicad_pcb' if i == len(steps) - 1 else f's{i + 1}.kicad_pcb'
        lines.append(s.replace('{in}', f's{i}.kicad_pcb').replace('{out}', out))
    return '\n'.join(lines) + '\n' + VERIFY


def verdict(td, name, text, board):
    path = os.path.join(td, name)
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    evidence(path, name)
    r = subprocess.run([sys.executable, '-X', 'utf8', CHECKER, path]
                       + (['--board', board] if board else []),
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=120)
    assert 'Traceback' not in r.stdout + r.stderr, r.stdout + r.stderr
    lines = r.stdout.splitlines()
    failed = {ln.split()[1] for ln in lines if ln.strip().startswith('FAIL')}
    at = next(i for i, ln in enumerate(lines) if ln.split()[1:2] == ['R12'])
    under = [ln.strip() for ln in lines[at + 1:at + 3] if ln.startswith('          ')]
    return r.returncode, failed, lines[at].split()[0], under


def main():
    print('1. the premise: the pass has nothing to move on a board with no BGA')
    with tempfile.TemporaryDirectory() as td:
        r = check([sys.executable, '-X', 'utf8', tool('place_fanout_clearance.py'),
                   NO_BGA, os.path.join(td, 'out.kicad_pcb')], accept=True, timeout=300)
        ok('place_fanout_clearance: BGAs (none), no caps moved',
           'BGAs: (none)' in r.stdout and 'No caps moved' in r.stdout, r.stdout[-400:])

    with tempfile.TemporaryDirectory() as td:
        print('2. a qfn_fanout-only plan on a board with no BGA')
        chain = '\n'.join((POUR, QFN, ROUTE + ' --json-out {dir}/route.json')) + '\n'
        rc, failed, st, _ = verdict(td, 'chain.txt', chain, NO_BGA)
        ok('the judging chain: R12 holds, only R05 (by design) refuses',
           st == 'ok' and failed == {'R05'}, f'rc={rc} {sorted(failed)} {st}')
        rc, failed, st, _ = verdict(td, 'plan.sh', as_plan((POUR, QFN, ROUTE)), NO_BGA)
        ok('the same as a plan: OK', rc == 0 and st == 'ok', f'rc={rc} {sorted(failed)}')

        print('3. what R12 still refuses')
        for board in (None, NO_BGA):
            rc, failed, st, why = verdict(td, 'bga.sh', as_plan((POUR, BGA, ROUTE)), board)
            ok(f'a bga_fanout with no cap pass ({"with" if board else "no"} --board)',
               rc == 4 and failed == {'R12'} and why and 'bga_fanout.py' in why[0],
               f'rc={rc} {sorted(failed)} {why}')
        rc, failed, st, why = verdict(td, 'qb.sh', as_plan((POUR, QFN, ROUTE)), WITH_BGA)
        ok('a qfn_fanout-only plan on a board that has a BGA, naming it',
           rc == 4 and failed == {'R12'} and why and 'U30' in why[0],
           f'rc={rc} {sorted(failed)} {why}')
        rc, failed, st, why = verdict(td, 'order.sh',
                                      as_plan((POUR, BGA, CAP, QFN, ROUTE)), NO_BGA)
        ok('a cap pass before a later fanout',
           failed == {'R12'} and why and 'before a later fanout' in why[0],
           f'{sorted(failed)} {why}')

        print('4. without --board: noted, not refused')
        rc, failed, st, why = verdict(td, 'nb.sh', as_plan((POUR, QFN, ROUTE)), None)
        ok('a qfn_fanout-only plan is a NOTE, exit 0',
           rc == 0 and st == 'NOTE' and why and 'pass --board' in why[0],
           f'rc={rc} {st} {why}')

    print('5. the skill says what the pass is for')
    text = open(SKILL, encoding='utf-8').read()
    head = next(ln for ln in text.splitlines() if ln.startswith('### Step 1c:'))
    ok('Step 1c heading: on a board with a BGA/PGA', 'BGA/PGA' in head, head)
    item = text[text.index('1c. **'):]
    item = item[:item.index('\n2. ')]
    ok('the 1c list item: a board with no BGA skips it', 'qfn_fanout.py' in item
       and 'BGA/PGA' in item, item[:300])
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

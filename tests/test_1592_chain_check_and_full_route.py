#!/usr/bin/env python3
"""#1592: a judging chain can be proved before it runs, and run for the full route.

plan-pcb-placement 3.7 has the agent write the board's routing chain as a
`{in}`/`{out}` file for `judge_seeds.py`, and plan-pcb-routing writes the same
commands as a `<board>_plan.sh` with boards by path and a `cd` first. Neither
tool read the other's file: `route_plan_check` refused a chain with "the plan
has no board-mutating command at all" (its boards were `.kicad_pcb` tokens
only) and asked it for a `cd`, while judge_seeds refused the plan ("no step
reads {in}"). The only full-level run of a chain was `--confirm`, which routes
level 2 first.

Checks:
  1. route_plan_check reads a file whose commands name a placeholder as a
     chain, as judge_seeds reads it: the compliant chain is OK with R01 (the
     `cd`) not asked and both steps board-mutating, and the same commands
     spelled out as a plan are OK too (the control: one set of rules);
  2. ...so R04 reports what the chain does: a chain ending on a re-pour is
     refused BY R04, naming route_planes.py -- not "no board-mutating command
     at all"; a step with no `{out}` writes no board, as judge_seeds reads
     it, so a read-only step after route.py does not end the chain;
  3. a chain judge_seeds would refuse is exit 3 for judge_seeds' reason (an
     unknown placeholder, a shell `cd` line), and judge_seeds refuses a `cd`
     line itself rather than dying on it mid-run; a plan that mentions `{in}`
     only in a comment is still a plan (R01 asked);
  4. `judge_seeds --full`: no level-2 pass (no `s<S>_route/`), the route
     steps at the full level (no `--quick` on a route.py step), the board in
     `s<S>_full.kicad_pcb`, `level: "full"` in the summary, and refused with
     `--confirm`;
  5. every route folder carries `redo_commands.sh`, a manifest of that route
     alone: the placement's copy and each step as run (with its env-knob
     prefix), which `redo_stress_test.parse_manifest` reads back -- while the
     tools still record themselves into the caller's REDO_MANIFEST (all but
     the --help step, which wrote nothing, #1640).

    python3 tests/test_1592_chain_check_and_full_route.py
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _d in ('py_router', 'py_tools', 'py_placer', os.path.join('tests', 'stress')):
    _p = os.path.join(ROOT, _d)
    if _p not in sys.path:
        sys.path.insert(0, _p)
sys.path.insert(0, HERE)

import judge_seeds                                            # noqa: E402
from copy_board import copy_board                             # noqa: E402
from redo_stress_test import parse_manifest                   # noqa: E402
from run_utils import check, evidence, tool                   # noqa: E402

CHECKER = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-routing', 'scripts',
                       'route_plan_check.py')
RUNNER = tool('judge_seeds.py')
# a committed board WITH its .kicad_pro (as test_1327 uses)
BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

VERIFY = ('# python3 -X utf8 py_router/check_drc.py final.kicad_pcb\n'
          '# python3 -X utf8 py_router/check_connected.py final.kicad_pcb\n'
          '# python3 -X utf8 py_tools/check_orphan_stubs.py final.kicad_pcb\n')
POUR = 'python3 -X utf8 py_router/route_planes.py {in} {out} --nets GND --plane-layers B.Cu\n'
ROUTE = ("python3 -X utf8 py_router/route.py {in} {out} --nets '*' --power-nets GND "
         "--json-out {dir}/route.json\n")
CHAIN = VERIFY + POUR + ROUTE

failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail and not cond else ''}")
    if not cond:
        failures.append(label)


def write(path, text):
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    return evidence(path, os.path.basename(path))


def rules(stdout, verdict):
    return {ln.split()[1] for ln in stdout.splitlines() if ln.strip().startswith(verdict)}


def checker_cases(td):
    print('1. a chain is proved, as judge_seeds reads it')
    evidence(BOARD, 'fixture board')
    r = check([sys.executable, '-X', 'utf8', CHECKER, write(os.path.join(td, 'chain.txt'), CHAIN),
               '--board', BOARD], accept=True)
    ok('the compliant chain is OK', 'OK: 16 rule(s) checked, 1 skipped' in r.stdout, r.stdout[-600:])
    ok('...read as a judging chain, both steps board-mutating',
       'a judging chain' in r.stdout and '2 command(s), 2 of them board-mutating' in r.stdout,
       r.stdout[:400])
    ok('...R01 (the cd) not asked of it, and said so',
       rules(r.stdout, 'SKIP') == {'R01'} and 'not asked of a judging chain' in r.stdout,
       r.stdout[:600])
    plan = ('#!/bin/bash\nset -e\ncd /repo\n' + VERIFY
            + POUR.replace('{in}', 's0.kicad_pcb').replace('{out}', '1.kicad_pcb')
            + ROUTE.replace('{in}', '1.kicad_pcb').replace('{out}', '2.kicad_pcb')
            .replace('{dir}', '.'))
    r = check([sys.executable, '-X', 'utf8', CHECKER, write(os.path.join(td, 'plan.sh'), plan),
               '--board', BOARD], accept=True)
    ok('control: the same commands as a plan are OK, all 17 rules run',
       'OK: 17 rule(s) checked, 0 skipped' in r.stdout and 'judging chain' not in r.stdout,
       r.stdout[-600:])

    print('2. R04 reports what the chain does')
    repour = CHAIN + POUR
    r = check([sys.executable, '-X', 'utf8', CHECKER, write(os.path.join(td, 'repour.txt'), repour)],
              refuse='the last board-mutating command is route_planes.py', code=4)
    ok('a chain ending on a re-pour is refused by R04 alone',
       rules(r.stdout, 'FAIL') == {'R04'}
       and 'no board-mutating command at all' not in r.stdout, r.stdout[-600:])
    tail = CHAIN + 'python3 -X utf8 py_router/list_nets.py {in}\n'
    check([sys.executable, '-X', 'utf8', CHECKER, write(os.path.join(td, 'tail.txt'), tail)],
          accept=True)
    ok('a step with no {out} writes no board: a read-only tail does not end the chain', True)

    print('3. what judge_seeds would refuse, and what is still a plan')
    check([sys.executable, '-X', 'utf8', CHECKER,
           write(os.path.join(td, 'typo.txt'), CHAIN.replace('{dir}', '{dri}'))],
          refuse='names {dri}', code=3)
    ok('an unknown placeholder is exit 3, for judge_seeds\' reason', True)
    check([sys.executable, '-X', 'utf8', CHECKER,
           write(os.path.join(td, 'cd.txt'), 'cd /repo\n' + CHAIN)],
          refuse="step 1 is the shell's `cd`", code=3)
    ok('a cd line in a chain is exit 3: judge_seeds would refuse it', True)
    bad = judge_seeds.chain_problems(['cd /repo', 'export KICAD_X=1',
                                      'python3 py_router/route.py {in} {out}'])
    ok('judge_seeds refuses a cd / export line rather than running it',
       any("shell's `cd`" in b for b in bad) and any("shell's `export`" in b for b in bad)
       and len(bad) == 2, str(bad))
    commented = plan.replace('# python3 -X utf8 py_router/check_drc.py final.kicad_pcb',
                             '# python3 -X utf8 py_router/check_drc.py final.kicad_pcb  # was {in}')
    r = check([sys.executable, '-X', 'utf8', CHECKER,
               write(os.path.join(td, 'plan_nocd.sh'), commented.replace('cd /repo\n', ''))],
              refuse='not a `cd <repo>`', code=4)
    ok('a plan naming {in} only in a comment is a plan: R01 asked',
       'judging chain' not in r.stdout and 'R01' in rules(r.stdout, 'FAIL'), r.stdout[-600:])


def runner_cases(td):
    print('4. judge_seeds --full: the full route with no level-2 pass')
    evidence(BOARD, 'fixture board')
    run = os.path.join(td, 'run')
    os.makedirs(run)
    copy_board(BOARD, os.path.join(run, 's0.kicad_pcb'))
    judge = write(os.path.join(run, 'judge.json'), json.dumps({'judge': [0], 'illegal': []}))
    chain = write(os.path.join(td, 'run_chain.txt'),
                  'KICAD_SMOOTH_ROUTE=1 python3 -X utf8 py_router/copy_board.py {in} {out}\n'
                  'python3 -X utf8 py_router/route.py {in} --help\n')
    check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain, '--full',
           '--confirm', '1'], refuse='--full routes every seed at the full level', code=2)
    ok('--full with --confirm is refused, nothing run',
       not os.path.exists(os.path.join(run, 's0_full_route')))
    caller = os.path.join(td, 'caller_redo.sh')
    env = dict(os.environ, REDO_MANIFEST=caller, REDO_RUNDIR=td)
    r = check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain, '--seeds', '0',
               '--full'], accept=True, env=env)
    w = os.path.join(run, 's0_full_route')
    ok('no level-2 pass: no s0_route/', not os.path.exists(os.path.join(run, 's0_route')))
    evidence(os.path.join(w, '1_copy_board.kicad_pcb'), 'the step board')
    evidence(os.path.join(run, 's0_full.kicad_pcb'), 'the full route board')
    evidence(os.path.join(run, 's0_full.score.json'), 'its score')
    log = open(evidence(os.path.join(w, 'chain.log')), encoding='utf-8').read()
    route_line = next((ln for ln in log.splitlines() if ln.startswith('$') and 'route.py' in ln), '')
    ok('the route.py step ran at the full level (no --quick)',
       route_line and '--quick' not in route_line and '--quick 2 added' not in log, route_line)
    line = next(x for x in r.stdout.splitlines() if x.startswith('JSON_SUMMARY: '))
    doc = json.loads(line[len('JSON_SUMMARY: '):])
    ok('the summary says the level, and ranks the full route',
       doc.get('level') == 'full' and doc.get('ranking') == [0] and doc.get('close') == [],
       str(doc))
    ok('...and names the routed board', 's0_full.kicad_pcb' in r.stdout, r.stdout[-400:])

    print('5. the route folder carries a manifest of that route alone')
    cmds = parse_manifest(evidence(os.path.join(w, 'redo_commands.sh'), 'route manifest'))
    tools = [next((os.path.basename(a) for a in argv if a.endswith('.py')), '?')
             for _cwd, argv in cmds]
    ok('the copy of the placement, then each step', tools == ['copy_board.py', 'copy_board.py',
                                                              'route.py'], str(tools))
    ok('...the copy reads the placement, the step writes its board',
       os.path.join(run, 's0.kicad_pcb') in cmds[0][1]
       and os.path.join(w, '1_copy_board.kicad_pcb') in cmds[1][1], str(cmds[:2]))
    ok("...the step carries its env-knob prefix, as redo_record writes it",
       cmds[1][1][0] == 'KICAD_SMOOTH_ROUTE=1' and 'KICAD_SMOOTH_ROUTE=1' not in cmds[2][1],
       str(cmds[1][1][:3]))
    rec = parse_manifest(evidence(caller, "the caller's manifest"))
    rec_tools = [next((os.path.basename(a) for a in argv if a.endswith('.py')), '?')
                 for _cwd, argv in rec]
    # each tool records itself at the top of its main; the judge's own copy is
    # not a CLI run, and route.py's --help wrote nothing, so the recorder took
    # it back (#1640)
    ok("the tools still record themselves into the caller's REDO_MANIFEST",
       rec_tools == ['copy_board.py'], str(rec_tools))
    check([sys.executable, '-X', 'utf8', RUNNER, judge, '--chain', chain], accept=True)
    cmds = parse_manifest(evidence(os.path.join(run, 's0_route', 'redo_commands.sh'),
                                   'level-2 manifest'))
    ok('the level-2 route has its own, with --quick 2 as it ran',
       len(cmds) == 3 and cmds[2][1][-2:] == ['--quick', '2'], str(cmds[-1:]))


def main():
    with tempfile.TemporaryDirectory() as td:
        checker_cases(td)
        runner_cases(td)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

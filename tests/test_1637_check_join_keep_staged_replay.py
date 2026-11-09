#!/usr/bin/env python3
"""#1637: a `check_join --keep-staged OUT` line is a COPPER step to every
reader of a manifest, not a check.

#1397 made the join record itself, but the readers still classed it as a
check by its file name, so the pruner never saw it produce a board: a chain
route -> join -> join -> copy replayed WITHOUT its route step (exit 0 on the
original run's copper seeded in, or dead at the copy with the manifest
alone), lineage_scope read the routed board as the input, and manifest_to_plan
wrote 0 steps. A pure check_join (no --keep-staged) stays a check.

  1. is_check_cmd: a pure check_join and check_drc are checks; with
     --keep-staged (spelled whole, `=`-joined or abbreviated) it is not;
  2. the chain route -> join -> join (+ a dead-end join) -> copy, recorded
     for real, replays PRUNED into a fresh folder from the manifest alone:
     the route step runs, the dead-end join is dropped, nothing is seeded,
     and the final board carries the joined copper;
  3. lineage_scope: the lineage is route + both joins + copy, from the
     unrouted input, and both joined nets are scoped;
  4. manifest_to_plan: the route step is converted and the joins are
     refused by name (the plan format has no hand-join step).

    python3 tests/test_1637_check_join_keep_staged_replay.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
STRESS = os.path.join(ROOT, 'tests', 'stress')
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, STRESS)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
J1 = ['DPA_P', '10,15,F.Cu', '14,15,F.Cu', '14,13,F.Cu', '17.52,13,F.Cu']
J2 = ['DPA_N', '10,17.54,F.Cu', '14,17.54,F.Cu', '14,17,F.Cu', '17.52,17,F.Cu']
DEAD = ['DPA_N', '10,17.54,F.Cu', '15,17.54,F.Cu', '15,17,F.Cu', '17.52,17,F.Cu']
FAILS = []


def ok(what, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {what}" + (f"  [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(what)


def main():
    from redo_stress_test import is_check_cmd
    print("1: is_check_cmd")
    cj = ['python3', 'py_tools/check_join.py', 'b.kicad_pcb', 'N', '1,1,F.Cu']
    ok("a pure check_join is a check", is_check_cmd(cj))
    ok("check_drc is a check",
       is_check_cmd(['python3', 'py_router/check_drc.py', 'b.kicad_pcb']))
    for spelled in (['--keep-staged', 'o.kicad_pcb'], ['--keep-staged=o.kicad_pcb'],
                    ['--keep-st', 'o.kicad_pcb']):
        ok(f"check_join {spelled[0]} is not a check", not is_check_cmd(cj + spelled))
    ok("route.py is not a check",
       not is_check_cmd(['python3', 'py_router/route.py', 'a.kicad_pcb', 'b.kicad_pcb']))

    evidence(BOARD, 'cap_chain board')
    tmp = tempfile.mkdtemp(prefix='t1637_')
    try:
        print("2: a chain with hand joins replays its route step")
        run = os.path.join(tmp, 'run')
        os.makedirs(run)
        inb = os.path.join(tmp, 'in.kicad_pcb')
        check([sys.executable, '-X', 'utf8', tool('copy_board.py'), BOARD, inb],
              accept=True)
        env = dict(os.environ, REDO_MANIFEST=os.path.join(run, 'redo_commands.sh'),
                   REDO_RUNDIR=run)
        py = [sys.executable, '-X', 'utf8']
        check(py + [tool('route.py'), inb, '01_route.kicad_pcb', '--nets', 'DPB_P',
                    'DPB_N'], accept=True, env=env, cwd=run)
        check(py + [tool('check_join.py'), '01_route.kicad_pcb', *J1,
                    '--keep-staged', 'j1.kicad_pcb', '-q'], accept=True, env=env, cwd=run)
        check(py + [tool('check_join.py'), 'j1.kicad_pcb', *J2,
                    '--keep-staged', 'j2.kicad_pcb', '-q'], accept=True, env=env, cwd=run)
        check(py + [tool('check_join.py'), 'j1.kicad_pcb', *DEAD,
                    '--keep-staged', 'dead.kicad_pcb', '-q'], accept=True, env=env,
              cwd=run)
        check(py + [tool('copy_board.py'), 'j2.kicad_pcb', '02_join.kicad_pcb'],
              accept=True, env=env, cwd=run)
        man = evidence(os.path.join(run, 'redo_commands.sh'), 'the manifest')
        alone = os.path.join(tmp, 'alone')
        os.makedirs(alone)
        shutil.copy(man, alone)
        man_alone = os.path.join(alone, 'redo_commands.sh')
        work = os.path.join(tmp, 'replay')
        r = check(py + [os.path.join(STRESS, 'redo_stress_test.py'), man_alone,
                        '--workdir', work, '--skip-validate'], accept=True)
        out = r.stdout
        ok("prunes to 4 of 5, the dead-end join dropped",
           'running 4 of 5' in out and 'drop [4] check_join.py -> dead.kicad_pcb' in out,
           out[-800:])
        ok("nothing is seeded from the original run", 'Seeded input' not in out)
        ok("the route step ran", os.path.isfile(os.path.join(work, '01_route.kicad_pcb')))
        final = os.path.join(work, '02_join.kicad_pcb')
        ok("the final board was made", os.path.isfile(final))
        if os.path.isfile(final):
            from kicad_parser import parse_kicad_pcb
            pcb = parse_kicad_pcb(final)
            names = {pcb.nets[s.net_id].name for s in pcb.segments
                     if s.net_id in pcb.nets}
            ok("...carrying the routed AND the joined nets",
               {'DPB_P', 'DPB_N', 'DPA_P', 'DPA_N'} <= names, str(sorted(names)))

        print("3: lineage_scope")
        lj = os.path.join(tmp, 'ls.json')
        check(py + [os.path.join(STRESS, 'lineage_scope.py'), man, '--json', lj],
              accept=True)
        rep = json.load(open(evidence(lj, 'lineage_scope json')))[0]
        ok("input board is the unrouted one",
           os.path.basename(rep['board']) == 'in.kicad_pcb', rep['board'])
        tools = [s['tool'] for s in rep['lineage']]
        ok("lineage is route + 2 joins + copy",
           tools == ['route.py', 'check_join.py', 'check_join.py', 'copy_board.py'],
           str(tools))
        kinds = [s['kind'] for s in rep['lineage'] if s['tool'] == 'check_join.py']
        ok("each join scopes its net", kinds == ['join', 'join']
           and all(s['nets'] == 1 for s in rep['lineage']
                   if s['tool'] == 'check_join.py'), str(rep['lineage']))
        ok("no joined net reads unscoped",
           not ({'DPA_P', 'DPA_N'} & {r['net'] for k in ('unscoped',
                 'unscoped_input_copper') for r in rep[k]}), str(rep))

        print("4: manifest_to_plan")
        import manifest_to_plan as m2p
        steps, _skipped = m2p.plan_steps_from_manifest(man_alone)[:2]
        ok("one plan step, the route", len(steps) == 1
           and steps[0].get('action') == 'route', str(steps)[:300])
        cmds = __import__('redo_stress_test').parse_manifest(man_alone)
        refused = [m2p.parse_command(a) for _c, a in cmds
                   if any(os.path.basename(t) == 'check_join.py' for t in a)]
        ok("the joins are refused by name",
           refused and all('_refused' in s and 'check_join.py' in s['_refused']
                           for s in refused), str(refused)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()

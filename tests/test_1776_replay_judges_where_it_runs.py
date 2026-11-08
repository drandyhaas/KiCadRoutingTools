#!/usr/bin/env python3
"""#1776: a replay judges a step's named files where it RUNS the step.

`redo_stress_test` asks each recorded command's tool whether it would refuse
(`refused_commands`, #1640). It used to ask in the command's RECORDED folder,
before `--remap` / `--workdir` were applied and before the side files were
seeded. An in-place replay re-creates a gone recorded folder EMPTY, so every
later `--workdir` / `--remap` replay found the folder, missed the step's
`--intent intent.json` in it, refused a step that runs fine where the replay
runs it, pruned the steps above it, and graded the original run's copper.

  1. in-process: judged in the recorded (empty) folder the step is refused;
     judged in --workdir / under --remap after seeding it is not, and without
     the seed it is (so the judgement really is made there);
  2. a --workdir dry-run replay keeps all 3 steps, refuses none, and prints no
     contamination banner;
  3. (control) an in-place dry-run replay, which WOULD run in the empty folder,
     refuses the step; its banner names the refused writer of the hole rather
     than blaming the agent, and names no board the pruned replay never writes
     as the one to grade.

    python3 tests/test_1776_replay_judges_where_it_runs.py
"""
import os
import shlex
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
REPLAY = os.path.join(ROOT, 'tests', 'stress', 'redo_stress_test.py')
INTENT = '{"schema": 1, "kind": "floorplan-intent"}\n'
FAILS = []


def ok(what, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {what}" + (f"  [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(what)


def write_archive(tmp):
    """An archived 3-step run whose recorded folder is back and EMPTY."""
    rec = os.path.join(tmp, 'run')
    arch = os.path.join(tmp, 'archive')
    os.makedirs(rec)
    os.makedirs(arch)
    shutil.copy2(evidence(BOARD, 'source board'), os.path.join(arch, 'in.kicad_pcb'))
    for b in ('s1', 's2', 's3'):     # the original run's boards, archived
        shutil.copy2(BOARD, os.path.join(arch, b + '.kicad_pcb'))
    with open(os.path.join(arch, 'intent.json'), 'w', encoding='utf-8') as f:
        f.write(INTENT)
    q = shlex.quote
    route, cap = q(tool('route.py')), q(tool('place_fanout_clearance.py'))
    man = os.path.join(arch, 'redo_commands.sh')
    with open(man, 'w', encoding='utf-8') as f:
        f.write(f"# cwd={q(rec)}\n"
                f"python3 {route} in.kicad_pcb s1.kicad_pcb --nets DPA_P\n"
                f"# cwd={q(rec)}\n"
                f"python3 {cap} s1.kicad_pcb s2.kicad_pcb --intent intent.json\n"
                f"# cwd={q(rec)}\n"
                f"python3 {route} s2.kicad_pcb s3.kicad_pcb --nets DPB_P\n")
    return rec, arch, man


def main():
    tmp = tempfile.mkdtemp(prefix='t1776_')
    try:
        rec, arch, man = write_archive(tmp)
        evidence(os.path.join(arch, 'intent.json'), 'archived intent')
        import redo_stress_test as R
        cmds = R.parse_manifest(man)
        ok('the manifest reads 3 commands, all in the recorded folder',
           len(cmds) == 3 and all(c == rec for c, _a in cmds), str(cmds))

        print('1: judged where the step runs')
        here = R.refused_commands(cmds, man)
        ok('(control) judged in the empty recorded folder, step 2 is refused',
           list(here) == [1] and 'cannot load intent intent.json' in here[1], str(here))
        bare = os.path.join(tmp, 'bare')
        os.makedirs(bare)
        r = R.refused_commands(cmds, man, workdir=bare)
        ok('judged in a --workdir that lacks the intent, step 2 is refused',
           list(r) == [1] and 'cannot load intent' in r[1], str(r))
        w = os.path.join(tmp, 'W')
        os.makedirs(w)
        ok('seed_side_files puts the intent in the workdir',
           R.seed_side_files(man, cmds, w) == ['intent.json'])
        r = R.refused_commands(cmds, man, workdir=w)
        ok('judged in the seeded --workdir, nothing is refused', r == {}, str(r))
        w2 = os.path.join(tmp, 'W2')
        os.makedirs(w2)
        R.seed_side_files(man, cmds, w2)
        r = R.refused_commands(cmds, man, remaps=[(rec, w2)])
        ok('judged under --remap RUN:W2 after seeding, nothing is refused', r == {}, str(r))
        ok('run_site applies the remap to the cwd and the argv',
           R.run_site(rec, ['x', os.path.join(rec, 'a.kicad_pcb')], [(rec, w2)])
           == (w2, ['x', os.path.join(w2, 'a.kicad_pcb')]))

        print('2: a --workdir replay keeps the whole chain')
        out = check([sys.executable, '-X', 'utf8', REPLAY, man,
                     '--workdir', os.path.join(tmp, 'C'), '--dry-run'], accept=True).stdout
        ok('no step is refused', 'refused [' not in out, out[-900:])
        ok('all 3 commands are kept', 'running 3 of 3 command(s)' in out, out[-900:])
        ok('no contamination banner', 'REPLAY CONTAMINATION' not in out, out[-900:])
        ok('the side file is seeded before the judgement',
           0 <= out.find('Seeded side-file(s)') < out.find('Pruning'), out[-900:])

        print('3: (control) an in-place replay runs, and is judged, in the empty folder')
        out = check([sys.executable, '-X', 'utf8', REPLAY, man, '--dry-run'],
                    accept=True, allow=('No such file or directory', 'Errno 2')).stdout
        ok('step 2 is refused there', 'refused [2]: ' in out and 'cannot load intent' in out,
           out[-1200:])
        ok('the banner names the refused writer of the hole',
           's2.kicad_pcb  (its writer [2] is refused here' in out, out[-1200:])
        ok('...and does not blame the agent', "cp/mv'd" not in out, out[-1200:])
        ok('it never names s1.kicad_pcb, which this pruned replay does not write, '
           'as the board to grade',
           'grade THIS' not in out
           and 'No board this replay writes has fully current-code ancestry' in out,
           out[-1200:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()

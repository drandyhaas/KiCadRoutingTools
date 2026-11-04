#!/usr/bin/env python3
"""#1640: a command its tool REFUSED wrote nothing, so it is not a replay step.

Every self-recording tool records itself before it does its work (so a killed
run is still in the manifest), and the refusals are everywhere: argparse, a
`parser.error` after the parse, a `print` + `sys.exit(2)`, a traceback on a
missing input. `redo_record` takes the record back as the process ends when
the command wrote nothing. A refused route as the last command used to become
the pruned chain's final board, prune the real work away and kill the replay.

  1. refusals of three shapes leave no record and no timings line: route.py's
     post-parse `parser.error`, copy_board's `sys.exit(main())` (the shape
     whose status no in-process hook can learn), qfn_fanout's traceback;
  2. a command that wrote its board IS recorded, and so is one interrupted
     by Ctrl-C (the killed run the early record exists for);
  3. `retract_record` takes back only its own record, also after another
     record was appended behind it;
  4. an OLDER manifest that still carries a refused command: the pruner skips
     what the tool's own parser refuses (`refused_commands`), so the real
     last board stays final, and a dry-run replay says so.

    python3 tests/test_1640_refused_command_not_recorded.py
"""
import os
import shlex
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'stress'))
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
FAILS = []


def ok(what, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {what}" + (f"  [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(what)


def records(man):
    if not os.path.exists(man):
        return []
    return [l for l in open(man, encoding='utf-8') if l.startswith('python3 ')]


def run_dir(tmp, name):
    d = os.path.join(tmp, name)
    os.makedirs(d)
    man = os.path.join(d, 'redo_commands.sh')
    return d, man, dict(os.environ, REDO_MANIFEST=man, REDO_RUNDIR=d)


def main():
    evidence(BOARD, 'cap_chain board')
    tmp = tempfile.mkdtemp(prefix='t1640_')
    try:
        print("1: a refused command leaves no record")
        d, man, env = run_dir(tmp, 'route')
        check([sys.executable, '-X', 'utf8', tool('route.py'), 'nope.kicad_pcb',
               'x.kicad_pcb', '--nets', 'GND'], refuse='does not exist',
               code=2, env=env, cwd=d)
        ok("route.py: no record", records(man) == [], str(records(man)))
        ok("route.py: no timings line",
           not os.path.exists(man[:-3] + '.timings.jsonl'))
        d, man, env = run_dir(tmp, 'copy')
        check([sys.executable, '-X', 'utf8', tool('copy_board.py'),
               'nope.kicad_pcb', 'o.kicad_pcb'], refuse='copy_board: error',
              code=1, env=env, cwd=d)
        ok("copy_board (sys.exit(main())): no record", records(man) == [],
           str(records(man)))
        # A traceback is run_utils.check's "accident" by design, so this one
        # is run by hand -- and its REASON asserted: the missing board.
        d, man, env = run_dir(tmp, 'qfn')
        r = subprocess.run([sys.executable, '-X', 'utf8', tool('qfn_fanout.py'),
                            'nope.kicad_pcb', '--component', 'U1', '--output',
                            'o.kicad_pcb'], cwd=d, env=env, capture_output=True,
                           text=True)
        ok("qfn_fanout died on the missing board",
           r.returncode == 1 and "FileNotFoundError" in r.stderr
           and 'nope.kicad_pcb' in r.stderr, r.stderr[-300:])
        ok("qfn_fanout (traceback): no record", records(man) == [],
           str(records(man)))

        print("2: a command that wrote, or was interrupted, stays recorded")
        d, man, env = run_dir(tmp, 'good')
        shutil.copy(BOARD, os.path.join(d, 'in.kicad_pcb'))
        check([sys.executable, '-X', 'utf8', tool('copy_board.py'),
               'in.kicad_pcb', 'out.kicad_pcb'], accept=True, env=env, cwd=d)
        evidence(os.path.join(d, 'out.kicad_pcb'), 'the copied board')
        ok("a copy that wrote its board is recorded", len(records(man)) == 1,
           str(records(man)))
        ok("...with its timings line",
           os.path.isfile(man[:-3] + '.timings.jsonl'))
        d, man, env = run_dir(tmp, 'killed')
        r = subprocess.run(
            [sys.executable, '-c',
             'import sys; sys.path.insert(0, "py_router"); '
             'sys.argv = ["py_router/route.py", "a.kicad_pcb", "b.kicad_pcb"]; '
             'import redo_record; redo_record.record_invocation(); '
             'raise KeyboardInterrupt'],
            cwd=ROOT, env=env, capture_output=True, text=True)
        ok("the probe was interrupted", 'KeyboardInterrupt' in r.stderr,
           r.stderr[-300:])
        ok("an interrupted command keeps its record", len(records(man)) == 1,
           str(records(man)))
        d, man, env = run_dir(tmp, 'unnamed')
        side = os.path.join(tmp, 'elsewhere')
        os.makedirs(side)
        side_file = os.path.join(side, 'x.kicad_pcb')
        check([sys.executable, '-c',
               'import sys; sys.path.insert(0, "py_router"); '
               'sys.argv = ["py_router/route.py", "a.kicad_pcb"]; '
               'import redo_record; redo_record.record_invocation(); '
               f'open({side_file!r}, "w").write("x")'], accept=True, env=env)
        evidence(side_file, 'the unnamed output')
        ok("a write to a path its command line does not name keeps the record",
           len(records(man)) == 1, str(records(man)))

        print("3: retract_record takes back only its own record")
        import redo_record
        man = os.path.join(tmp, 'm.sh')
        a = redo_record.write_record(man, ['python3', 'a.py'], tmp, {})
        b = redo_record.write_record(man, ['python3', 'b.py'], tmp, {})
        redo_record.write_record(man, ['python3', 'c.py'], tmp, {})
        ok("found behind a later record", redo_record.retract_record(man, b))
        got = [l.split()[-1] for l in records(man)]
        ok("a and c stay", got == ['a.py', 'c.py'], str(got))
        ok("the header stays", open(man).read().startswith('#!/bin/bash'))
        man2 = os.path.join(tmp, 'm2.sh')
        only = redo_record.write_record(man2, ['python3', 'a.py'], tmp, {})
        redo_record.retract_record(man2, only)
        ok("a manifest its one record created is removed",
           not os.path.exists(man2))
        ok("a record already gone is not found",
           not redo_record.retract_record(man, b))
        ok("...and the file is unchanged",
           [l.split()[-1] for l in records(man)] == ['a.py', 'c.py'])
        del a

        print("4: an older manifest still carrying a refused command")
        d = os.path.join(tmp, 'old')
        os.makedirs(d)
        route = tool('route.py')
        old = os.path.join(d, 'redo_commands.sh')
        with open(old, 'w', encoding='utf-8') as f:
            f.write(f"# cwd={shlex.quote(d)}\n"
                    f"python3 {route} {BOARD} good.kicad_pcb --nets DPB_P\n"
                    f"# cwd={shlex.quote(d)}\n"
                    f"python3 {route} good.kicad_pcb x.kicad_pcb --nets GND "
                    f"--no-such-flag\n")
        from redo_stress_test import (compute_prune_keep, parse_manifest,
                                      refused_commands)
        cmds = parse_manifest(old)
        refused = refused_commands(cmds, old)
        ok("the tool's parser refuses command 2", list(refused) == [1],
           str(refused))
        ok("...naming the flag", '--no-such-flag' in (refused.get(1) or ''),
           str(refused))
        keep, info = compute_prune_keep(cmds, refused=refused)
        ok("the real last board stays final",
           info['final_board'] == 'good.kicad_pcb' and keep == {0},
           f"{info['final_board']} {keep}")
        _k, plain = compute_prune_keep(cmds)
        ok("(control) unaided, the refused command takes over",
           plain['final_board'] == 'x.kicad_pcb', str(plain['final_board']))
        r = check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'tests', 'stress', 'redo_stress_test.py'),
                   old, '--workdir', os.path.join(tmp, 'replay'), '--dry-run'],
                  accept=True, allow=('unrecognized arguments',))  # it quotes the refusal
        ok("a dry-run replay names the refusal",
           'refused [2]' in r.stdout, r.stdout[-600:])
        ok("...and keeps good.kicad_pcb final",
           'Final board: good.kicad_pcb' in r.stdout, r.stdout[-600:])
        import manifest_to_plan as m2p
        keep2, rej2 = m2p.prune_manifest(cmds, old)
        ok("manifest_to_plan prunes the same", keep2 == {0} and list(rej2) == [1],
           f"{keep2} {rej2}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()

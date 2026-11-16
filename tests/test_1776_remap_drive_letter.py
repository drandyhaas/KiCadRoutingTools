#!/usr/bin/env python3
"""#1776: `redo_stress_test --remap OLD:NEW` splits at the colon BETWEEN the
two paths, not at a Windows drive letter's.

`r.split(":", 1)` read `C:/a:C:/b` as old = 'C', new = '/a:C:/b': every C in
the manifest was rewritten and a folder named '/a:C:/b' was made. Now the
split skips a leading drive letter on either side.

  1. split_remap on drive-letter, POSIX and malformed rules;
  2. a dry-run replay with `--remap <recorded>:<dest>` spelled as this
     machine's absolute paths (drive letters on Windows) prints that remap
     and keeps the whole chain in the destination;
  3. a rule with no separating colon is refused by name.

    python3 tests/test_1776_remap_drive_letter.py
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
FAILS = []


def ok(what, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {what}" + (f"  [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(what)


def main():
    from redo_stress_test import split_remap
    print('1: split_remap')
    for rule, want in (('C:/a:C:/b', ('C:/a', 'C:/b')),
                       ('C:\\runs\\x:D:\\replay', ('C:\\runs\\x', 'D:\\replay')),
                       ('/runs/x:/tmp/replay', ('/runs/x', '/tmp/replay')),
                       ('C:/runs/x:/tmp/replay', ('C:/runs/x', '/tmp/replay')),
                       ('runs/x:C:/replay', ('runs/x', 'C:/replay')),
                       ('C:/runs/x', None), ('old:', None), (':new', None), ('nocolon', None)):
        ok(f'{rule!r} -> {want!r}', split_remap(rule) == want, repr(split_remap(rule)))

    tmp = tempfile.mkdtemp(prefix='t1776r_')
    try:
        rec = os.path.join(tmp, 'run')          # the recorded folder: gone
        arch = os.path.join(tmp, 'archive')
        dest = os.path.join(tmp, 'dest')
        os.makedirs(arch)
        shutil.copy2(evidence(BOARD, 'source board'), os.path.join(arch, 'in.kicad_pcb'))
        q = shlex.quote
        route = q(tool('route.py'))
        man = os.path.join(arch, 'redo_commands.sh')
        with open(man, 'w', encoding='utf-8') as f:
            f.write(f"# cwd={q(rec)}\n"
                    f"python3 {route} in.kicad_pcb {q(os.path.join(rec, 's1.kicad_pcb'))}"
                    f" --nets DPA_P\n"
                    f"# cwd={q(rec)}\n"
                    f"python3 {route} {q(os.path.join(rec, 's1.kicad_pcb'))} s2.kicad_pcb"
                    f" --nets DPB_P\n")
        evidence(man, 'manifest')

        print('2: a dry-run replay remapped by absolute paths')
        out = check([sys.executable, '-X', 'utf8', REPLAY, man,
                     '--remap', f'{rec}:{dest}', '--dry-run'], accept=True).stdout
        ok('the remap is read whole', f'Remaps: {rec} -> {dest}' in out, out[-900:])
        ok('the destination is made', os.path.isdir(dest))
        ok('both commands are kept', 'running 2 of 2 command(s)' in out, out[-900:])
        ok('the absolute board path lands in the destination',
           os.path.join(dest, 's1.kicad_pcb') in out
           and os.path.join(rec, 's1.kicad_pcb') not in out, out[-900:])
        ok('no command runs outside the remap', 'not covered by --remap' not in out
           and 'ABORT' not in out, out[-900:])

        print('3: a rule with no separating colon')
        check([sys.executable, '-X', 'utf8', REPLAY, man, '--remap', dest, '--dry-run'],
              refuse='--remap needs OLD:NEW', code=2, allow=('error: argument',))
        ok('is refused by name', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()

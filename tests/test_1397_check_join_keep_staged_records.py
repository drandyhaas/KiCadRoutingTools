#!/usr/bin/env python3
"""#1397: check_join --keep-staged writes a board a chain continues from, so
it records itself into redo_commands.sh like copy_board does; a pure check
(no --keep-staged) writes nothing and records nothing.

    python3 tests/test_1397_check_join_keep_staged_records.py
"""
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
from run_utils import check, evidence, tool        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
JOIN = ['DPA_P', '10,15,F.Cu', '14,15,F.Cu', '14,13,F.Cu', '17.52,13,F.Cu']


def main():
    evidence(BOARD, 'cap_chain board')
    tmp = tempfile.mkdtemp(prefix='t1397_')
    try:
        inb = os.path.join(tmp, 'in.kicad_pcb')
        shutil.copy(BOARD, inb)
        base = dict(os.environ)
        # kept board: manifest gets exactly one check_join line
        d1 = os.path.join(tmp, 'kept')
        os.makedirs(d1)
        env = dict(base, REDO_RUNDIR=d1,
                   REDO_MANIFEST=os.path.join(d1, 'redo_commands.sh'))
        out = os.path.join(d1, 'joined.kicad_pcb')
        check([sys.executable, '-X', 'utf8', tool('check_join.py'), inb,
               *JOIN, '--keep-staged', out], accept=True, env=env)
        evidence(out, 'kept staged board')
        man = evidence(os.path.join(d1, 'redo_commands.sh'),
                       'redo manifest (check_join --keep-staged recorded nothing)')
        lines = [l for l in open(man) if 'check_join.py' in l]
        assert len(lines) == 1 and '--keep-staged' in lines[0], lines
        # pure check: no manifest
        d2 = os.path.join(tmp, 'pure')
        os.makedirs(d2)
        env = dict(base, REDO_RUNDIR=d2,
                   REDO_MANIFEST=os.path.join(d2, 'redo_commands.sh'))
        check([sys.executable, '-X', 'utf8', tool('check_join.py'), inb,
               *JOIN], accept=True, env=env)
        assert not os.path.exists(os.path.join(d2, 'redo_commands.sh')), \
            'a pure check must not record'
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('PASS')


if __name__ == '__main__':
    main()

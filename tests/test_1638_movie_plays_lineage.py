#!/usr/bin/env python3
"""#1638: a run-dir movie plays the LINEAGE of the final board, not every
board the manifest writes.

`route_movie.run_boards` -- read by route_movie, render_run.final_board_of
and place_movie -- listed every board a manifest's commands write, so a try
the run went back from was filmed, and a run whose last try was rejected
ended on it. It now plays the boards a replay of the chain makes
(`redo_stress_test.compute_prune_keep`), ending on the final board: `final`
(route_movie's `--final`) when given, else the last board written whose name
says `final`, else the last board written.

The boards are copies of one board -- the chain is read off the manifest, and
nothing here routes.

  1. route -> try (rejected) -> kept -> try (rejected), last writer final:
     the rejected middle try is not played;
  2. `final=` the kept board: 01_route, 03_kept, ending on it; a board no
     command writes is refused by name, and route_movie's CLI says so;
  3. a `final*` copy of the kept board at the end: its lineage, and
     render_run.final_board_of agrees;
  4. a check in the manifest is never a board of the movie.

    python3 tests/test_1638_movie_plays_lineage.py
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
from run_utils import check, evidence        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
FAILS = []


def ok(what, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}  {what}" + (f"  [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(what)


def names(paths):
    return [os.path.basename(p)[:-len('.kicad_pcb')] for p in paths]


def write_run(d, steps):
    """A run dir whose manifest records `steps` [(tool, in, out)], each out a
    copy of the board."""
    os.makedirs(d)
    with open(os.path.join(d, 'redo_commands.sh'), 'w', encoding='utf-8') as f:
        f.write('#!/bin/bash\nset -e\n')
        for t, i, o in steps:
            f.write(f"# cwd={shlex.quote(d)}\n")
            f.write(f"python3 -u -X utf8 py_router/{t} {i} {o} --nets X\n")
            if o.endswith('.kicad_pcb'):
                shutil.copy(BOARD, os.path.join(d, o))


def main():
    import route_movie as rm
    from render_run import final_board_of
    evidence(BOARD, 'cap_chain board')
    tmp = tempfile.mkdtemp(prefix='t1638_')
    try:
        chain = [('route.py', BOARD, '01_route.kicad_pcb'),
                 ('route.py', '01_route.kicad_pcb', '02_try.kicad_pcb'),
                 ('route.py', '01_route.kicad_pcb', '03_kept.kicad_pcb'),
                 ('route.py', '01_route.kicad_pcb', '04_try.kicad_pcb')]
        print("1: the last writer final")
        d = os.path.join(tmp, 'a')
        write_run(d, chain)
        got = names(rm.run_boards(d))
        ok("its lineage only", got == ['01_route', '04_try'], str(got))

        print("2: final= the kept board")
        got = names(rm.run_boards(d, final='03_kept.kicad_pcb'))
        ok("01_route, then 03_kept", got == ['01_route', '03_kept'], str(got))
        got = names(rm.run_boards(d, final=os.path.join(d, '03_kept.kicad_pcb')))
        ok("a path names it too", got == ['01_route', '03_kept'], str(got))
        try:
            rm.run_boards(d, final='nope.kicad_pcb')
            ok("a board no command writes is refused", False)
        except ValueError as e:
            ok("a board no command writes is refused", 'nope.kicad_pcb' in str(e),
               str(e))
        check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router',
               'route_movie.py'), d, '--final', 'nope.kicad_pcb'],
              refuse='writes nope.kicad_pcb', code=2)

        print("3: a final* copy at the end")
        d = os.path.join(tmp, 'b')
        write_run(d, chain + [('copy_board.py', '03_kept.kicad_pcb',
                               'final.kicad_pcb')])
        got = names(rm.run_boards(d))
        ok("the kept board's lineage, then final",
           got == ['01_route', '03_kept', 'final'], str(got))
        ok("render_run.final_board_of agrees",
           os.path.basename(final_board_of(d) or '') == 'final.kicad_pcb',
           str(final_board_of(d)))

        print("4: a check is never a board of the movie")
        d = os.path.join(tmp, 'c')
        write_run(d, chain[:1] + [('check_drc.py', '01_route.kicad_pcb', '')])
        got = names(rm.run_boards(d))
        ok("the check plays nothing", got == ['01_route'], str(got))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        sys.exit(1)
    print('PASS')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Run `place_seed` with one edge connector's first seat displaced -- the #982
control.

argv: N place_seed.py ARGS...  -- displaces the FIRST pose `apply_move` is asked
for, for the first declared edge connector it sees, by N mm in y, and then runs
`place_seed.main()` in this process.

Why it exists: `pad_conflicts_seeded` moved 0 -> 1 on ulx3s when #975 seated
AUDIO1 0.386 mm further inward. That could be read as #975 breaking something.
This shows the count moves for ANY displacement of that size on the UNMUTATED
engine, so the cause is a hole landing on an unseated part's written copper --
which is what #982 is about. Used by `measure_982_unseated_pairs.py --sham`.

NOT named `test_*.py`: it is an instrument, and it patches the engine in
process.
"""
from __future__ import annotations

import os
import sys

_TESTS = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_TESTS)
for _p in (_ROOT, os.path.join(_ROOT, 'py_router'), os.path.join(_ROOT, 'py_placer'),
           os.path.join(_ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def main():
    shift = float(sys.argv[1])
    argv = sys.argv[2:]
    from placement import quench
    original = quench.QuenchState.apply_move
    done = {'ref': None}

    def apply_move(self, ref, x, y, rot, *a, **k):
        # The first move of the first ref that asks for one, once. Which ref
        # that is depends on the board's own declaration order, and the run
        # prints it, so a reader is never guessing which part moved.
        if shift and done['ref'] is None:
            done['ref'] = ref
            print(f"  SHAM: {ref} first seat displaced {shift:+.3f}mm in y")
            y = round(y + shift, 3)
        return original(self, ref, x, y, rot, *a, **k)

    quench.QuenchState.apply_move = apply_move
    sys.argv = list(argv)
    import runpy
    try:
        runpy.run_path(argv[0], run_name='__main__')
    except SystemExit as exc:
        return exc.code or 0
    finally:
        quench.QuenchState.apply_move = original
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""pick_braid.py NETS BOARD [BOARD...] -- the braid A/B verdict.

Grades each candidate with `grade_k.py` (the chain's own grade, scoped to
the run's nets) and prints the path of the best by **(open, vias)** --
completion first, as every grade in this chain is. The reasoning is
printed on stderr so the driver's stdout is just the winner.

A board that does not GRADE is not a candidate: `grade_k` refuses a
missing or truncated file loudly, and a picker that read that silence as
"0 open, 0 vias" would choose it every time (run_utils.evidence's rule).
Prints nothing and exits 1 when no candidate graded, so the caller can
keep its own arm rather than act on an empty string.
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def grade(board, nets):
    r = subprocess.run([sys.executable, os.path.join(HERE, 'grade_k.py'), board, nets],
                       capture_output=True, text=True)
    txt = r.stdout + r.stderr
    mo, mv = re.search(r'open=(\d+)', txt), re.search(r'vias=(\d+)', txt)
    if not mo or not mv or 'BROKEN' in txt:
        return None, (txt.strip().splitlines() or ['(no output)'])[-1][:120]
    return (int(mo.group(1)), int(mv.group(1))), txt.strip().splitlines()[-1]


def main(argv):
    if len(argv) < 3:
        sys.exit('usage: pick_braid.py NETS BOARD [BOARD...]')
    nets, boards = argv[1], argv[2:]
    best, scored = None, []
    for b in boards:
        v, line = grade(b, nets)
        scored.append((v, b, line))
        print(f'  braid A/B: {os.path.basename(b)}: '
              + (f'{v[1]} via(s), {v[0]} open' if v else f'NO GRADE -- {line}'),
              file=sys.stderr)
        if v is not None and (best is None or v < best[0]):
            best = (v, b)
    if best is None:
        return 1
    print(f'  braid A/B: keeping {os.path.basename(best[1])} '
          f'({best[0][1]} via(s), {best[0][0]} open)', file=sys.stderr)
    print(best[1])
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

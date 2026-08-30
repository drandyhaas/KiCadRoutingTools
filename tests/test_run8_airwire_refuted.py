#!/usr/bin/env python3
"""A refuted vector source, kept as its measurement.

Run 7 endorsed a third vector source for reconstruct: cluster the STRETCHED
AIRWIRES of small nets, on the theory that a translate stretches them all by
the damage vector while a healthy board's long airwires point in unrelated
directions.

Implemented and measured, it fails both ways at once: healthy boards fire, and
on a board translated by a known vector the top cluster is not that vector.

The repo's own A/B doctrine says a rejected term keeps its rows, marked
rejected with its measured numbers, so it stays a change detector instead of
becoming folklore someone re-proposes. Same here: the function stays in the
tree, is NOT wired into the driver, and these numbers are pinned.

Run: python3 -X utf8 tests/test_run8_airwire_refuted.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

# Fast (~4 s) despite importing run_utils for corpus_boards, which makes one
# cheap `git ls-files` call. Without this, --fast silently stops running it.
RUN_ALL_FAST_OK = True

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import reconstruct                              # noqa: E402
import pose_score                                              # noqa: E402
from run_utils import corpus_boards                            # noqa: E402

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def vectors_for(path, clearance=0.1):
    pcb = parse_kicad_pcb(path)
    st = pose_score.make_state(pcb, path, clearance=clearance,
                               board_edge_clearance=0.2, grid_step=0.1)
    return reconstruct.airwire_cluster_vectors(st)


def main():
    print('it is not wired into the driver')
    drv = open(os.path.join(ROOT, 'py_placer', 'place_reconstruct.py'),
               encoding='utf-8').read()
    check('the driver does not call it', 'airwire_cluster_vectors' not in drv)
    doc = reconstruct.airwire_cluster_vectors.__doc__ or ''
    check('the function says REFUTED in its first line',
          doc.strip().startswith('REFUTED'), doc[:80])
    # Parse the HEADLINE line, not a substring of the whole docstring: the
    # prose below it legitimately quotes other counts, so `'18 of 22' in doc`
    # stays true when the headline itself is edited -- measured, that mutation
    # passed. The captured numbers are compared to a re-measurement further
    # down, so the doc and the code cannot disagree silently.
    m = re.search(r'healthy boards firing\s+(\d+) of (\d+)', doc)
    check('...and carries the numbers that refuted it',
          m is not None and '60mm' in doc,
          f'headline not parseable from the docstring: {doc[:80]!r}')
    recorded = (int(m.group(1)), int(m.group(2))) if m else None

    print('why: healthy boards fire')
    # The TRACKED corpus, never a glob: kicad_files/ also fills with generated
    # boards, so a glob gives 22 / 27 / 33+ depending on how used the working
    # copy is. The docstring's count is a claim about a FIXED set, so the set
    # has to be fixed too (run_utils.corpus_boards).
    boards = corpus_boards()
    if not boards:
        print('  SKIP  git cannot identify the tracked corpus; refusing to '
              'grade a recorded count against a set it cannot name')
        return 0
    firing = []
    for f in boards:
        if vectors_for(f):
            firing.append(os.path.basename(f))
    check('a healthy board firing at all disqualifies it as a source',
          len(firing) >= 3,
          f'{len(firing)} of {len(boards)} fire: {firing[:6]}')
    # RE-MEASURED, not grepped. The docstring claims this file "pins these
    # numbers ... so a change that alters the behaviour is visible", and the
    # old form could not do that: it asserted the prose contained a substring,
    # so the recorded `6 of 33` sat there contradicted by the code -- at these
    # same defaults, at the very commit that wrote it -- and the loose
    # `>= 3` above could not see the difference either. An exact pin is the
    # point: if this fires, RE-MEASURE and update BOTH this row and the
    # docstring, rather than widening the tolerance.
    measured = (len(firing), len(boards))
    check('the docstring headline agrees with a re-measurement',
          recorded == measured,
          f'docstring says {recorded}, code measures {measured} -- re-measure '
          f'and update the docstring headline')
    # ...and the ABSOLUTE value too, so editing BOTH the docstring and the code
    # together cannot quietly re-baseline a refutation. Same discipline as the
    # correlated-revert row in test_check_weird: two mirrored numbers agreeing
    # is not evidence that either is right. If this fires, re-measure and
    # update BOTH -- deliberately, not to clear a red.
    check('the recorded count is still the one that was refuted',
          measured == (18, 22),
          f'pinned (18, 22), measured {measured}')

    print('and the vector it finds on real damage is not the damage')
    dmg = os.path.join(ROOT, 'wk', 'run7', 'glasgow_revC',
                       'perturbed.kicad_pcb')
    if os.path.isfile(dmg):
        got = vectors_for(dmg, 0.09)
        check('it does produce a confident cluster', bool(got), str(got[:1]))
        if got:
            vx, vy = got[0]['v']
            # The recorded truth for that board is (4.5, -2.4).
            miss = ((vx - 4.5) ** 2 + (vy + 2.4) ** 2) ** 0.5
            check('...and it misses the true vector by a wide margin',
                  miss > 5.0, f'top cluster {(vx, vy)}, miss {miss:.1f}mm')
    else:
        print('  SKIP  recorded damaged board not present; the measured '
              'result is in the docstring')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

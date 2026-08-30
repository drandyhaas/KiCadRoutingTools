#!/usr/bin/env python3
"""Copper landing on a LOCKED part is not a placement search's to settle (E6).

A locked pose is a decision somebody made outside this toolchain: a standoff
against an enclosure, a connector against a panel cut-out, a hole in a pattern
the mechanical drawing owns. When a placement lands other copper on top of one,
"move the other part somewhere it likes better" is not a resolution, and the
waiver classes chosen for unlocked parts do not apply.

This is the first of the three counter-gates for WRONG-BASIN convergence: a
placement that grades DRC-0 and oracle-consistent while sitting ~2.4mm from the
truth it was asked to reconstruct. Legality alone cannot see it, because the
wrong basin is legal. Measured:

    a wrong-basin placement      1 locked-contact pair
    its TRUTH control            0
    its damaged input            3
    every tracked corpus board  0   (22 boards; see below)

That last line is what makes it gateable rather than advisory.

Also pinned here: the run-7 report that this channel false-positives on
SAME-NET contact is REFUTED. The disputed pair really was same-net -- a 0402's
whole pad inside a connector pad on the same GND -- and it is still a defect.
DRC exempts same-net because DRC grades electrical clearance; this channel
grades whether both parts can be built, and two parts cannot occupy the same
copper whatever the net. The pair of stacked 0402s the channel was calibrated
on were same-net too.

Run: python3 -X utf8 tests/test_run8_locked_contact.py
"""
import os
import re
import shutil
import sys
import tempfile

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
from placement.legality import grade_body_overlap              # noqa: E402
from run_utils import corpus_boards                            # noqa: E402

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def lock_ref(path, ref):
    text = open(path, encoding='utf-8').read()
    pat = re.compile(r'\(footprint\s')
    idx = 0
    while True:
        m = pat.search(text, idx)
        if not m:
            raise AssertionError(f'{ref} not found')
        nxt = pat.search(text, m.end())
        end = nxt.start() if nxt else len(text)
        chunk = text[m.start():end]
        if re.search(r'\(property\s+"Reference"\s+"%s"' % re.escape(ref), chunk):
            ins = chunk.index('\n') + 1
            chunk = chunk[:ins] + '\t\t(locked yes)\n' + chunk[ins:]
            open(path, 'w', encoding='utf-8', newline='').write(
                text[:m.start()] + chunk + text[end:])
            return
        idx = m.end()


def main():
    print('the channel is silent on every healthy in-repo board')
    # The TRACKED corpus, never a glob of the directory. kicad_files/ also
    # accumulates GENERATED boards (routed/fanout/plane outputs, several of
    # them gitignored), so a glob returns 22 entries on a clean clone, 27 on a
    # lightly used one and 33+ on a machine that has run the suite for a
    # while. The guard below used to read `quiet >= 30`, which no clean
    # checkout can satisfy -- it failed on every fresh clone and in CI while
    # passing on the author's machine, so a TEST bug reported as a product one.
    boards = corpus_boards()
    if not boards:
        print('  SKIP  git cannot identify the tracked corpus (tarball '
              'export?); refusing to grade against a set it cannot name')
        return 0
    fires, quiet = [], 0
    for f in boards:
        g = grade_body_overlap(parse_kicad_pcb(f), 0.1, pcb_file=f)
        if g['locked_contact_pairs']:
            fires.append(os.path.basename(f))
        else:
            quiet += 1
    check(f'{quiet} board(s) silent, 0 firing', not fires, str(fires))
    # Anti-vacuity only: the claim above is "silent on every board", so this
    # needs to catch a gutted or partial checkout, not pin an exact count that
    # churns whenever a board is added. A FLOOR under the tracked set (22)
    # does that and stays true when the corpus grows.
    check('the corpus is the one the channel was calibrated on',
          len(boards) >= 20, f'{len(boards)} tracked boards')

    print('it fires when copper lands on a locked part')
    with tempfile.TemporaryDirectory() as tmp:
        board = os.path.join(tmp, 'b.kicad_pcb')
        shutil.copy(os.path.join(ROOT, 'kicad_files',
                                 'splitflap_driver.kicad_pcb'), board)
        from placement.perturb import perturb
        dmg = os.path.join(tmp, 'd.kicad_pcb')
        perturb(board, dmg, kind='translate', seed=31, dose_mm=7.0)

        g0 = grade_body_overlap(parse_kicad_pcb(dmg), 0.1, pcb_file=dmg)
        check('the damaged board has overlapping pads to work with',
              g0['blocking'] > 0, f"blocking={g0['blocking']}")
        check('...but no locked part is involved yet',
              not g0['locked_contact_pairs'])

        victim = g0['blocking_pairs'][0].a
        lock_ref(dmg, victim)
        g1 = grade_body_overlap(parse_kicad_pcb(dmg), 0.1, pcb_file=dmg)
        lc = g1['locked_contact_pairs']
        check('locking one member makes the pair a locked contact',
              any(victim in (p.a, p.b) for p in lc), str([(p.a, p.b) for p in lc]))
        check('the pair names which member is locked',
              any(victim in (p.locked_ref or '') for p in lc),
              str([p.locked_ref for p in lc]))
        check('the blocking count did not change (this is a second channel, '
              'not a re-count)', g1['blocking'] == g0['blocking'])

    print('a same-net overlap is still a defect (finding REFUTED)')
    src = open(os.path.join(ROOT, 'py_placer', 'placement', 'legality.py'),
               encoding='utf-8').read()
    check('the refutation is recorded where the policy lives',
          'REFUTED' in src and 'same-net' in src.lower())
    check('different-net intersections are disclosed rather than re-derived',
          'shorts' in src and 'SHORT' in open(
              os.path.join(ROOT, 'py_tools', 'check_assembly.py'), encoding='utf-8').read())

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

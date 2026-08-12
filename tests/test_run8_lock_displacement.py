#!/usr/bin/env python3
"""A displaced part must never be recommended for a lock.

Found by running the combined placement+routing skill on a deliberately
scrambled board. The placement half's second stage says "separate the parts
whose position is NOT a netlist question" and runs the lock advisor. On that
board the advisor promoted to HIGH -- and printed in its paste-ready `--lock`
line -- three parts whose ONLY qualifying reason was:

    courtyard leaves the board outline by 44.44 mm
    courtyard leaves the board outline by 21.80 mm
    courtyard leaves the board outline by  8.88 mm

Those were the scrambled parts. Pasting that line freezes the damage as a
decision, and every later stage then treats the wrong pose as ground truth --
the wrong-basin failure, handed to the executor by a tool it was told to trust.

The rule itself is sound for the case it was written for: an edge connector or
a castellated module DOES leave the outline, and that overhang is a real signal
that its position is a mechanical fact. What was missing is that the same
measurement has two opposite meanings:

    straddling  -- pads on the board, shell past the edge   -> mechanical fact
    wholly off  -- touching nothing                         -> displacement

So the test is not the magnitude, it is whether any of the body is still on the
board. Magnitude alone cannot separate them: a big connector on a small board
can out-hang a small part parked just past the edge.

Note what this does NOT fix, deliberately: a part hanging PARTLY off an edge is
geometrically indistinguishable from a genuine edge-mounted part, so it stays
HIGH. Separating those needs corroboration this rule does not have.

Run: python3 -X utf8 tests/test_run8_lock_displacement.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ['KRT_NO_BANNER'] = '1'

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement.lock_advisor import advise_locks, _straddles_outline  # noqa: E402

BOARD = os.path.join('kicad_files', 'splitflap_driver.kicad_pcb')

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def advise(pcb):
    return advise_locks(pcb, None, min_confidence='medium')


def by_ref(adv):
    return {f.ref: f for f in adv.findings}


def main():
    pcb = parse_kicad_pcb(BOARD)
    bounds = pcb.board_info.board_bounds
    check('the reference board has bounds', bounds is not None)

    base = by_ref(advise(pcb))
    base_high = {r for r, f in base.items() if f.confidence == 'high'}
    print(f'  (undamaged: {len(base_high)} HIGH finding(s))')

    # Pick a part the undamaged board does NOT already flag, so the assertion
    # below is about the displacement and not about what the part is.
    victim = next((r for r in sorted(pcb.footprints)
                   if r not in base and pcb.footprints[r].pads), None)
    check('there is an unflagged part to displace', victim is not None,
          'every part is already a lock finding on the clean board')
    if victim is None:
        return 1

    # Park it well clear of the board, the way a swap or a bad import does.
    fp = pcb.footprints[victim]
    home = (fp.x, fp.y)
    fp.x = bounds[0] - 60.0
    fp.y = bounds[1] - 60.0
    for pad in fp.pads:
        pad.global_x = fp.x + (pad.local_x or 0.0)
        pad.global_y = fp.y + (pad.local_y or 0.0)

    dmg = by_ref(advise(pcb))
    f = dmg.get(victim)
    check(f'the displaced part is reported at all ({victim})', f is not None)
    if f is None:
        return 1

    check('...and is NOT high confidence', f.confidence != 'high',
          f'confidence={f.confidence} rules={f.rules}')
    check('...and is classified as displacement, not an edge seat',
          'off_board_entirely' in f.rules and 'off_board_overhang' not in f.rules,
          str(f.rules))
    check('...and says why, in the finding itself',
          any('WHOLLY off the board' in r for r in f.reasons))
    pats = advise(pcb).lock_patterns()
    check('...and the paste-ready lock line does not carry it',
          victim not in pats, f'--lock {" ".join(pats)}')

    # The parts that were HIGH for real reasons must stay HIGH: this is a
    # demotion of one specific misreading, not a general weakening.
    still = {r for r, ff in dmg.items() if ff.confidence == 'high'}
    check('every other HIGH finding survives unchanged',
          base_high - {victim} <= still,
          f'lost: {sorted(base_high - {victim} - still)}')

    fp.x, fp.y = home

    print('the straddle test itself')

    class _RectGate:
        outer = None
        cutouts = ()

        def __init__(self, b):
            self.bounds = b

    g = _RectGate((0.0, 0.0, 100.0, 50.0))
    check('a body wholly outside does not straddle',
          _straddles_outline(g, (-30.0, -30.0, -20.0, -20.0)) is False)
    check('a body hanging over an edge does straddle',
          _straddles_outline(g, (90.0, 20.0, 115.0, 30.0)) is True)
    check('a body fully inside straddles (it is not off-board at all)',
          _straddles_outline(g, (10.0, 10.0, 20.0, 20.0)) is True)
    check('no bounds and no outline answers None, not a guess',
          _straddles_outline(_RectGate(None), (0.0, 0.0, 1.0, 1.0)) is None)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

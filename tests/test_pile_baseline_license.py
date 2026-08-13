#!/usr/bin/env python3
"""A pile-degenerate seed must not license pad intersections.

Run 19, cycle 1: every conjunct of `LegalityContext.pads_ok` is relative to
`seed_baseline`, and on a PILE the seed pair carries huge `base.pad` with
`pad_overlap` and `stack` both True -- so ANY smaller intersection passed,
producing the SW23/SW28<->U2 blocker pairs on the shipped board.

The fix sits at the single choke point every consumer routes through
(`seed_baseline`, feeding `pads_ok` and `swap_pads_ok`): a seed bucket of
>= 3 parts at one round(coord, 3) point is the pile signature, and every part
in one gets the ZERO baseline -- its pose must be absolutely clean toward
every neighbor. Deliberately >= 3, not >= 2: a legitimate two-part by-design
overlap (run 18's under-module R1<->U2 pairs) KEEPS its baseline license,
and this test pins BOTH sides of that line.

Run: python3 -X utf8 tests/test_pile_baseline_license.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

from kicad_parser import Footprint, Pad                        # noqa: E402
from placement import legality                                 # noqa: E402

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


_NET = iter(range(1, 100))


def two_pad_part(ref):
    """A 2-pad 0603-ish SMD part in the part frame (fp at origin), every pad
    on its own net so cross-part contact is a different-net intersection."""
    pads = [Pad(ref, str(i + 1), x, 0.0, x, 0.0, 0.8, 0.9, 'rect',
                ['F.Cu'], next(_NET), f'/N{ref}{i}', pad_type='smd')
            for i, x in enumerate((-0.5, 0.5))]
    return Footprint(ref, f'test:{ref}', 0.0, 0.0, 0.0, 'F.Cu', pads=pads)


def main():
    fps = {r: two_pad_part(r) for r in ('A', 'B', 'C', 'D', 'E')}
    part_pads = legality.build_part_pads(fps, 0.2)
    # A, B, C piled at ONE seed point (>= 3: degenerate). D, E share a seed
    # point too, but as a PAIR (run 18's by-design under-module overlap).
    seed = {'A': (10.0, 10.0, 0.0), 'B': (10.0, 10.0, 0.0),
            'C': (10.0, 10.0, 0.0),
            'D': (50.0, 50.0, 0.0), 'E': (50.0, 50.0, 0.0)}
    cur = {'A': (20.0, 10.0, 0.0), 'B': (30.0, 10.0, 0.0),
           'C': (40.0, 10.0, 0.0),
           'D': (50.0, 50.0, 0.0), 'E': (50.0, 50.0, 0.0)}
    ctx = legality.LegalityContext(part_pads, None, 0.2,
                                   pose_of=lambda r: cur[r],
                                   seed_of=lambda r: seed[r])

    print('the pile (3 parts at one seed point) licenses NOTHING')
    check('a piled part\'s baseline is ZERO_SHORTFALL',
          ctx.seed_baseline('A', 'B') is legality.ZERO_SHORTFALL)
    # A proposes B's exact current pose: a full different-net pad
    # intersection. Pre-fix the pile baseline carried the same intersection
    # (cur.pad == base.pad) and pads_ok ACCEPTED this pose.
    check('pads_ok REFUSES a pose carrying a pad intersection',
          ctx.pads_ok('A', 30.0, 10.0, 0.0, ['B']) is False)
    check('a clean pose is still accepted for a piled part',
          ctx.pads_ok('A', 20.0, 10.0, 0.0, ['B', 'C']) is True)

    print('the 2-part by-design overlap KEEPS its license (run 18)')
    base_de = ctx.seed_baseline('D', 'E')
    check('the designed pair\'s baseline still carries its overlap',
          base_de.pad > 0 and base_de.stack, str(base_de))
    check('pads_ok still admits the designed pose',
          ctx.pads_ok('D', 50.0, 50.0, 0.0, ['E']) is True)

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

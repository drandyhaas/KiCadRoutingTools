#!/usr/bin/env python3
"""`candidate_valid` refuses a seat that buries a part in another part's body.

`reconstruct._pair_conflicts` refuses an assign/exchange CANDIDATE, but
`candidate_valid` is what `pose_ok`, `_try_place`, `legalize` and every quench
pass actually accept a pose with -- and it was body-blind. A part could be
re-seated straight back into the body it had just been charged for, and
`_try_place`'s clearance ladder (full, full/2, 0.02) makes that seat MORE
reachable, not less.

**The fixture had to be hunted, because the obvious ones prove nothing.** On
tigard and watchy the term blocks NOTHING: a courtyard contains its own body,
so the courtyard and pad conjuncts already reject every containment pose there.
Measured, same-side non-exempt containment poses accepted by `candidate_valid`:

    board                gate ON   gate OFF   blocked by this term
    tigard                     0          0     0   (already rejected)
    watchy                     0          0     0   (already rejected)
    esp_prog                   0          0     0   (already rejected)
    orangecrab_ext_pll         0        696   696
    ulx3s                    114      27132   27018

(The zero rows are NOT "no such pose exists" -- tigard sweeps 139183 such
poses, esp_prog 1776. Every one is already refused by the courtyard or pad
conjunct, which is the whole reason this term looks inert until you find a
board where it is not.)

So the subject is **ulx3s**, where the term is load-bearing, and the pose is one
found by sweeping for `gate ON rejects AND gate OFF accepts` -- the only
definition of a non-vacuous fixture here. Two earlier candidates were rejected
during development because BOTH arms said False: another conjunct was doing the
work and the test would have passed with the term deleted.

(ulx3s's 114 residue is not a hole: every one sits at exactly frac 0.500, the
`CONTAINMENT_FRAC` boundary, where the sweep's file-parsed rect and the state's
own `fab_rect` disagree in the last bit.)

Run: python3 -X utf8 tests/test_run22_seat_containment_gate.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_tools', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _p))
os.environ.setdefault('KRT_NO_BANNER', '1')

from kicad_parser import parse_kicad_pcb                       # noqa: E402
from placement import quench as Q                              # noqa: E402
from placement.quench import QuenchState                       # noqa: E402
from placement.legality import CONTAINMENT_FRAC                # noqa: E402
import routing_defaults as defaults                            # noqa: E402

SKIP_EXIT = 77
ULX3S = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
TIGARD = os.path.join(ROOT, 'kicad_files', 'tigard.kicad_pcb')

# C2 wholly inside U9's body (U9 spans x 108.300..126.300, y 87.015..118.415).
BURIED = (109.3, 87.515)
# Walking C2 out of U9's right edge: 0.500 is the last contained step, 0.375
# the first free one. The threshold is `>=`, so both sides are asserted.
AT_THRESHOLD = (126.3, 87.515)     # frac exactly 0.500
KISS = (126.7, 87.515)             # frac 0.250
# tigard: U5's body centre. D3 buried there is the run-22 defect shape; TP1 at
# the SAME point is the paired control -- a testpoint, so marker_class.
U5_CENTRE = (62.27, 60.90)

FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def state(path):
    return QuenchState(parse_kicad_pcb(path), path, clearance=0.15,
                       board_edge_clearance=0.55, crossing_penalty=10.0,
                       halo_base=0.5, halo_coef=0.25, halo_weight=2.0,
                       edge_halo=2.0, edge_weight=2.0,
                       grid_step=defaults.GRID_STEP, length_weight=1.0)


def main():
    if not os.path.exists(ULX3S) or not os.path.exists(TIGARD):
        print('SKIP: corpus boards not present')
        return SKIP_EXIT

    was = Q._CONTAINMENT_GATE
    try:
        st = state(ULX3S)

        print('the seat gate refuses a buried pose -- and ONLY it does')
        Q._CONTAINMENT_GATE = True
        check('the pose IS a containment by the engine s own term',
              st._body_contained_at('C2', BURIED[0], BURIED[1], 0.0))
        check('candidate_valid refuses it',
              not st.candidate_valid('C2', BURIED[0], BURIED[1], 0.0))
        # THE anti-vacuity assertion, and the reason this file names ulx3s
        # rather than tigard. If another conjunct also rejected this pose, the
        # test would pass with the term deleted and prove nothing.
        Q._CONTAINMENT_GATE = False
        check('...and with the term disabled it is ACCEPTED, so the term is '
              'what rejected it',
              st.candidate_valid('C2', BURIED[0], BURIED[1], 0.0),
              'another conjunct now rejects this pose too -- the fixture has '
              'gone vacuous and must be re-hunted, not relaxed')
        Q._CONTAINMENT_GATE = True

        print('it does not over-refuse: a KISS is not a containment')
        check('a 0.25-frac overlap is not charged',
              not st._body_contained_at('C2', KISS[0], KISS[1], 0.0))
        check('...while the threshold step itself is',
              st._body_contained_at('C2', AT_THRESHOLD[0], AT_THRESHOLD[1],
                                    0.0),
              f'CONTAINMENT_FRAC is {CONTAINMENT_FRAC} and the comparison is '
              f'>=, so frac 0.500 must count')

        print('the exemption reaches the SEAT path too')
        # Or a displaced fiducial could never come home under a connector --
        # orangecrab ships FID2 inside J5 at frac 1.000 by design.
        tg = state(TIGARD)
        check('D3 buried in U5 is charged',
              tg._body_contained_at('D3', U5_CENTRE[0], U5_CENTRE[1], 0.0))
        check('...but TP1 at the SAME point is not (marker_class)',
              not tg._body_contained_at('TP1', U5_CENTRE[0], U5_CENTRE[1],
                                        0.0),
              'only the part CLASS differs between these two calls')

        print('a healthy board is undisturbed')
        d3 = parse_kicad_pcb(TIGARD).footprints['D3']
        check('a part at its own shipped pose is not contained',
              not tg._body_contained_at('D3', d3.x, d3.y, d3.rotation or 0.0))
        check('...and its own pose is still a valid candidate',
              tg.candidate_valid('D3', d3.x, d3.y, d3.rotation or 0.0))
    finally:
        Q._CONTAINMENT_GATE = was

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

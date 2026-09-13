#!/usr/bin/env python3
"""`place_seed --rotate-by-facing`: among the rotations that fit, seat the one
whose connected pads least face the board outline (opt-in, OFF by default).

The seeder keeps the FIRST pose that fits, and its ladder starts at the part's
input angle -- on a pile that is a generator default, not a decision. Run 26's
regulator was seated with its three pins 0.40 mm from the north edge that way.
This flag lets every angle of the ladder find its own first fit and keeps the
pose `placement.edge_facing` scores lowest; a tie keeps #893's author order, and
with the flag off nothing runs (`state.rotation_prefer` is never set). It is
measured and REJECTED as a default by the facing-seed rows of
tests/test_placement_ab.py; this file pins the mechanism, not the verdict.

Fixture: a 30 x 20 board, U2 a three-pin row 2 mm ABOVE its origin (so at rot 0
the row is 1 mm from the north edge and at rot 180 it is 5 mm from it), one
partner per pin south of it, and nothing else in the way -- both angles fit.

Run:
    python3 tests/test_run26_rotate_by_facing.py
"""

import functools
import os
import random
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan as fp  # noqa: E402
from placement import seeder  # noqa: E402
import pose_score  # noqa: E402

RUN_ALL_FAST_OK = True

BOARD = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:SOT" (layer "F.Cu") (uuid "fp-u2") (at 6 3)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" smd rect (at -1.5 -2) (size 0.8 1.3) (layers "F.Cu") (net 1 "/A") (uuid "u2p1"))
   (pad "2" smd rect (at 0 -1.9) (size 0.9 1.5) (layers "F.Cu") (net 2 "/B") (uuid "u2p2"))
   (pad "3" smd rect (at 1.5 -2) (size 0.8 1.3) (layers "F.Cu") (net 3 "/C") (uuid "u2p3"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-c1") (at 4 12)
   (property "Reference" "C1" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "c1p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 0 "") (uuid "c1p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-c2") (at 6 12)
   (property "Reference" "C2" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/B") (uuid "c2p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 0 "") (uuid "c2p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-c3") (at 8 12)
   (property "Reference" "C3" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "c3p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 0 "") (uuid "c3p2"))
 )
)
'''


def _board(tmp):
    p = os.path.join(tmp, 'b.kicad_pcb')
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(BOARD)
    return p


def _state(path):
    pcb = parse_kicad_pcb(path)
    return pose_score.make_state(pcb, path, clearance=0.2,
                                 board_edge_clearance=0.3, grid_step=0.1), pcb


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory(prefix='t_rbf_') as tmp:
        path = _board(tmp)

        # --- 0: the rank itself, at the two angles that matter -------------
        st, _ = _state(path)
        r0 = seeder._facing_rank(st, 'U2', 6.0, 3.0, 0.0, set(), edge_refs=set())
        r180 = seeder._facing_rank(st, 'U2', 6.0, 3.0, 180.0, set(), edge_refs=set())
        check('rot 0 puts 3 pads on the edge-facing row; rot 180 puts 0',
              (r0, r180) == (3, 0), f'{(r0, r180)}')
        check('a declared edge connector ranks 0 at every angle',
              seeder._facing_rank(st, 'U2', 6.0, 3.0, 0.0, set(),
                                  edge_refs={'U2'}) == 0)

        # --- 1: _try_place keeps the FIRST fit; the preference reorders ----
        st_off, _ = _state(path)
        seeder._try_place(st_off, 'U2', 6.0, 3.0, set())
        st_on, _ = _state(path)
        st_on.rotation_prefer = functools.partial(seeder._facing_rank, st_on,
                                                  edge_refs=set())
        seeder._try_place(st_on, 'U2', 6.0, 3.0, set())
        check('off: the input angle (0) is kept, as it always was',
              st_off.parts['U2'].rot % 360 == 0.0, f'{st_off.parts["U2"].rot}')
        check('on: the inboard-facing angle (180) is seated instead',
              st_on.parts['U2'].rot % 360 == 180.0, f'{st_on.parts["U2"].rot}')
        check('the preference moved rotation only, not position',
              abs(st_on.parts['U2'].x - st_off.parts['U2'].x) < 1e-9
              and abs(st_on.parts['U2'].y - st_off.parts['U2'].y) < 1e-9)
        # A FAR target: the rings find nothing, so the whole-board sweep
        # must seat the part nearest the target -- with the flag off, exactly
        # where the seeder seated it before the tie-break existed (measured
        # on the pristine seeder: (26.3, 18.3) at rot 0, clearance 0.2). The
        # first form of the tie-break lost this sweep on the OFF path and
        # the review measured it on a corpus board (splitflap 0 -> 6
        # unseated); the 30 x 20 fixture never needed the sweep at the
        # target above, so nothing here had seen it.
        st_far, _ = _state(path)
        far = seeder._try_place(st_far, 'U2', 100.0, 100.0, set())
        check('off, far target: the whole-board sweep still seats the part',
              far is not None
              and (round(st_far.parts['U2'].x, 3), round(st_far.parts['U2'].y, 3),
                   st_far.parts['U2'].rot % 360) == (26.3, 18.3, 0.0),
              f'{far} {(st_far.parts["U2"].x, st_far.parts["U2"].y, st_far.parts["U2"].rot)}')
        st_far_on, _ = _state(path)
        st_far_on.rotation_prefer = functools.partial(seeder._facing_rank, st_far_on,
                                                      edge_refs=set())
        far_on = seeder._try_place(st_far_on, 'U2', 100.0, 100.0, set())
        check('on, far target: every angle sweeps too, and the part is seated',
              far_on is not None, f'{far_on}')

        # --- 2: end to end through seed_from_intent -----------------------
        # A zone along the north edge, so the seeder seats U2 where one angle
        # faces the edge and the other does not (mid-board, next to its
        # partners, every angle ties and the flag has nothing to choose).
        intent = fp.intent_from_dict({'schema': 1, 'kind': fp.KIND,
                                      'units': 'mm',
                                      'blocks': [{'name': 'ldo', 'refs': ['U2'],
                                                  'zone': [0.0, 0.5, 12.0, 7.0]}]})
        outs = {}
        for flag in (False, True):
            pcb = parse_kicad_pcb(path)
            res = seeder.seed_from_intent(pcb, path, intent, random.Random('26'),
                                          group_sources=('kicad', 'sheet'),
                                          clearance=0.2,
                                          board_edge_clearance=0.3,
                                          grid_step=0.1,
                                          rotate_by_facing=flag)
            outs[flag] = {p['reference']: p for p in res['placements']}
            if flag:
                check('the flag announces itself in the seed notes',
                      any('rotate-by-facing' in n for n in res['notes']),
                      f'{res["notes"]}')
        check('seed_from_intent reaches the tie-break: U2 turns only with the flag',
              'U2' in outs[True] and 'U2' in outs[False]
              and outs[True]['U2']['new_rotation'] % 360 != outs[False]['U2']['new_rotation'] % 360,
              f'{outs[False].get("U2")} vs {outs[True].get("U2")}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

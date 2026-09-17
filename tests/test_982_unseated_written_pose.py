#!/usr/bin/env python3
"""A pad conflict against a part the seed could NOT seat is not the seed's.

#982. An unseated part keeps the pose it came in with, and that pose is
written: `placements` has no row for it, so the writer leaves its block alone.
Every later stage passes the pile as `exclude` -- deliberately, since a pile at
meaningless input coordinates must not veto real poses -- so later stages pack
parts onto that copper, and the pair used to be charged to
`pad_conflicts_seeded` through the partner the seed did move. Measured on
ulx3s: seeding an edge connector 0.386 mm further inward for a board-edge
copper fix took the count 0 -> 1, on a pair between a mounting hole and an
unseated connector.

The copper is real: `py_tools/check_assembly.py` on that seed-1 output prints
`H4 <-> J1  pad_intersection  1.5089mm2  side  BLOCKING` and `VERDICT: NOT
BUILDABLE`. So this file pins that such a pair stays counted and NAMED -- in
`pad_conflicts_unseated` -- and that the three buckets partition
`pad_conflicts_after`. Not every pair in that bucket is that severe: ulx3s
seed 0's is a 0.191mm graze on a board the same tool grades buildable.

Fixture: a 30 x 20 board. J9 is declared on the north edge and is 36 mm wide,
wider than the board, so no pose is legal anywhere and it stays unseated at its
input pose in the middle of the board, where its centre pad sits inside the
zone the U parts are declared into. U2 is then packed onto it. The control
keeps J9 and shrinks it to 6mm, which the north edge can take: seated, it
leaves the middle of the board and no conflict exists. Same board, same intent,
same seed, one pad span apart -- so the arm above is a statement about J9
staying at its INPUT pose, not about U2's choice of pose (which is reported,
not asserted).

Run:
    python3 tests/test_982_unseated_written_pose.py
"""

import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

RUN_ALL_FAST_OK = True

SEED = os.path.join(ROOT, 'py_placer', 'place_seed.py')

#: The `(layers ...)` section is load-bearing: without one `copper_layers` is
#: empty, `grade_pad_legality` resolves no shared layer for any pair, and the
#: assertions below would pass on a grader that can see no conflict at all.
BOARD = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
%s (footprint "test:C" (layer "F.Cu") (uuid "fp-u2") (at 15 10)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" thru_hole circle (at -1 0) (size 1.8 1.8) (drill 1.0) (layers "*.Cu") (net 1 "/A") (uuid "u2p1"))
   (pad "2" thru_hole circle (at 1 0) (size 1.8 1.8) (drill 1.0) (layers "*.Cu") (net 2 "/B") (uuid "u2p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u3") (at 15 10)
   (property "Reference" "U3" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "u3p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 3 "/C") (uuid "u3p2"))
 )
)
'''

#: 36 mm between the outer pads on a 30 mm board: no pose is legal anywhere, so
#: this part is unseated in every stage and keeps its input `(at 15 10)`. The
#: CENTRE pad is what a part packed into the zone can land on.
WIDE = ''' (footprint "test:HDR" (layer "F.Cu") (uuid "fp-j9") (at 15 10)
   (property "Reference" "J9" (at 0 0 0))
   (pad "1" thru_hole circle (at -18 0) (size 1.8 1.8) (drill 1.0) (layers "*.Cu") (net 1 "/A") (uuid "j9p1"))
   (pad "2" thru_hole circle (at 0 0) (size 2.4 2.4) (drill 1.2) (layers "*.Cu") (net 2 "/B") (uuid "j9p2"))
   (pad "3" thru_hole circle (at 18 0) (size 1.8 1.8) (drill 1.0) (layers "*.Cu") (net 0 "") (uuid "j9p3"))
 )
'''

#: The control: the same part, 6 mm wide, which the north edge can take.
NARROW = WIDE.replace('(at -18 0)', '(at -3 0)').replace('(at 18 0)',
                                                         '(at 3 0)')

INTENT = {
    'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
    'edge_connectors': [{'ref': 'J9', 'edge': 'north',
                         'overhang_mm': {'min': 0.0, 'max': 0.0}}],
    'blocks': [{'name': 'ics', 'refs': ['U*'], 'zone': [12.0, 7.0, 18.0, 13.0],
                'note': 'a zone the size of the parts, over J9 centre pad'}],
}


def _run(tmp, name, extra=WIDE, seed='0'):
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % extra)
    ipath = os.path.join(tmp, f'{name}.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(INTENT, fh)
    out = os.path.join(tmp, f'{name}_out.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', SEED, board, out,
                        '--intent', ipath, '--seed', seed,
                        '--board-edge-clearance', '0.2', '--no-polish'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    text = (r.stdout or '') + (r.stderr or '')
    summ = None
    for ln in (r.stdout or '').splitlines():
        if ln.startswith('JSON_SUMMARY: '):
            summ = json.loads(ln[len('JSON_SUMMARY: '):])
    return r.returncode, summ, text, out


def _pairs(summ, key):
    return {frozenset(p[:2]) for p in (summ.get(key) or [])}


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # ---- the split itself, on hand-built pairs --------------------------
    # Every combination the caller can hand it, so the classification is
    # pinned apart from the seeder's behaviour on any one board.
    from place_seed import split_pad_pairs
    worst = [('A', 'B', 0.1),      # both moved            -> the seed's
             ('A', 'J9', 0.2),     # moved + unseated      -> unseated
             ('J9', 'A', 0.25),    # the other order       -> unseated
             ('A', 'LOCK', 0.3),   # moved + untouched     -> the seed's
             ('LOCK', 'OLD', 0.4),  # neither moved         -> the board's
             ('J9', 'J8', 0.5)]    # both unseated         -> the board's
    mine, uns = split_pad_pairs(worst, {'A', 'B'}, ['J9', 'J8'])
    check('a pair between two parts it moved is the seed\'s',
          ('A', 'B', 0.1) in mine, f'{mine}')
    check('a pair with an unseated part is not the seed\'s, either order',
          [w[:2] for w in uns] == [('A', 'J9'), ('J9', 'A')], f'{uns}')
    check('a moved part against an untouched one stays the seed\'s',
          ('A', 'LOCK', 0.3) in mine, f'{mine}')
    check('a pair with nothing it moved is in neither list',
          all(w[:2] not in [x[:2] for x in mine + uns]
              for w in (('LOCK', 'OLD', 0.4), ('J9', 'J8', 0.5))),
          f'{mine} {uns}')
    check('the two lists keep `worst` order',
          mine == [w for w in worst if w in mine]
          and uns == [w for w in worst if w in uns], f'{mine} {uns}')
    check('nothing unseated means nothing in the second list',
          split_pad_pairs(worst, {'A', 'B'}, [])[1] == [], 'with unseated=[]')

    with tempfile.TemporaryDirectory(prefix='t_982_') as tmp:
        # ---- 1: a part packed onto an unseated part's written pose -------
        rc, s, text, out = _run(tmp, 'onto')
        check('the wide part is unseated',
              s is not None and 'J9' in (s.get('unseated_refs') or []),
              f"{s and s.get('unseated_refs')}")
        check('a pair against it EXISTS on the written board',
              s is not None and (s.get('pad_conflicts_after') or 0) >= 1,
              f"after {s and s.get('pad_conflicts_after')}\n{text[-600:]}")
        check('...charged to the unseated bucket, not to the seed',
              s is not None and (s.get('pad_conflicts_unseated') or 0) >= 1
              and s.get('pad_conflicts_seeded') == 0,
              f"seeded {s and s.get('pad_conflicts_seeded')} unseated "
              f"{s and s.get('pad_conflicts_unseated')}")
        check('...with J9 in every pair of that bucket',
              s is not None and (s.get('pad_conflicts_unseated_pairs') or [])
              and all('J9' in p[:2]
                      for p in s['pad_conflicts_unseated_pairs']),
              f"{s and s.get('pad_conflicts_unseated_pairs')}")
        check('...the PAIR is named on the console, not just counted',
              'could NOT seat' in text and 'J9' in text, text[-700:])
        check('...and the console says it is not charged',
              'reported not charged' in text, text[-700:])
        check('the three buckets partition the total',
              s is not None
              and (s.get('pad_conflicts_seeded') or 0)
              + (s.get('pad_conflicts_unseated') or 0)
              + (s.get('pad_conflicts_inherited') or 0)
              == (s.get('pad_conflicts_after') or 0),
              f"{s and {k: s.get(k) for k in ('pad_conflicts_seeded', 'pad_conflicts_unseated', 'pad_conflicts_inherited', 'pad_conflicts_after')}}")
        check('the exit code still refuses the seed, for the unseated part',
              rc == 4 and 'does NOT satisfy its intent' in text,
              f'rc {rc}\n{text[-400:]}')

        # ---- 2: control -- the SAME part, seatable ------------------------
        # Identical board, identical intent, identical seed; only J9's pad
        # span changes, from wider than the board to 6 mm. It is then seated
        # on the north edge and leaves the middle of the board, and there is
        # no conflict at all. So arm 1's pair is J9 staying at its INPUT
        # pose, which is the claim -- not U2's choice of pose, and not the
        # fixture merely being crowded.
        rc2, s2, _t2, out2 = _run(tmp, 'seatable', extra=NARROW)
        check('the control seats the same part, so nothing is unseated',
              rc2 == 0 and not (s2.get('unseated_refs') or []),
              f"rc {rc2} {s2 and s2.get('unseated_refs')}\n{_t2[-500:]}")
        check('...and then there is no conflict at all',
              s2 is not None and (s2.get('pad_conflicts_after') or 0) == 0
              and (s2.get('pad_conflicts_unseated') or 0) == 0,
              f"after {s2 and s2.get('pad_conflicts_after')}")

        from kicad_parser import parse_kicad_pcb
        p1 = parse_kicad_pcb(out).footprints
        p2 = parse_kicad_pcb(out2).footprints
        check('the control moved J9 out of the middle of the board',
              abs(p2['J9'].y - 10.0) > 1.0, f"J9 at ({p2['J9'].x}, "
              f"{p2['J9'].y}) in the control")
        # Reported, not asserted: U2's own pose is free to differ between the
        # arms, and the claim above does not rest on it.
        print(f"  INFO: U2 at ({p1['U2'].x}, {p1['U2'].y}) with J9 unseated, "
              f"({p2['U2'].x}, {p2['U2'].y}) with J9 seated")

        # ---- 3: the unseated part was written where it came in ----------
        check('the unseated part is written at its input pose',
              abs(p1['J9'].x - 15.0) < 1e-6 and abs(p1['J9'].y - 10.0) < 1e-6,
              f"J9 at ({p1['J9'].x}, {p1['J9'].y})")

    print()
    if fails:
        print(f"FAIL: {len(fails)} check(s): " + '; '.join(fails))
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

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
zone the U parts are declared into. A U part is then packed onto it -- U3, as
the run works out, which is why the pair is read from the summary rather than
spelled here. The control
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

from kicad_parser import parse_kicad_pcb   # noqa: E402  (after the path setup)

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


def _run(tmp, name, extra=WIDE, seed='0', polish=False):
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % extra)
    ipath = os.path.join(tmp, f'{name}.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(INTENT, fh)
    out = os.path.join(tmp, f'{name}_out.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', SEED, board, out,
                        '--intent', ipath, '--seed', seed,
                        '--board-edge-clearance', '0.2']
                       + ([] if polish else ['--no-polish']),
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


def _independent_counts(tmp, name, out, summ):
    """(inherited, charged) re-derived from the WRITTEN board, not from the
    summary's own arithmetic.

    `pad_conflicts_inherited` is published as the residual
    `total - seeded - unseated`, so any check that adds the three up and
    compares with the total is true by construction. This re-grades the output
    at the same clearance, asks which refs moved off their input pose, and
    counts the pairs on each side of that line itself.
    """
    from placement.legality import grade_pad_legality
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    before = parse_kicad_pcb(board)
    after = parse_kicad_pcb(out)
    moved_refs = set()
    for ref, fp in (after.footprints or {}).items():
        was = (before.footprints or {}).get(ref)
        if was is None or (abs(fp.x - was.x) > 1e-6 or abs(fp.y - was.y) > 1e-6
                           or abs((fp.rotation - was.rotation) % 360.0) > 1e-6):
            moved_refs.add(ref)
    graded = grade_pad_legality(after, 0.25, edge_margin=0.2, pcb_file=out,
                                worst_n=0)
    worst = graded.get('worst') or ()
    charged = [w for w in worst if w[0] in moved_refs or w[1] in moved_refs]
    return len(worst) - len(charged), len(charged)


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
    # A ref in BOTH sets is charged to the seed -- the direction that keeps the
    # gate honest. The seeder keeps them disjoint, so this pins a tie-break
    # rather than a case in the wild.
    both = split_pad_pairs([('A', 'J9', 0.1)], {'A', 'J9'}, ['J9'])
    check('a ref that is both moved and unseated is charged to the seed',
          both == ([('A', 'J9', 0.1)], []), f'{both}')
    # A bare ref would be split by CHARACTER, and every pair would land in the
    # wrong bucket in silence.
    try:
        split_pad_pairs([('A', 'J9', 0.1)], {'A'}, 'J9')
        refused = False
    except TypeError:
        refused = True
    check('a bare ref is refused, not split by character', refused)

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
        # The pair must be named ON THAT LINE. Asserting it of the whole
        # output proves nothing: JSON_SUMMARY carries every ref, and the
        # "not charged" wording is also printed by the inherited line, which
        # this fixture happens to leave empty.
        line = ([ln for ln in text.splitlines() if 'could NOT seat' in ln]
                or [''])[0]
        pair = s and (s.get('pad_conflicts_unseated_pairs') or [[None, None]])[0]
        check('...the PAIR is named on that console line, not just counted',
              bool(line) and pair and all(r in line for r in pair[:2])
              and f"{pair[2]:.3f}" in line,
              f'line {line!r} pair {pair}')
        check('...and that same line says it is not charged',
              'reported not charged' in line, f'line {line!r}')
        # The partition cannot be checked against the summary alone: the
        # inherited count IS the residual `total - seeded - unseated`, so
        # summing the three and comparing with the total is an identity that
        # holds however wrongly the pairs were split (measured: a mutation
        # sending every pair to the unseated bucket keeps it true). Count the
        # board's own pairs INDEPENDENTLY, from the written board's grade.
        inherited, moved = _independent_counts(tmp, 'onto', out, s)
        # `inherited + moved` is this file's own count of the written board's
        # pairs: if it disagrees with the published total, the re-grade used
        # different terms than the run did and the two numbers below are not
        # comparable -- so that is part of the check, not an assumption.
        check('the buckets are the written board\'s own pairs, counted apart',
              s is not None
              and inherited + moved == s.get('pad_conflicts_after')
              and s.get('pad_conflicts_inherited') == inherited
              and s.get('pad_conflicts_seeded') + s.get('pad_conflicts_unseated')
              == moved,
              f"inherited {s and s.get('pad_conflicts_inherited')} vs {inherited}; "
              f"charged {s and s.get('pad_conflicts_seeded')}+"
              f"{s and s.get('pad_conflicts_unseated')} vs {moved}")
        check('...and they still partition the published total',
              s is not None
              and (s.get('pad_conflicts_seeded') or 0)
              + (s.get('pad_conflicts_unseated') or 0)
              + (s.get('pad_conflicts_inherited') or 0)
              == (s.get('pad_conflicts_after') or 0),
              f"{s and {k: s.get(k) for k in ('pad_conflicts_seeded', 'pad_conflicts_unseated', 'pad_conflicts_inherited', 'pad_conflicts_after')}}")
        check('the exit code still refuses the seed, for the unseated part',
              rc == 4 and 'does NOT satisfy its intent' in text,
              f'rc {rc}\n{text[-400:]}')

        # ---- 1b: the DEFAULT path, polish ON -------------------------------
        # Every other arm passes --no-polish, so without this one the file
        # pins only a path the tool is not normally run on. Two things are
        # asked of it. First the PREMISE the whole bucket rests on: an
        # unseated part is written at the pose it came in with. Nothing
        # enforces that -- quench takes `movable = [not locked]`, so an
        # unseated part is a polish candidate -- and here it holds, measured.
        # Second, nothing this seed placed is charged for J9.
        # On this fixture the polish clears the conflict entirely (it moves
        # the U parts off J9), so the bucket is EMPTY here: that is reported,
        # and the arm does not pretend to check a non-empty one.
        rc_p, s_p, text_p, out_p = _run(tmp, 'onto_polished', polish=True)
        polished = parse_kicad_pcb(out_p).footprints
        check('[polished] the unseated part is still written at its input pose',
              s_p is not None and 'J9' in (s_p.get('unseated_refs') or [])
              and abs(polished['J9'].x - 15.0) < 1e-6
              and abs(polished['J9'].y - 10.0) < 1e-6,
              f"J9 at ({polished['J9'].x}, {polished['J9'].y}), unseated "
              f"{s_p and s_p.get('unseated_refs')}")
        check('[polished] no pair against it is charged to the seed',
              s_p is not None and rc_p == 4
              and not [p for p in (s_p.get('pad_conflicts_seeded_pairs') or [])
                       if 'J9' in p[:2]],
              f"rc {rc_p} seeded_pairs {s_p and s_p.get('pad_conflicts_seeded_pairs')}")
        print(f"  INFO: polished, the conflict is "
              f"{'still there' if (s_p.get('pad_conflicts_unseated') or 0) else 'gone'}"
              f" ({s_p.get('pad_conflicts_unseated')} in the unseated bucket)")

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

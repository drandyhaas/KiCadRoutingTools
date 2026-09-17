#!/usr/bin/env python3
"""`place_seed`'s gate looks at the copper it just arranged.

The intent grade answers "does this satisfy the document it was built from",
and a pad short is not in that document: `legality_budget` carries `oob_count`
and, on an emitted intent, withholds `overlap_area`, while pad and hole
conflicts are not a budgeted channel at all. So a seed could drive one part's
pin through another's pad, grade clean against its own intent, and be RANKED
clean by `compare_seeds` -- and `check_assembly` would then call the board NOT
BUILDABLE for a `pad_intersection` nobody upstream had looked for. Measured on
esp_prog (run 27): ten seeds of ten, all passing their gate, all unbuildable.

Attribution, so the seed answers for its own work and not the board's: a PAD
pair is the seed's when either member is a part it placed, by ref. A hole
conflict cannot be attributed that way (`grade_pad_legality` counts holes
without recording the pair), so it is judged on the delta against the input.

Fixture: a 30 x 20 board whose south edge is spanned by ONE locked pad, and a
3-pin header declared on that edge. No seat on the band clears the locked
part, so the seeder keeps the declared edge and leaves the conflict for this
gate -- which is exactly the hand-off the stage-1 slide documents, and it is
why this fixture keeps working once that slide exists.

Run:
    python3 tests/test_run27_seed_gate_shorts.py
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
#: empty, `grade_pad_legality` resolves no shared layer for any pair, and
#: every assertion below passes on a grader that can see no conflict at all.
BOARD = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:FIX" (layer "F.Cu") (uuid "fp-fix") (at %s) (locked yes)
   (property "Reference" "FIX1" (at 0 0 0))
   (pad "1" thru_hole circle (at 0 0) (size %s 2.2) (drill 1.0) (layers "*.Cu") (net 3 "/C") (uuid "fixp1"))
 )
 (footprint "test:HDR" (layer "F.Cu") (uuid "fp-j1") (at 15 10)
   (property "Reference" "J1" (at 0 0 0))
   (pad "1" thru_hole circle (at -2.54 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 1 "/A") (uuid "j1p1"))
   (pad "2" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 2 "/B") (uuid "j1p2"))
   (pad "3" thru_hole circle (at 2.54 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 0 "") (uuid "j1p3"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u2") (at 15 10)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "u2p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (uuid "u2p2"))
 )
%s)
'''

#: Two locked parts already shorting each other, which the seed never touches.
INHERITED = ''' (footprint "test:FIX" (layer "F.Cu") (uuid "fp-a") (at 4 4) (locked yes)
   (property "Reference" "OLD1" (at 0 0 0))
   (pad "1" thru_hole circle (at 0 0) (size 2.2 2.2) (drill 1.0) (layers "*.Cu") (net 1 "/A") (uuid "oldp1"))
 )
 (footprint "test:FIX" (layer "F.Cu") (uuid "fp-b") (at 4.6 4) (locked yes)
   (property "Reference" "OLD2" (at 0 0 0))
   (pad "1" thru_hole circle (at 0 0) (size 2.2 2.2) (drill 1.0) (layers "*.Cu") (net 2 "/B") (uuid "oldp2"))
 )
'''

INTENT = {
    'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
    'edge_connectors': [{'ref': 'J1', 'edge': 'south',
                         'class': 'edge_receptacle',
                         'overhang_mm': {'min': 0.0, 'max': 0.0},
                         'along_edge_band': {'from': 0.1, 'to': 0.9}}],
    'blocks': [{'name': 'ics', 'refs': ['U*'], 'zone': [8.0, 6.0, 22.0, 14.0],
                'note': 'the pile'}],
}


def _run(tmp, name, fix_w='2.2', fix_at='15 18.4', extra=''):
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % (fix_at, fix_w, extra))
    ipath = os.path.join(tmp, f'{name}.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(INTENT, fh)
    out = os.path.join(tmp, f'{name}_out.kicad_pcb')
    # `--board-edge-clearance 0.2`: the header is declared with no
    # overhang, so at the 0.55mm default its courtyard seats 0.55mm inboard
    # and `rule_edge_connector`'s 0.50mm seat tolerance fires -- a fixture
    # artefact that has nothing to do with the channel under test.
    r = subprocess.run([sys.executable, '-X', 'utf8', SEED, board, out,
                        '--intent', ipath, '--seed', '0',
                        '--board-edge-clearance', '0.2'],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    text = (r.stdout or '') + (r.stderr or '')
    summ = None
    for ln in (r.stdout or '').splitlines():
        if ln.startswith('JSON_SUMMARY: '):
            summ = json.loads(ln[len('JSON_SUMMARY: '):])
    return r.returncode, summ, text


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory(prefix='t_sgs_') as tmp:
        # --- 1: a short among the parts the seed placed FAILS the gate ------
        rc, s, text = _run(tmp, 'shorted', fix_w='26')
        check('a pad conflict the seed left behind fails the gate',
              rc == 4, f'rc {rc}\n{text[-700:]}')
        check('...counted as the seed\'s own',
              s is not None and (s.get('pad_conflicts_seeded') or 0) >= 1,
              f'{s and {k: s.get(k) for k in ("pad_conflicts_seeded", "pad_conflicts_inherited")}}')
        check('...the PAIR is named, not just counted',
              'J1' in text and 'FIX1' in text and 'pad conflict' in text,
              text[-500:])
        check('...and the pairs ride in the summary for a caller to read',
              s is not None and any(
                  set(p[:2]) == {'J1', 'FIX1'}
                  for p in (s.get('pad_conflicts_seeded_pairs') or [])),
              f"{s and s.get('pad_conflicts_seeded_pairs')}")
        check('...with the intent grade itself still clean, so the two '
              'channels are distinguishable',
              s is not None and s.get('grade_errors') == 0
              and 'closer than their clearance' in text,
              f"grade_errors {s and s.get('grade_errors')}")

        # --- 2: a clean seed still passes -----------------------------------
        # FIX1 well inboard of the south band, so the seat is clear on
        # any seeder: this arm must not depend on the stage-1 slide.
        rc2, s2, _t2 = _run(tmp, 'clean', fix_at='15 4')
        check('a seed that leaves no short passes',
              rc2 == 0 and s2 is not None
              and s2.get('pad_conflicts_seeded') == 0,
              f"rc {rc2} {s2 and s2.get('pad_conflicts_seeded')}")

        # --- 3: the board's OWN short is reported, never charged ------------
        # OLD1/OLD2 are locked and overlapping before the seed runs. The seed
        # places neither, so the pair is the board's: named on the console,
        # counted apart, and the exit code does not move. Charging it would
        # make every seed of a board with one pre-existing short unrankable.
        rc3, s3, text3 = _run(tmp, 'inherited', fix_at='15 4',
                              extra=INHERITED)
        check('a short between parts the seed never placed is not charged',
              rc3 == 0, f'rc {rc3}\n{text3[-700:]}')
        check('...but it IS counted and reported',
              s3 is not None and (s3.get('pad_conflicts_inherited') or 0) >= 1
              and 'did not place' in text3,
              f"{s3 and {k: s3.get(k) for k in ('pad_conflicts_seeded', 'pad_conflicts_inherited')}}")
        check('...and none of it lands in the seeded count',
              s3 is not None and s3.get('pad_conflicts_seeded') == 0,
              f"{s3 and s3.get('pad_conflicts_seeded')}")
        # The two counts PARTITION the board's shorts. Stated here because
        # the inherited count is derived from `pad_conflicts` rather than
        # from `len(worst)`, which is capped: on this fixture the cap is
        # never reached, so a mutation restoring the default cap SURVIVES
        # this file (measured) and the arithmetic below is what keeps the
        # totals honest if it ever does.
        for _tag, _s in (('shorted', s), ('inherited', s3), ('clean', s2)):
            if _s is None:
                continue
            check(f'[{_tag}] seeded + inherited accounts for every short',
                  (_s.get('pad_conflicts_seeded') or 0)
                  + (_s.get('pad_conflicts_inherited') or 0)
                  == (_s.get('pad_conflicts_after')
                      if _s.get('pad_conflicts_after') is not None
                      else (_s.get('pad_conflicts_seeded') or 0)
                      + (_s.get('pad_conflicts_inherited') or 0)),
                  f'{_s.get("pad_conflicts_seeded")} + '
                  f'{_s.get("pad_conflicts_inherited")}')

    # ---- the stderr line names the channel that actually fired -----------
    # Two failures reach exit 4 and they are not the same failure. The
    # hole-only arm is the one that had no way to be seen from a fixture:
    # `grade_pad_legality` counts holes without recording the pair, so a
    # hole-only refusal cannot print pairs -- and the line used to say
    # "pads ... see the pairs above" anyway, naming the wrong channel and
    # pointing at output that is not there. `gate_reason` is pure, so the
    # four arms are gradable without a board that can produce each one.
    sys.path.insert(0, os.path.join(ROOT, 'py_placer'))
    import place_seed as _ps
    _g = _ps.gate_reason
    check('nothing fired -> no refusal line at all', _g([], [], [], 0) is None,
          repr(_g([], [], [], 0)))
    _miss = _g(['U1'], [], [], 0) or ''
    check('a seed that misses its intent says so',
          'does NOT satisfy its intent' in _miss, _miss)
    _pad = _g([], [], [('A', 'B', 0.1)], 0) or ''
    check('a pad-only refusal names pads and points at the pairs',
          'satisfies its intent' in _pad and 'pads closer' in _pad
          and 'named above' in _pad and 'hole' not in _pad, _pad)
    _hole = _g([], [], [], 3) or ''
    check('a HOLE-only refusal names holes, not pads, and promises no pairs',
          'hole conflict' in _hole and '3' in _hole
          and 'pads closer' not in _hole and 'pairs' not in _hole, _hole)
    _both = _g([], [], [('A', 'B', 0.1)], 2) or ''
    check('both channels are named when both fired',
          'pads closer' in _both and 'hole conflict' in _both, _both)

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

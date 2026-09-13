#!/usr/bin/env python3
"""`place_seed`'s gate grades the seed's OWN work: a grade error on a part the
seed was told not to move is reported and set aside, not exit 4.

Run 27 replayed the run-26 placement half from a hand-authored zone plan and
every one of ten seeds failed its gate on the same fixed USB socket -- declared
`along_edge: center` within 0.6 mm, sitting 1.75 mm off centre by the
mechanical declaration the run was given. A contradiction between the brief
and the board, which no seed can resolve and which failed all of them, so
`compare_seeds` had nothing to rank -- exactly run 26's shape (0 of 10
passing) with a different clause behind it.

Fixture: a 30 x 20 board; J1, a two-pad `edge_receptacle` declared on the west
edge, `(locked yes)` in the file 3 mm inboard of it (its seat conjunct fires,
courtyard basis, no drawn body); U1..U3 stacked at the board centre, the pile
the seed places, with a zone that holds them.

Run:
    python3 tests/test_run27_seed_gate_pinned.py
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

BOARD = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:RCPT" (layer "F.Cu") (uuid "fp-j1") (at 3 10)%s
   (property "Reference" "J1" (at 0 0 0))
   (pad "1" smd rect (at 0 -1) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "j1p1"))
   (pad "2" smd rect (at 0 1) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (uuid "j1p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u1") (at 15 10)
   (property "Reference" "U1" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "u1p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "u1p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u2") (at 15 10)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "/B") (uuid "u2p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "u2p2"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u3") (at 15 10)
   (property "Reference" "U3" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 3 "/C") (uuid "u3p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.5 0.5) (layers "F.Cu") (net 1 "/A") (uuid "u3p2"))
 )
)
'''


def _intent(zone, must_lock=()):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'must_lock': list(must_lock),
            'edge_connectors': [{'ref': 'J1', 'edge': 'west',
                                 'class': 'edge_receptacle',
                                 'overhang_mm': {'min': 0.0, 'max': 0.5}}],
            'blocks': [{'name': 'ics', 'refs': ['U*'], 'zone': zone,
                        'note': 'the pile'}]}


def _run(tmp, name, locked, intent):
    board = os.path.join(tmp, f'{name}.kicad_pcb')
    with open(board, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % (' (locked yes)' if locked else ''))
    ipath = os.path.join(tmp, f'{name}.json')
    with open(ipath, 'w', encoding='utf-8') as fh:
        json.dump(intent, fh)
    out = os.path.join(tmp, f'{name}_out.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', SEED, board, out,
                        '--intent', ipath, '--seed', '0'],
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

    with tempfile.TemporaryDirectory(prefix='t_sgp_') as tmp:
        # --- 1: the pinned arm: J1 locked in the file, must_lock too --------
        rc, s, text = _run(tmp, 'pinned', True,
                           _intent([8, 5, 22, 15], must_lock=['J1']))
        check('a locked receptacle off its seat does not fail the gate',
              rc == 0, f'rc {rc}\n{text[-900:]}')
        check('...its errors are counted apart, as pinned',
              s is not None and s.get('grade_errors') == 0
              and (s.get('grade_errors_pinned') or 0) >= 1,
              f'{s and {k: s.get(k) for k in ("grade_errors", "grade_errors_pinned")}}')
        check('...and NAMED on the console, with the rule',
              'J1: edge_connector' in text and 'GRADE ERROR (pinned)' in text
              and 'set aside' in text, text[-600:])
        # --- 2: the file lock alone pins (must_lock not declared) ----------
        rc2, s2, _ = _run(tmp, 'filelock', True, _intent([8, 5, 22, 15]))
        check('(locked yes) in the file pins on its own',
              rc2 == 0 and s2 is not None and s2.get('grade_errors') == 0
              and (s2.get('grade_errors_pinned') or 0) >= 1,
              f'rc {rc2} {s2 and {k: s2.get(k) for k in ("grade_errors", "grade_errors_pinned")}}')
        # --- 3: the gate still bites on the seed's OWN work ----------------
        # A zone too small for three parts: the seed cannot satisfy it, and
        # that IS the seeder's failure -- exit 4 as before, the pinned count
        # still reported beside it.
        rc3, s3, text3 = _run(tmp, 'own', True,
                              _intent([14.5, 9.5, 15.5, 10.5], must_lock=['J1']))
        check('an error on a part the seed placed still fails the gate',
              rc3 == 4 and s3 is not None
              and ((s3.get('grade_errors') or 0) >= 1
                   or (s3.get('unseated') or 0) >= 1),
              f'rc {rc3} {s3 and {k: s3.get(k) for k in ("grade_errors", "grade_errors_pinned", "unseated")}}')
        check('...with the pinned errors still counted beside it',
              s3 is not None and (s3.get('grade_errors_pinned') or 0) >= 1)
        # --- 4: unlocked, the receptacle is the seed's to seat -------------
        # Nothing pinned, nothing set aside: the seeder's edge stage seats J1
        # on its band and the gate reads its own work.
        rc4, s4, _ = _run(tmp, 'free', False, _intent([8, 5, 22, 15]))
        check('an unlocked receptacle is seated by the seed and pins nothing',
              s4 is not None and s4.get('grade_errors_pinned') == 0,
              f'rc {rc4} {s4 and {k: s4.get(k) for k in ("grade_errors", "grade_errors_pinned")}}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1370: check_complete's copper-to-hole grade of a LOWERED floor.

Two defects, both of which let a board that breaks its authored floor read
`measured_kept` / `reference_too`:

  * `kicad_hole_clearance` re-poured every zone at the authored value
    (`--refill-zones`), so a delivered fill that sits 0.1505 mm from a hole
    graded clean at 0.25. The fill is graded AS SHIPPED now.
  * `reference_too` compared only the two boards' minimum distances, so a
    reference that is an unplaced pile (pads on holes at 0.0) excused every
    track-to-via violation of a board it shares no copper with. The reference
    now excuses a violation only when it breaks the same KIND of pair at
    least as badly.

    python3 tests/test_1370_hole_floor_shipped_fill.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import check_complete as CC                                    # noqa: E402
from copy_board import copy_board                              # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(tmp, name, floor):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(SRC, dst)
    pro = dst[:-len('.kicad_pcb')] + '.kicad_pro'
    doc = {}
    if os.path.isfile(pro):
        with open(pro, encoding='utf-8') as fh:
            doc = json.load(fh)
    doc.setdefault('board', {}).setdefault('design_settings', {})['rules'] = {
        'min_hole_clearance': floor}
    with open(pro, 'w', encoding='utf-8') as fh:
        json.dump(doc, fh)
    return dst


def pairs(n, actual, kinds):
    return [{'kinds': kinds, 'actual': actual} for _ in range(n)]


def main():
    tmp = tempfile.mkdtemp(prefix='t1370_')
    try:
        # 1. kicad-cli is run on the fill as shipped, and the report's items
        #    are classed by kind.
        calls = []
        report = {'violations': [
            {'type': 'hole_clearance', 'severity': 'error',
             'description': 'Hole clearance violation (board setup constraints '
                            'hole clearance 0.2500 mm; actual 0.1505 mm)',
             'items': [{'description': 'Zone [GND] on F.Cu', 'pos': {'x': 1, 'y': 2}},
                       {'description': 'NPTH pad 1 [] of H1',
                        'pos': {'x': 1, 'y': 2}}]},
            {'type': 'hole_clearance', 'severity': 'error',
             'description': 'Hole clearance violation (board setup constraints '
                            'hole clearance 0.2500 mm; actual 0.2000 mm)',
             'items': [{'description': 'Track [N] on F.Cu, length 0.5375 mm',
                        'pos': {'x': 1, 'y': 2}},
                       {'description': 'Via [M] on F.Cu - B.Cu',
                        'pos': {'x': 3, 'y': 4}}]}]}

        def fake_run(argv, **kw):
            calls.append(list(argv))
            out = argv[argv.index('--output') + 1]
            with open(out, 'w', encoding='utf-8') as fh:
                json.dump(report, fh)
            return subprocess.CompletedProcess(argv, 0, '', '')
        import kicad_oracle
        real_find, real_run = kicad_oracle.find_kicad_cli, CC.subprocess.run
        kicad_oracle.find_kicad_cli = lambda warn=True: 'fake-kicad-cli'
        CC.subprocess.run = fake_run
        try:
            g = CC.kicad_hole_clearance(SRC, 0.25)
        finally:
            kicad_oracle.find_kicad_cli = real_find
            CC.subprocess.run = real_run
        check('kicad-cli ran once', len(calls) == 1, str(calls))
        check('the shipped fill is graded: no --refill-zones',
              bool(calls) and '--refill-zones' not in calls[0],
              ' '.join(calls[0]) if calls else '')
        kinds = sorted(i['kinds'] for i in g.get('items', ()))
        check('violations are classed by the kinds of their two items',
              kinds == [('NPTH pad', 'Zone'), ('Track', 'Via')], str(kinds))
        check('count and minimum come through',
              g.get('count') == 2 and g.get('min_actual') == 0.1505, str(g)[:200])

        # 2. the reference excuses only the kinds it breaks, at least as badly
        a = stage(tmp, 'a.kicad_pcb', 0.25)
        b = stage(tmp, 'b.kicad_pcb', 0.1)

        def grader(board_grade, ref_grade):
            return lambda p, v: ref_grade if p == a else board_grade
        board = {'count': 5, 'min_actual': 0.15,
                 'items': pairs(5, 0.15, ('Track', 'Via'))}
        pile = {'count': 4, 'min_actual': 0.0,
                'items': pairs(4, 0.0, ('NPTH pad', 'Pad'))}
        r = CC.fab_floor_integrity(b, a, hole_grader=grader(board, pile))
        rl = r['relaxed'][0] if r['relaxed'] else {}
        check('a pile with pads on holes at 0.0 does not excuse track-via '
              'violations: relaxed',
              rl.get('key') == 'min_hole_clearance' and not r['reference_too']
              and rl.get('not_in_reference') == 5, json.dumps(r)[:400])
        same = {'count': 9, 'min_actual': 0.12,
                'items': pairs(9, 0.12, ('Track', 'Via'))}
        r = CC.fab_floor_integrity(b, a, hole_grader=grader(board, same))
        check('the reference breaks the same kind worse: reference_too',
              [x['key'] for x in r['reference_too']] == ['min_hole_clearance']
              and not r['relaxed'], json.dumps(r)[:300])
        mixed = {'count': 2, 'min_actual': 0.1,
                 'items': pairs(2, 0.1, ('Track', 'Via')) +
                          pairs(1, 0.1, ('Zone', 'Via'))}
        board2 = {'count': 3, 'min_actual': 0.15,
                  'items': pairs(2, 0.15, ('Track', 'Via')) +
                           pairs(1, 0.15, ('Pad', 'Via'))}
        r = CC.fab_floor_integrity(b, a, hole_grader=grader(board2, mixed))
        check('one kind the reference does not break is enough: relaxed',
              [x['key'] for x in r['relaxed']] == ['min_hole_clearance']
              and r['relaxed'][0].get('not_in_reference') == 1, json.dumps(r)[:300])
        nameless = {'count': 5, 'min_actual': 0.15}
        r = CC.fab_floor_integrity(b, a, hole_grader=grader(nameless, same))
        check('a grade that cannot name its pairs cannot be compared: relaxed',
              [x['key'] for x in r['relaxed']] == ['min_hole_clearance'],
              json.dumps(r)[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

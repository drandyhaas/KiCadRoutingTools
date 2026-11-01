#!/usr/bin/env python3
"""#1580: check_complete --authored-from says how far the copper is from the
project the user AUTHORED, per violation, not per key.

Before: the authored grade compared one smallest object per size floor (a row
with no count) and left out a class clearance the chain lowered on request,
so a board KiCad grades with ~170 clearance errors under its authored project
read two count-less rows and nothing about clearance.

Gated here, on a two-net board whose shipped project is looser than its
authored one (Default clearance 0.3 -> 0.2, via 0.5 -> 0.4, hole 0.3 -> 0.2,
no class_clearance_relaxed record -- the chain's own ceiling):

  1. `count_below_floors` counts the objects `scan_board_minima` reads, and
     the minima did not move.
  2. A relaxed size row carries a count (`violations`), and the reason says it.
  3. The staged authored grade carries the AUTHORED .kicad_pro and .kicad_dru
     (and no .kicad_dru when the authored project has none), on the fill as
     shipped, at the project's own error severities.
  4. The disclosure never moves the verdict: the same exit and verdict
     whether the grade finds hundreds of errors or could not run -- and a
     grade that could not run says so rather than reading 0.
  5. With kicad-cli installed, the real CLI reports the clearance errors the
     authored class implies, which the shipped project has none of.

    python3 tests/test_1580_authored_project_drc.py
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
sys.path.insert(0, TESTS_DIR)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import check_complete as CC                                    # noqa: E402
from fix_kicad_drc_settings import (count_below_floors,        # noqa: E402
                                    scan_board_minima)
from run_utils import evidence                                 # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


W = 0.2


def netclass(clr):
    return {'name': 'Default', 'clearance': clr, 'track_width': W,
            'via_diameter': 0.6, 'via_drill': 0.3, 'microvia_diameter': 0.3,
            'microvia_drill': 0.1, 'diff_pair_width': 0.2, 'diff_pair_gap': 0.25,
            'diff_pair_via_gap': 0.25, 'priority': 2147483647, 'wire_width': 6,
            'bus_width': 12, 'line_style': 0,
            'schematic_color': 'rgba(0,0,0,0)', 'pcb_color': 'rgba(0,0,0,0)'}


def project(path, clr, via, hole):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
                       'min_clearance': 0.1, 'min_track_width': 0.1,
                       'min_via_diameter': via, 'min_via_annular_width': 0.1,
                       'min_through_hole_diameter': hole,
                       'min_hole_clearance': 0.0,
                       'min_copper_edge_clearance': 0.0}}},
                   'meta': {'filename': os.path.basename(path), 'version': 3},
                   'net_settings': {'classes': [netclass(clr)],
                                    'meta': {'version': 3}}}, fh, indent=1)


def board(path, clr, via, hole, vsize=0.45, vdrill=0.2):
    """Net A: a track between two vias (0.45/0.2 by default). Net B: a
    parallel track 0.25 mm from it -- legal at 0.2, a clearance error at 0.3."""
    yb = 10 + W + 0.25
    lines = [
        '(kicad_pcb (version 20241229) (generator "t") (generator_version "9.0")',
        ' (general (thickness 1.6)) (paper "A4")',
        ' (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))',
        ' (setup (pad_to_mask_clearance 0))',
        ' (net 0 "") (net 1 "A") (net 2 "B")',
        ' (gr_rect (start 0 0) (end 40 20) (stroke (width 0.1) (type default))'
        ' (fill none) (layer "Edge.Cuts") (uuid "00000000-0000-0000-0001-000000000001"))',
        ' (segment (start 5 10) (end 35 10) (width 0.2) (layer "F.Cu") (net 1)'
        ' (uuid "00000000-0000-0000-0000-000000000001"))',
        f' (segment (start 10 {yb:.4f}) (end 30 {yb:.4f}) (width 0.2) (layer "F.Cu")'
        ' (net 2) (uuid "00000000-0000-0000-0000-000000000002"))',
        f' (via (at 5 10) (size {vsize}) (drill {vdrill}) (layers "F.Cu" "B.Cu")'
        ' (net 1) (uuid "00000000-0000-0000-0000-000000000003"))',
        f' (via (at 35 10) (size {vsize}) (drill {vdrill}) (layers "F.Cu" "B.Cu")'
        ' (net 1) (uuid "00000000-0000-0000-0000-000000000004"))',
        ')']
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + '\n')
    project(path[:-len('.kicad_pcb')] + '.kicad_pro', clr, via, hole)
    return path


def main():
    tmp = tempfile.mkdtemp(prefix='t1580_')
    try:
        shipped = board(os.path.join(tmp, 'shipped.kicad_pcb'), 0.2, 0.4, 0.2)
        # The authored board's own vias meet its floors, so a breach is this
        # board's and not the reference's (#1198).
        authored = board(os.path.join(tmp, 'authored.kicad_pcb'), 0.3, 0.5, 0.3,
                         vsize=0.6, vdrill=0.3)

        print('1. counts read the objects the minima read')
        mins = scan_board_minima(shipped)
        check('minima unchanged by the refactor',
              mins.get('min_via_diameter') == 0.45
              and mins.get('min_via_drill') == 0.2
              and mins.get('min_through_hole_diameter') == 0.2
              and mins.get('min_track_width') == 0.2
              and abs(mins.get('min_via_annular_width', 0) - 0.125) < 1e-9,
              str(mins))
        n = count_below_floors(shipped, {'min_via_diameter': 0.5,
                                         'min_through_hole_diameter': 0.3,
                                         'min_track_width': 0.2,
                                         'min_via_annular_width': 0.1})
        check('2 vias below 0.5, 2 holes below 0.3, no track below its own '
              'width, no ring below 0.1',
              n == {'min_via_diameter': 2, 'min_through_hole_diameter': 2,
                    'min_track_width': 0, 'min_via_annular_width': 0}, str(n))

        print('2. a relaxed size row carries a count')
        fl = CC.fab_floor_integrity(shipped, authored,
                                    hole_grader=lambda b, v: {'count': 0})
        rows = {r['key']: r for r in fl.get('relaxed', ())}
        check('via diameter and hole diameter relaxed, each with its count',
              rows.get('min_via_diameter', {}).get('violations') == 2
              and rows.get('min_through_hole_diameter', {}).get('violations') == 2,
              json.dumps(fl.get('relaxed'))[:400])
        check('class clearance lowered without a descent record is not a '
              'relaxed row (#1160: the decision stands)',
              not any('clearance' in k for k in rows), str(sorted(rows)))

        print('3. the staged authored grade carries the authored rules')
        with open(authored[:-len('.kicad_pcb')] + '.kicad_dru', 'w') as fh:
            fh.write('(version 1)\n(rule authored_rule (constraint clearance (min 0.4mm)))\n')
        with open(shipped[:-len('.kicad_pcb')] + '.kicad_dru', 'w') as fh:
            fh.write('(version 1)\n(rule shipped_rule (constraint clearance (min 0.1mm)))\n')
        seen = []

        def fake_run(argv, **kw):
            dst = argv[-1]
            base = dst[:-len('.kicad_pcb')]
            pro = json.load(open(base + '.kicad_pro'))
            dru = (open(base + '.kicad_dru').read()
                   if os.path.isfile(base + '.kicad_dru') else None)
            seen.append({'argv': list(argv), 'pro': pro, 'dru': dru})
            with open(argv[argv.index('--output') + 1], 'w') as fh:
                json.dump({'violations': [
                    {'type': 'clearance', 'severity': 'error'},
                    {'type': 'clearance', 'severity': 'error'},
                    {'type': 'via_diameter', 'severity': 'error'}]}, fh)
            return subprocess.CompletedProcess(argv, 0, '', '')
        import kicad_oracle
        real_find, real_run = kicad_oracle.find_kicad_cli, CC.subprocess.run
        kicad_oracle.find_kicad_cli = lambda warn=True: 'fake-kicad-cli'
        CC.subprocess.run = fake_run
        try:
            d = CC.authored_project_drc(shipped, authored)
            os.remove(authored[:-len('.kicad_pcb')] + '.kicad_dru')
            seen_before = len(seen)
            CC.authored_project_drc(shipped, authored)
        finally:
            kicad_oracle.find_kicad_cli = real_find
            CC.subprocess.run = real_run
        cls = lambda s: s['pro']['net_settings']['classes'][0]['clearance']  # noqa: E731
        check('two kicad-cli runs: the authored project, then the shipped one',
              len(seen) == 4 and cls(seen[0]) == 0.3 and cls(seen[1]) == 0.2,
              str([cls(s) for s in seen]))
        check("the authored run carries the authored .kicad_dru, the shipped "
              "run the board's own",
              'authored_rule' in (seen[0]['dru'] or '')
              and 'shipped_rule' in (seen[1]['dru'] or ''),
              str([s['dru'] for s in seen[:2]]))
        check('an authored project with no .kicad_dru is graded with none',
              seen[seen_before]['dru'] is None, str(seen[seen_before]['dru']))
        check('the fill as shipped (no --refill-zones), at the project\'s own '
              'error severities',
              all('--refill-zones' not in s['argv'] and '--severity-error' in s['argv']
                  for s in seen), str(seen[0]['argv']))
        check('per-type counts, both arms',
              d.get('ran') and d['by_type'] == {
                  'clearance': {'authored': 2, 'shipped': 2},
                  'via_diameter': {'authored': 1, 'shipped': 1}}
              and d['authored']['count'] == 3, json.dumps(d)[:300])

        print('4. the disclosure never moves the verdict')
        real_apd = CC.authored_project_drc
        results = {}
        for name, fake in (
                ('many errors', lambda b, a, timeout=0: {
                    'ran': True, 'authored': {'count': 900, 'by_type': {'clearance': 900}},
                    'shipped': {'count': 0, 'by_type': {}},
                    'by_type': {'clearance': {'authored': 900, 'shipped': 0}}}),
                ('no kicad-cli', lambda b, a, timeout=0: {
                    'ran': False, 'reason': 'kicad-cli not found (set KICAD_CLI)'})):
            CC.authored_project_drc = fake
            js = os.path.join(tmp, f'cc_{len(results)}.json')
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    code = CC.main([shipped, '--authored-from', authored,
                                    '--json', js])
            finally:
                CC.authored_project_drc = real_apd
            doc = json.load(open(evidence(js, 'check_complete --json')))
            results[name] = (code, doc.get('verdict'), doc.get('reason') or '')
        (c1, v1, r1), (c2, v2, r2) = results['many errors'], results['no kicad-cli']
        check('same exit and verdict either way (UNSOUND on the size floors)',
              c1 == c2 == CC.UNSOUND and v1 == v2 == 'UNSOUND', f'{results}'[:400])
        check('the reason names the count per type, disclosed',
              'DISCLOSED, not counted: KiCad DRC of this copper' in r1
              and 'clearance 900 (shipped 0)' in r1, r1[-400:])
        check('a grade that did not run says so: no count, not 0',
              'NOT RUN (kicad-cli not found' in r2 and 'not a count of 0' in r2,
              r2[-400:])
        check('the size rows are counted in the reason',
              'via diameter 0.5 -> 0.45 (2 vias below it)' in r1, r1[:400])

        print('5. the real grade, through kicad-cli')
        from kicad_oracle import find_kicad_cli
        if find_kicad_cli(warn=False):
            d = CC.authored_project_drc(shipped, authored)
            bt = d.get('by_type') or {}
            check('KiCad: the 0.25 mm gap is a clearance error under the '
                  'authored 0.3 class and none under the shipped 0.2',
                  d.get('ran') and (bt.get('clearance') or {}).get('authored', 0) >= 1
                  and (bt.get('clearance') or {}).get('shipped') == 0,
                  json.dumps(d)[:500])
            check('KiCad: both vias break the authored via and hole floors',
                  (bt.get('via_diameter') or {}).get('authored') == 2
                  and (bt.get('drill_out_of_range') or {}).get('authored') == 2,
                  json.dumps(bt))
        else:
            print('  [skip] kicad-cli not installed: the real-KiCad grade (5)')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

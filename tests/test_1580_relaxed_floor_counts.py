#!/usr/bin/env python3
"""#1580: check_complete --authored-from counts the objects below each relaxed
floor, not only the smallest one.

Before: the authored grade compared one smallest object per size floor, a row
with no count, so `via diameter 0.5 -> 0.45` read the same for one via as for
thirty-six.

Gated here, on a two-net board whose shipped project is looser than its
authored one (Default clearance 0.3 -> 0.2, via 0.5 -> 0.4, hole 0.3 -> 0.2,
no class_clearance_relaxed record -- the chain's own ceiling):

  1. `count_below_floors` counts the objects `scan_board_minima` reads, and
     the minima did not move.
  2. A relaxed size row carries a count (`violations`).
  3. The reason names each count, and the verdict is unchanged.

    python3 tests/test_1580_relaxed_floor_counts.py
"""
import contextlib
import io
import json
import os
import shutil
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

        print('3. the reason counts the relaxed size rows')
        js = os.path.join(tmp, 'cc.json')
        with contextlib.redirect_stdout(io.StringIO()):
            code = CC.main([shipped, '--authored-from', authored, '--json', js])
        doc = json.load(open(evidence(js, 'check_complete --json')))
        why = doc.get('reason') or ''
        check('UNSOUND on the size floors, as before',
              code == CC.UNSOUND and doc.get('verdict') == 'UNSOUND',
              f"{code} {doc.get('verdict')}")
        check('each relaxed size row is counted in the reason',
              'via diameter 0.5 -> 0.45 (2 vias below it)' in why
              and '(2 drilled holes below it)' in why, why[:400])
        check('no KiCad grade under the authored project is run (#1580: the '
              'counts, not a re-grade at rules the chain relaxed)',
              'authored_drc' not in doc, str(sorted(doc))[:300])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

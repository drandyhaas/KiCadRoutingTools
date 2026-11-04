#!/usr/bin/env python3
"""#1390: a .kicad_dru track_width rule with an unmodelled condition
(insideCourtyard()) was graded by nothing.

check_drc noted the rule's clearance half as skipped and said nothing about
its track_width; kicad_drc_compare dropped every KiCad error type outside its
copper classes (`track_width` among them) and printed CONSISTENT; board_score
and check_complete read DONE while kicad-cli reported the errors. Now:

  1. check_drc prints `NOT GRADED (size)` for each rule with a size
     constraint (track_width, via_diameter, hole_size, annular_width, ...)
     whose condition it cannot evaluate; a modelled rule prints nothing.
  2. board_score lists `undersized.dru_rules` as ungraded and carries the rule
     names in components.undersized.ungraded_rules.
  3. check_complete is INCOMPLETE naming the rule (a declaration nothing
     grades), not DONE.
  4. kicad_drc_compare names every KiCad error type no channel classifies
     (compare_board_data -> kicad_unclassified, printed by compare_board).

    python3 tests/test_1390_unmodelled_size_rule_disclosed.py
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
for _p in ('py_router', 'py_tools', os.path.join('tests', 'stress')):
    sys.path.insert(0, os.path.join(ROOT, _p))

import run_utils                                               # noqa: E402
from copy_board import copy_board                              # noqa: E402

SRC = os.path.join(ROOT, 'kicad_files', 'qfn_interior_pads.kicad_pcb')
failures = []

UNMODELLED = '''(version 1)
(rule "QFN fanout"
   (layer outer)
   (constraint track_width (min 0.2mm))
   (constraint clearance (min 0.1mm))
   (condition "A.insideCourtyard('U1')"))
'''
MODELLED = '''(version 1)
(rule "wide power"
   (constraint track_width (min 0.2mm))
   (condition "A.NetClass == 'Power'"))
'''


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(tmp, name, dru):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(SRC, dst)
    with open(dst[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as fh:
        fh.write(dru)
    return dst


def run(*argv):
    r = subprocess.run([sys.executable, '-X', 'utf8', *argv], cwd=ROOT,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    return r.returncode, (r.stdout or '') + (r.stderr or '')


def main():
    tmp = tempfile.mkdtemp(prefix='t1390_')
    try:
        bad = stage(tmp, 'bad.kicad_pcb', UNMODELLED)
        _rc, out = run(os.path.join(ROOT, 'py_router', 'check_drc.py'), bad,
                       '--clearance-margin', '0.1')
        check("check_drc: NOT GRADED (size) names the rule and its kind",
              "NOT GRADED (size): rule 'QFN fanout' (track_width)" in out,
              out[-500:])
        good = stage(tmp, 'good.kicad_pcb', MODELLED)
        _rc, out = run(os.path.join(ROOT, 'py_router', 'check_drc.py'), good,
                       '--clearance-margin', '0.1')
        check('control: a modelled size rule prints no such note',
              'NOT GRADED (size)' not in out, out[-300:])

        js = os.path.join(tmp, 'score.json')
        run(os.path.join(ROOT, 'py_tools', 'board_score.py'), bad, '--quiet',
            '--json', js)
        run_utils.evidence(js, 'board_score json')
        with open(js, encoding='utf-8') as fh:
            sc = json.load(fh)
        check('board_score: undersized.dru_rules is ungraded',
              'undersized.dru_rules' in sc['ungraded'], str(sc['ungraded']))
        check('...and the rule is named in components.undersized',
              sc['components']['undersized'].get('ungraded_rules') == ['QFN fanout'],
              str(sc['components']['undersized']))
        js = os.path.join(tmp, 'score_good.json')
        run(os.path.join(ROOT, 'py_tools', 'board_score.py'), good, '--quiet',
            '--json', js)
        run_utils.evidence(js, 'board_score json')
        with open(js, encoding='utf-8') as fh:
            sc = json.load(fh)
        check('control: nothing ungraded by a modelled rule',
              'undersized.dru_rules' not in sc['ungraded']
              and not sc['components']['undersized'].get('ungraded_rules'),
              str(sc['ungraded']))

        cj = os.path.join(tmp, 'cc.json')
        run(os.path.join(ROOT, 'check_complete.py'), bad, '--skip-slow',
            '--json', cj)
        run_utils.evidence(cj, 'check_complete json')
        with open(cj, encoding='utf-8') as fh:
            doc = json.load(fh)
        check('check_complete: INCOMPLETE, naming the rule',
              doc['verdict'] == 'INCOMPLETE'
              and ".kicad_dru size rule(s) nothing grades: 'QFN fanout'"
              in doc['reason'], doc['reason'][:300])

        # kicad_drc_compare: unclassified error types are named, not dropped
        import kicad_drc_compare as kdc
        data = {'violations': [
            {'type': 'track_width', 'severity': 'error', 'items': []},
            {'type': 'track_width', 'severity': 'error', 'items': []},
            {'type': 'clearance', 'severity': 'error', 'items': []},
            {'type': 'silk_overlap', 'severity': 'warning', 'items': []}]}
        check('unclassified_error_types: errors outside every class, counted',
              kdc.unclassified_error_types(data) == {'track_width': 2},
              str(kdc.unclassified_error_types(data)))
        saved = kdc.kicad_items_for

        def fake(board, clearance=None):
            kdc.LAST_UNCLASSIFIED.clear()
            kdc.LAST_UNCLASSIFIED.update({'track_width': 20})
            return [], None
        kdc.kicad_items_for = fake
        try:
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                d = kdc.compare_board_data(SRC)
                kdc.compare_board(SRC)
        finally:
            kdc.kicad_items_for = saved
        check('compare_board_data carries them',
              d.get('kicad_unclassified') == {'track_width': 20}, str(d.get('kicad_unclassified')))
        check('compare_board prints them next to the verdict',
              'NOT CLASSIFIED' in buf.getvalue() and 'track_width=20' in buf.getvalue(),
              buf.getvalue()[-300:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""#1210: check_drc grades at max(Default class, Board Setup min_clearance),
as KiCad does, and a route below the board's minimum clearance says so.

KiCad floors a net-class clearance at rules.min_clearance (the repo's own
design_rules.resolve step 4, measured against KiCad 10.0.0). check_drc took
the Default class alone: multichannel_mixer (class 0.2, min_clearance 0.3)
read clean while kicad-cli reported 12 clearance errors, and board_score
inherited it. route.py routed that board at 0.2 and lowered min_clearance to
0.2, disclosing it only on the writeback line.

Checks:
  1. project_grading_clearance: the larger of the two, with its source; a 0
     is unset.
  2. The issue's repro on tracked routed_output: min_clearance raised 0.09 ->
     0.12 grades at 0.12 ("board minimum clearance") and finds more than the
     unchanged board does.
  3. route.py with no --clearance on splitflap_driver under a project with a
     0.2 Default class and min_clearance 0.3: the console says so, and
     JSON_SUMMARY design_rules.narrowed carries a run-wide clearance row
     (requested 0.3, delivered 0.2); the Design rules line names it.
  4. An explicit --clearance 0.15 says so too, but records no row.

    python3 tests/test_1210_min_clearance_graded.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402
from fix_kicad_drc_settings import project_grading_clearance   # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def proj(cls, mc):
    return {"board": {"design_settings": {"rules": {"min_clearance": mc}}},
            "net_settings": {"classes": [{"name": "Default", "clearance": cls}]}}


def drc(board):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', board],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    out = r.stdout + r.stderr
    m = re.search(r'FOUND (\d+) DRC VIOLATIONS', out)
    n = int(m.group(1)) if m else (0 if 'NO DRC VIOLATIONS' in out else None)
    line = next((l for l in out.splitlines() if l.startswith('Grading at clearance')), '')
    return n, line


def route(inp, out, extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/route.py', inp, out,
                        '--nets', '*'] + extra,
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    log = r.stdout + r.stderr
    line = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
    summ = json.loads(line.split(':', 1)[1]) if line else {}
    return log, summ


def main():
    # 1. Unit.
    cases = [((0.2, 0.3), (0.3, 'board minimum clearance')),
             ((0.2, 0.0), (0.2, 'Default net class')),
             ((0.2, 0.1), (0.2, 'Default net class')),
             ((0.2, 0.2), (0.2, 'Default net class'))]
    for (c, m), want in cases:
        got = project_grading_clearance(proj(c, m))
        check(f'grading clearance of class {c} / min {m}', got == want, str(got))

    tmp = tempfile.mkdtemp(prefix='t1210_')
    try:
        # 2. The issue's repro.
        src = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
        a, b = os.path.join(tmp, 'ro_a.kicad_pcb'), os.path.join(tmp, 'ro_b.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(src, a)
            copy_board(src, b)
        pb = os.path.splitext(b)[0] + '.kicad_pro'
        d = json.load(open(pb))
        rules = d['board']['design_settings']['rules']
        check('precondition: routed_output declares min_clearance 0.09',
              abs(rules.get('min_clearance', 0) - 0.09) < 1e-9, str(rules.get('min_clearance')))
        rules['min_clearance'] = 0.12
        json.dump(d, open(pb, 'w'), indent=2)
        n_a, line_a = drc(a)
        n_b, line_b = drc(b)
        check('the raised board grades at its minimum clearance',
              line_b.startswith('Grading at clearance 0.12 mm')
              and 'board minimum clearance' in line_b, line_b)
        check('the unchanged board still grades at its class', '0.09' in line_a, line_a)
        check('the stricter grade finds more', n_a is not None and n_b is not None
              and n_b > n_a, f'{n_a} -> {n_b}')

        # 3. / 4. route.py.
        sp = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
        for tag, extra in (('own', []), ('asked', ['--clearance', '0.15'])):
            inp = os.path.join(tmp, f'sp_{tag}.kicad_pcb')
            with contextlib.redirect_stdout(io.StringIO()):
                copy_board(sp, inp)
            json.dump({"board": {"design_settings": {"rules": {
                           "min_clearance": 0.3, "min_track_width": 0.1,
                           "min_via_diameter": 0.4, "min_through_hole_diameter": 0.2}}},
                       "net_settings": {"classes": [{
                           "name": "Default", "clearance": 0.2, "track_width": 0.25,
                           "via_diameter": 0.6, "via_drill": 0.3}]},
                       "meta": {"version": 1}},
                      open(os.path.splitext(inp)[0] + '.kicad_pro', 'w'), indent=2)
            log, summ = route(inp, os.path.join(tmp, f'sp_{tag}_out.kicad_pcb'), extra)
            note = next((l.strip() for l in log.splitlines()
                         if "below the board's minimum clearance" in l), '')
            rows = [r for r in (summ.get('design_rules') or {}).get('narrowed') or []
                    if r.get('kind') == 'clearance' and r.get('net') is None]
            check(f'[{tag}] the console names the relaxation', '0.3mm' in note, note)
            if tag == 'own':
                check('[own] design_rules.narrowed carries the run-wide row',
                      len(rows) == 1 and rows[0]['requested'] == 0.3
                      and rows[0]['delivered'] == 0.2, json.dumps(rows))
                dline = next((l for l in log.splitlines() if 'Design rules [' in l), '')
                check('[own] the Design rules line names it',
                      'below the board minimum clearance (Board Setup) 0.3 mm' in dline, dline)
            else:
                check('[asked] an explicit lower clearance is named, not counted',
                      not rows and 'asked' in note, json.dumps(rows))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())

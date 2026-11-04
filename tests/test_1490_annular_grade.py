"""#1490 (annular half): check_drc grades a via's ring against the board's own
annular minimums, as KiCad does.

The ring was graded against the fab floor's ring alone (#1433), so Board
Setup's `min_via_annular_width` and every .kicad_dru `annular_width` rule
were graded by nothing while KiCad reported them. The ring floor is now
resolved per via through `design_rules` -- Board Setup's minimum, then each
binding rule (NetClass, Type), at KiCad's value (never this tool's fab
profile, which no routing step enforces), and never below the fab ring.

Measured on the issue's reproductions: 23 / 47 / 2 rows against kicad-cli's
23 'hs ring' / 47 annular_width / 2 'via_ring'.

    python3 tests/test_1490_annular_grade.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
PY = [sys.executable, '-X', 'utf8']
failures = []

BOARD = '''(kicad_pcb (version 20240108) (generator pcbnew)
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "") (net 1 "/HS_A") (net 2 "/LOW")
  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default)) (layer "Edge.Cuts"))
  (via (at 10 10) (size 0.45) (drill 0.25) (layers "F.Cu" "B.Cu") (net 1))
  (via (at 30 20) (size 0.45) (drill 0.25) (layers "F.Cu" "B.Cu") (net 2))
)
'''


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def project(path, annular, classes=False):
    d = {'board': {'design_settings': {'rules': {'min_via_annular_width': annular}}},
         'net_settings': {'meta': {'version': 4},
                          'classes': [{'name': 'Default', 'clearance': 0.2, 'track_width': 0.2}]
                          + ([{'name': 'HS', 'clearance': 0.2, 'track_width': 0.2,
                               'priority': 0}] if classes else []),
                          'netclass_patterns': ([{'netclass': 'HS', 'pattern': '/HS_*'}]
                                                if classes else [])}}
    with open(os.path.splitext(path)[0] + '.kicad_pro', 'w') as fh:
        json.dump(d, fh)


def rings(path):
    r = subprocess.run(PY + [os.path.join('py_router', 'check_drc.py'), path,
                             '--max-print', '0'], capture_output=True, text=True, cwd=REPO)
    return sorted(ln.split()[0] for ln in r.stdout.splitlines()
                  if 'annular ring too small' in ln)


def main():
    td = tempfile.mkdtemp(prefix='krt1490a_')
    try:
        b = os.path.join(td, 'b.kicad_pcb')
        with open(b, 'w') as fh:
            fh.write(BOARD)
        # both vias ring 0.1 mm (0.45 / 0.25)
        project(b, 0.05)
        check('Board Setup 0.05: both rings (0.1) pass', rings(b) == [], f'{rings(b)}')
        project(b, 0.15)
        check('Board Setup 0.15: both rings (0.1) fail', rings(b) == ['Via:/HS_A', 'Via:/LOW'],
              f'{rings(b)}')
        project(b, 0.05, classes=True)
        with open(os.path.join(td, 'b.kicad_dru'), 'w') as fh:
            fh.write('(version 1)\n(rule "hs ring" (constraint annular_width (min 0.2mm))\n'
                     '  (condition "A.NetClass == \'HS\'"))\n')
        check("a NetClass 'HS' rule at 0.2: only the HS via fails", rings(b) == ['Via:/HS_A'],
              f'{rings(b)}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())

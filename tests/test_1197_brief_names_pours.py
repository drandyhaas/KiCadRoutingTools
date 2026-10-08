#!/usr/bin/env python3
"""#1197: board_brief and board_context name an existing copper pour.

Neither reported zones: sonde_u, with a GND zone over 96 % of the board, read
`state: placed; copper no (0 segs, 0 vias)` and nothing in the console, the
--json file or board_context --md said "zone", "pour" or "plane". The pour
decides the routing plan (route.py's finalize serves its net from fill), and
a campaign agent found it 30 minutes in.

Checks (tracked boards):
  1. sonde_u: `pours` names GND on B.Cu, filled, >= 90 % of the board; the
     console prints the pours line and `1 pour` in the copper line; the
     JSON_SUMMARY carries poured_nets ["GND"].
  2. interf_u_plane: two unfilled pours, GND and VCC.
  3. watchy: keep-out rule areas are counted, never listed as pours.
  4. board_context --md names the pour too.

    python3 tests/test_1197_brief_names_pours.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def brief(board, tmp):
    js = os.path.join(tmp, os.path.basename(board) + '.json')
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/board_brief.py',
                        os.path.join('kicad_files', board), '--json', js],
                       cwd=ROOT, capture_output=True, text=True,
                       encoding='utf-8', errors='replace')
    doc = json.load(open(js, encoding='utf-8')) if os.path.isfile(js) else {}
    line = next((l for l in r.stdout.splitlines()
                 if l.startswith('JSON_SUMMARY:')), '')
    summ = json.loads(line.split(':', 1)[1]) if line else {}
    return r.stdout, doc, summ


def main():
    tmp = tempfile.mkdtemp(prefix='t1197_')
    try:
        # 1. sonde_u
        out, doc, summ = brief('sonde_u.kicad_pcb', tmp)
        pours = (doc.get('pours') or {}).get('pours') or []
        gnd = next((p for p in pours if p.get('net') == 'GND'), {})
        check('sonde_u: GND poured on B.Cu, filled',
              gnd.get('layers') == ['B.Cu'] and gnd.get('filled') is True,
              json.dumps(gnd))
        check('sonde_u: the pour covers >= 90 % of the board',
              (gnd.get('area_fraction') or 0) >= 0.9, str(gnd.get('area_fraction')))
        check('sonde_u: the console names the pour',
              'pours: GND on B.Cu (filled' in out and '1 pour)' in out,
              next((l for l in out.splitlines() if 'state:' in l), ''))
        check('sonde_u: JSON_SUMMARY carries poured_nets',
              summ.get('poured_nets') == ['GND'] and summ.get('has_copper') is False,
              str(summ.get('poured_nets')))

        # 2. interf_u_plane
        out, doc, summ = brief('interf_u_plane.kicad_pcb', tmp)
        pours = (doc.get('pours') or {}).get('pours') or []
        check('interf_u_plane: two unfilled pours, GND and VCC',
              sorted(p['net'] for p in pours) == ['GND', 'VCC']
              and all(p.get('filled') is False for p in pours),
              json.dumps(pours)[:300])

        # 3. watchy: rule areas are counted apart
        out, doc, summ = brief('watchy.kicad_pcb', tmp)
        pc = doc.get('pours') or {}
        check('watchy: keep-out rule areas counted, no pour listed',
              pc.get('keepout_areas') == 5 and not pc.get('pours')
              and 'keep-out rule areas: 5' in out, json.dumps(pc))

        # 4. board_context
        r = subprocess.run([sys.executable, '-X', 'utf8', 'py_tools/board_context.py',
                            'kicad_files/sonde_u.kicad_pcb', '--md'], cwd=ROOT,
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace')
        check('board_context --md names the pour',
              'Copper pours: GND on B.Cu (filled' in r.stdout,
              next((l for l in r.stdout.splitlines() if 'pours' in l.lower()), ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
